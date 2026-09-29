/**
 * @file main_sil.c
 * @brief 固件在环（SIL）闭环主循环与实验 E1~E5（方案 Phase 3 核心交付）。
 *
 * 关键设计：本程序与单片机固件共享同一份 App_Car.c 与 Service 目录源文件，
 * 通过编译期替换 Port 实现（port_sim.c）把真实控制代码接进倒立摆模型——
 * 跑的就是要烧进芯片的那份算法，这是无硬件条件下最强的验证形态。
 *
 * 实验清单（对应方案 §3 Phase 3 验收）：
 *   E1  初始倾角 10° 的直立恢复 → 收敛时间/超调（要求 3s 内进 ±0.5°）
 *   E2  t=2s 施加速度冲击 → 抗扰恢复时间（要求 2s 内回到 ±0.5°）
 *   E3  静态台架：卡尔曼 vs 互补滤波 vs 纯积分（陀螺零偏漂移对比）
 *   E4  balance_kp 参数扫描 {-400..-1040} → 超调/收敛/振荡趋势
 *   E5  控制周期抖动敏感性 {0,1,2,5}ms → 证明"确定性周期"的价值
 *
 * 用法：
 *   sil_sim <e1|e2|e3|e4|e5|all> [outdir] [--check]
 *   --check 时按验收标准决定退出码（供 CTest 门禁用）。
 *
 * 所有随机过程（噪声、抖动、冲击时刻）均为确定性种子，实验可复现。
 */
#include "App_Car.h"
#include "port_sim.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <direct.h>
#define MKDIR(p) _mkdir(p)
#else
#include <sys/stat.h>
#define MKDIR(p) mkdir((p), 0755)
#endif

#define DT_NOM (0.01) /* 固件采样/控制周期（与 CAR_SAMPLE_PERIOD_S 绑定） */
#define E1_TILT0 (10.0)
#define E1_T_TOTAL (6.0)
#define E1_SETTLE_BAND (0.5)
#define E1_SETTLE_LIMIT (3.0) /* 验收：3 秒内进入 ±0.5° 且保持 */
#define E2_IMPULSE_T (2.0)
#define E2_IMPULSE_DV (0.3)
#define E2_T_TOTAL (8.0)
#define E2_RECOVER_LIMIT (2.0)
#define E3_T_TOTAL (30.0)
#define E3_FREEZE_THETA (10.0)
#define E3_COMP_ALPHA (0.99) /* 互补滤波权重：0.99 陀螺 + 0.01 加速度 */

/* ======== 轨迹记录 ======== */
typedef struct {
    int n;
    int cap;
    double *t;
    double *theta;  /* 真值倾角 */
    double *accel;  /* 加速度计角度（固件输入） */
    double *gyro_y; /* 陀螺角速度（固件输入） */
    double *kalman; /* 固件卡尔曼输出 */
    double *u;      /* PWM 指令 */
    double *v;      /* 车速 */
} trace_t;

/* ======== 实验配置 ======== */
typedef struct {
    const char *name;
    double theta0;
    double t_total;
    double jitter_amp_ms; /* E5：周期抖动幅度（±ms） */
    double impulse_t;     /* E2：冲击时刻（<0 = 无） */
    double impulse_dv;
    int freeze;           /* E3：台架冻结 */
    double freeze_theta;
} run_cfg_t;

/* ======== 收敛指标 ======== */
typedef struct {
    int stable;          /* 收敛到 ±band 且保持到结束 */
    double settle_t;     /* 进入并保持 ±band 的时刻（未收敛 = -1） */
    double overshoot;    /* 过零后的反向最大偏差（度） */
    double rms_end;      /* 结尾窗口 RMS 误差（度） */
    double max_end;      /* 结尾窗口最大 |θ|（度） */
} metrics_t;

static double g_jitter_rng = 0.13579;

static double JitterU(void) /* 确定性抖动序列，[-0.5, 0.5) */
{
    g_jitter_rng = fmod(g_jitter_rng * 9973.0 + 0.731, 1.0);
    return g_jitter_rng - 0.5;
}

static void Trace_Init(trace_t *tr, int cap)
{
    tr->n = 0;
    tr->cap = cap;
    tr->t = (double *)malloc(sizeof(double) * cap);
    tr->theta = (double *)malloc(sizeof(double) * cap);
    tr->accel = (double *)malloc(sizeof(double) * cap);
    tr->gyro_y = (double *)malloc(sizeof(double) * cap);
    tr->kalman = (double *)malloc(sizeof(double) * cap);
    tr->u = (double *)malloc(sizeof(double) * cap);
    tr->v = (double *)malloc(sizeof(double) * cap);
    if (!tr->t || !tr->theta || !tr->accel || !tr->gyro_y ||
        !tr->kalman || !tr->u || !tr->v)
    {
        fprintf(stderr, "FATAL: trace malloc failed\n");
        exit(2);
    }
}

static void Trace_Free(trace_t *tr)
{
    free(tr->t);
    free(tr->theta);
    free(tr->accel);
    free(tr->gyro_y);
    free(tr->kalman);
    free(tr->u);
    free(tr->v);
}

static void Trace_Push(trace_t *tr, double t, double theta, double accel,
                       double gyro_y, double kalman, double u, double v)
{
    if (tr->n >= tr->cap)
    {
        fprintf(stderr, "FATAL: trace overflow\n");
        exit(2);
    }
    tr->t[tr->n] = t;
    tr->theta[tr->n] = theta;
    tr->accel[tr->n] = accel;
    tr->gyro_y[tr->n] = gyro_y;
    tr->kalman[tr->n] = kalman;
    tr->u[tr->n] = u;
    tr->v[tr->n] = v;
    tr->n++;
}

/* ======== 核心：跑一次闭环 ======== */
static void RunClosedLoop(const run_cfg_t *cfg, trace_t *tr)
{
    double t = 0.0;
    int impulse_done = 0;

    /* 每次实验等价于一次上电：应用层状态（卡尔曼/速度环/协议）全复位。 */
    App_Car_Init();
    PortSim_Reset(cfg->theta0, 0xC0FFEEu);
    if (cfg->freeze)
    {
        PortSim_Freeze(1, cfg->freeze_theta);
    }
    g_jitter_rng = 0.13579;

    while (t < cfg->t_total)
    {
        double theta_true, v, x;
        float accel_f, gyro_f;
        double dt = DT_NOM;

        /* ---- 与固件完全相同的控制节拍 ---- */
        App_Car_GetAngle(); /* 读 IMU/编码器 + 卡尔曼 */
        App_Car_PID();      /* 三环 PID → port_motor_set */

        /* ---- 记录本拍 ---- */
        PortSim_GetTruth(&theta_true, &v, &x);
        PortSim_GetLastSensors(&accel_f, &gyro_f);
        Trace_Push(tr, t, theta_true, (double)accel_f, (double)gyro_f,
                   (double)App_Car_GetAttitude(),
                   (double)PortSim_GetLastPwmA(), v);

        /* ---- E2：整点冲击 ---- */
        if (!impulse_done && cfg->impulse_t >= 0.0 && t >= cfg->impulse_t)
        {
            PortSim_ApplyImpulse(cfg->impulse_dv);
            impulse_done = 1;
        }

        /* ---- E5：物理时间带抖动，而固件 dt 恒为 10ms（错配即被测对象） ---- */
        if (cfg->jitter_amp_ms > 0.0)
        {
            dt += 2.0 * JitterU() * cfg->jitter_amp_ms / 1000.0;
            if (dt < 0.002)
            {
                dt = 0.002;
            }
        }

        PortSim_Step(dt);
        t += dt;
    }
}

static int Trace_WriteCsv(const trace_t *tr, const char *dir, const char *name,
                          int with_e3_cols, const double *comp, const double *pure)
{
    char path[512];
    FILE *f;
    int i;

    snprintf(path, sizeof(path), "%s/%s.csv", dir, name);
    f = fopen(path, "w");
    if (!f)
    {
        fprintf(stderr, "FATAL: cannot open %s\n", path);
        return 0;
    }
    if (with_e3_cols)
    {
        fprintf(f, "t,theta_true,accel,gyro_y,kalman,comp,pure,u,v\n");
        for (i = 0; i < tr->n; i++)
        {
            fprintf(f, "%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.1f,%.5f\n",
                    tr->t[i], tr->theta[i], tr->accel[i], tr->gyro_y[i],
                    tr->kalman[i], comp[i], pure[i], tr->u[i], tr->v[i]);
        }
    }
    else
    {
        fprintf(f, "t,theta_true,accel,gyro_y,kalman,u,v\n");
        for (i = 0; i < tr->n; i++)
        {
            fprintf(f, "%.4f,%.5f,%.5f,%.5f,%.5f,%.1f,%.5f\n",
                    tr->t[i], tr->theta[i], tr->accel[i], tr->gyro_y[i],
                    tr->kalman[i], tr->u[i], tr->v[i]);
        }
    }
    fclose(f);
    return 1;
}

/* ======== 指标计算 ======== */
static metrics_t ComputeMetrics(const trace_t *tr, double band, int from_idx)
{
    metrics_t m;
    int i;
    int last_out = -1;
    double end_window = 1.5; /* 结尾统计窗口（秒） */
    double theta0_sign;
    int cross_idx = -1;
    double acc = 0.0;
    int n_end = 0;

    m.stable = 0;
    m.settle_t = -1.0;
    m.overshoot = 0.0;
    m.rms_end = 0.0;
    m.max_end = 0.0;
    if (tr->n == 0)
    {
        return m;
    }

    /* 1. 从后向前找"持续在带内"的起点。 */
    for (i = tr->n - 1; i >= from_idx; i--)
    {
        if (fabs(tr->theta[i]) > band)
        {
            last_out = i;
            break;
        }
    }
    if (last_out < tr->n - 1)
    {
        /* 尾部在带内：settle = 最后一次出带的下一拍（起点也在带内则为 0 时刻）。 */
        int idx = last_out + 1;
        if (last_out < from_idx)
        {
            idx = from_idx;
        }
        m.settle_t = tr->t[idx];
        m.stable = 1;
    }

    /* 2. 过零后的反向超调。 */
    theta0_sign = (tr->theta[from_idx] >= 0.0) ? 1.0 : -1.0;
    for (i = from_idx + 1; i < tr->n; i++)
    {
        if (cross_idx < 0 && tr->theta[i] * theta0_sign <= 0.0)
        {
            cross_idx = i;
        }
        if (cross_idx >= 0)
        {
            double opp = -theta0_sign * tr->theta[i];
            if (opp > m.overshoot)
            {
                m.overshoot = opp;
            }
        }
    }

    /* 3. 结尾窗口 RMS / max。 */
    for (i = tr->n - 1; i >= from_idx; i--)
    {
        if (tr->t[i] < tr->t[tr->n - 1] - end_window)
        {
            break;
        }
        acc += tr->theta[i] * tr->theta[i];
        n_end++;
        if (fabs(tr->theta[i]) > m.max_end)
        {
            m.max_end = fabs(tr->theta[i]);
        }
    }
    if (n_end > 0)
    {
        m.rms_end = sqrt(acc / (double)n_end);
    }

    /* 4. 未收敛或明显发散（倾角超 60° = 摔了）都算不稳定。 */
    for (i = 0; i < tr->n; i++)
    {
        if (fabs(tr->theta[i]) > 60.0)
        {
            m.stable = 0;
            m.settle_t = -1.0;
            break;
        }
    }
    if (m.settle_t < 0.0)
    {
        m.stable = 0;
    }
    return m;
}

/* ======== 实验 ======== */
static int DoE1(const char *dir, int check)
{
    run_cfg_t cfg;
    trace_t tr;
    metrics_t m;

    memset(&cfg, 0, sizeof(cfg));
    cfg.name = "e1";
    cfg.theta0 = E1_TILT0;
    cfg.t_total = E1_T_TOTAL;
    cfg.impulse_t = -1.0;

    Trace_Init(&tr, 2048);
    RunClosedLoop(&cfg, &tr);
    if (!Trace_WriteCsv(&tr, dir, "e1", 0, NULL, NULL))
    {
        return 2;
    }
    m = ComputeMetrics(&tr, E1_SETTLE_BAND, 0);
    printf("[E1] 初始倾角 %.0f 度直立恢复: settle=%s%.2fs overshoot=%.2f deg "
           "end_rms=%.3f deg -> %s\n",
           E1_TILT0, m.settle_t < 0 ? "NOT-SETTLED " : "", m.settle_t,
           m.overshoot, m.rms_end, m.stable ? "STABLE" : "UNSTABLE");
    Trace_Free(&tr);

    if (check && !(m.stable && m.settle_t >= 0.0 && m.settle_t <= E1_SETTLE_LIMIT))
    {
        fprintf(stderr, "E1 FAIL: 需 %.0fs 内进入 ±%.1f° 并保持\n",
                E1_SETTLE_LIMIT, E1_SETTLE_BAND);
        return 1;
    }
    return 0;
}

static int DoE2(const char *dir, int check)
{
    run_cfg_t cfg;
    trace_t tr;
    metrics_t m;
    int from_idx = 0;
    int i;

    memset(&cfg, 0, sizeof(cfg));
    cfg.name = "e2";
    cfg.theta0 = 0.0;
    cfg.t_total = E2_T_TOTAL;
    cfg.impulse_t = E2_IMPULSE_T;
    cfg.impulse_dv = E2_IMPULSE_DV;

    Trace_Init(&tr, 4096);
    RunClosedLoop(&cfg, &tr);
    if (!Trace_WriteCsv(&tr, dir, "e2", 0, NULL, NULL))
    {
        return 2;
    }
    for (i = 0; i < tr.n; i++)
    {
        if (tr.t[i] >= E2_IMPULSE_T)
        {
            from_idx = i;
            break;
        }
    }
    m = ComputeMetrics(&tr, E1_SETTLE_BAND, from_idx);
    /* settle_t 是绝对时刻，恢复时长 = 绝对时刻 - 冲击时刻。 */
    {
        double recover_t = (m.settle_t < 0.0) ? -1.0 : (m.settle_t - E2_IMPULSE_T);
        printf("[E2] t=%.1fs 速度冲击 %.1f m/s 后恢复: recover=%s%.2fs "
               "overshoot=%.2f deg -> %s\n",
               E2_IMPULSE_T, E2_IMPULSE_DV,
               recover_t < 0 ? "NOT-RECOVERED " : "", recover_t,
               m.overshoot, m.stable ? "STABLE" : "UNSTABLE");
        Trace_Free(&tr);
        if (check && !(m.stable && recover_t >= 0.0 && recover_t <= E2_RECOVER_LIMIT))
        {
            fprintf(stderr, "E2 FAIL: 需 %.0fs 内恢复到 ±%.1f°\n",
                    E2_RECOVER_LIMIT, E1_SETTLE_BAND);
            return 1;
        }
        return 0;
    }
}

static int DoE3(const char *dir, int check)
{
    run_cfg_t cfg;
    trace_t tr;
    double *comp, *pure;
    double rms[3] = {0, 0, 0}; /* kalman / comp / pure */
    double truth_sensor;
    double acc[3] = {0, 0, 0};
    int n_end = 0;
    int i;
    double end_window = 10.0;
    int rc = 0;

    memset(&cfg, 0, sizeof(cfg));
    cfg.name = "e3";
    cfg.theta0 = E3_FREEZE_THETA;
    cfg.t_total = E3_T_TOTAL;
    cfg.impulse_t = -1.0;
    cfg.freeze = 1;
    cfg.freeze_theta = E3_FREEZE_THETA;

    Trace_Init(&tr, 8192);
    RunClosedLoop(&cfg, &tr);

    comp = (double *)calloc((size_t)tr.n, sizeof(double));
    pure = (double *)calloc((size_t)tr.n, sizeof(double));
    if (!comp || !pure)
    {
        return 2;
    }

    /* 台架真值（传感器坐标系）：倾角 + 安装偏角。 */
    truth_sensor = E3_FREEZE_THETA + PORT_SIM_MOUNT_OFFSET_DEG;

    /* 用与固件同源的观测离线复算互补滤波与纯积分（对照组）。 */
    for (i = 1; i < tr.n; i++)
    {
        pure[i] = pure[i - 1] + tr.gyro_y[i] * DT_NOM;
        comp[i] = E3_COMP_ALPHA * (comp[i - 1] + tr.gyro_y[i] * DT_NOM)
                  + (1.0 - E3_COMP_ALPHA) * tr.accel[i];
    }
    /* 结尾 10 秒的误差统计。 */
    for (i = 0; i < tr.n; i++)
    {
        if (tr.t[i] < tr.t[tr.n - 1] - end_window)
        {
            continue;
        }
        acc[0] += (tr.kalman[i] - truth_sensor) * (tr.kalman[i] - truth_sensor);
        acc[1] += (comp[i] - truth_sensor) * (comp[i] - truth_sensor);
        acc[2] += (pure[i] - truth_sensor) * (pure[i] - truth_sensor);
        n_end++;
    }
    if (n_end > 0)
    {
        for (i = 0; i < 3; i++)
        {
            rms[i] = sqrt(acc[i] / (double)n_end);
        }
    }

    if (!Trace_WriteCsv(&tr, dir, "e3", 1, comp, pure))
    {
        rc = 2;
        goto out;
    }
    printf("[E3] 静态台架(%.0f 度, 陀螺零偏) 结尾 10s RMS 误差:\n"
           "     卡尔曼=%.3f  互补(α=%.2f)=%.3f  纯积分=%.3f 度 -> ",
           E3_FREEZE_THETA, rms[0], E3_COMP_ALPHA, rms[1], rms[2]);
    /* 验收：卡尔曼显著优于另两者（漂移被零偏估计压住）。 */
    if (check && !(rms[0] < rms[1] && rms[0] < rms[2] * 0.5))
    {
        printf("FAIL\n");
        rc = 1;
    }
    else
    {
        printf("KALMAN-BEST\n");
    }

out:
    free(comp);
    free(pure);
    Trace_Free(&tr);
    return rc;
}

/* E4/E5 共用的多场景执行器：逐场景跑并写轨迹 + 汇总行。 */
typedef struct {
    double key;       /* 扫描变量（kp 或抖动 ms） */
    char traj[32];    /* 轨迹文件名（不含扩展名） */
    metrics_t m;
} sweep_row_t;

static int DoE4(const char *dir, int check)
{
    static const double kps[] = {-400.0, -560.0, -720.0, -880.0, -1040.0};
    const int n = (int)(sizeof(kps) / sizeof(kps[0]));
    sweep_row_t rows[8];
    float saved_kp = g_car_params.balance_kp;
    char path[512];
    FILE *csv;
    int i, rc = 0;
    int stable_cnt = 0;

    for (i = 0; i < n; i++)
    {
        run_cfg_t cfg;
        trace_t tr;

        memset(&cfg, 0, sizeof(cfg));
        cfg.theta0 = E1_TILT0;
        cfg.t_total = E1_T_TOTAL;
        cfg.impulse_t = -1.0;

        g_car_params.balance_kp = (float)kps[i];
        Trace_Init(&tr, 2048);
        RunClosedLoop(&cfg, &tr);

        snprintf(rows[i].traj, sizeof(rows[i].traj), "e4_kp%04d",
                 (int)fabs(kps[i]));
        if (!Trace_WriteCsv(&tr, dir, rows[i].traj, 0, NULL, NULL))
        {
            Trace_Free(&tr);
            return 2;
        }
        rows[i].key = kps[i];
        rows[i].m = ComputeMetrics(&tr, E1_SETTLE_BAND, 0);
        printf("[E4] kp=%7.0f  settle=%.2fs overshoot=%.2f end_rms=%.3f -> %s\n",
               kps[i], rows[i].m.settle_t, rows[i].m.overshoot,
               rows[i].m.rms_end, rows[i].m.stable ? "STABLE" : "UNSTABLE");
        if (rows[i].m.stable)
        {
            stable_cnt++;
        }
        Trace_Free(&tr);
    }
    g_car_params.balance_kp = saved_kp;

    /* 汇总 CSV：供 plot.py 画"参数 vs 指标"。 */
    snprintf(path, sizeof(path), "%s/e4_summary.csv", dir);
    csv = fopen(path, "w");
    if (!csv)
    {
        return 2;
    }
    fprintf(csv, "kp,settle_t,overshoot,end_rms,stable\n");
    for (i = 0; i < n; i++)
    {
        fprintf(csv, "%.0f,%.3f,%.3f,%.4f,%d\n", rows[i].key,
                rows[i].m.settle_t, rows[i].m.overshoot, rows[i].m.rms_end,
                rows[i].m.stable);
    }
    fclose(csv);

    printf("[E4] %d/%d 组稳定（结论只讲趋势：kp 过小扶正不足、过大振荡）\n",
           stable_cnt, n);
    /* 验收：默认参数（-720，下标 2）必须稳定；各组差异由图表呈现。 */
    if (check && !rows[2].m.stable)
    {
        fprintf(stderr, "E4 FAIL: 默认 kp=-720 未收敛\n");
        rc = 1;
    }
    return rc;
}

static int DoE5(const char *dir, int check)
{
    static const double amps[] = {0.0, 1.0, 2.0, 5.0, 10.0};
    const int n = (int)(sizeof(amps) / sizeof(amps[0]));
    sweep_row_t rows[8];
    char path[512];
    FILE *csv;
    int i, rc = 0;

    for (i = 0; i < n; i++)
    {
        run_cfg_t cfg;
        trace_t tr;

        memset(&cfg, 0, sizeof(cfg));
        cfg.theta0 = E1_TILT0;
        cfg.t_total = E1_T_TOTAL;
        cfg.impulse_t = -1.0;
        cfg.jitter_amp_ms = amps[i];

        Trace_Init(&tr, 4096);
        RunClosedLoop(&cfg, &tr);

        snprintf(rows[i].traj, sizeof(rows[i].traj), "e5_j%d", (int)amps[i]);
        if (!Trace_WriteCsv(&tr, dir, rows[i].traj, 0, NULL, NULL))
        {
            Trace_Free(&tr);
            return 2;
        }
        rows[i].key = amps[i];
        rows[i].m = ComputeMetrics(&tr, E1_SETTLE_BAND, 0);
        printf("[E5] jitter=±%.0fms settle=%s%.2fs end_rms=%.3f max_end=%.2f -> %s\n",
               amps[i], rows[i].m.settle_t < 0 ? "NOT-SETTLED " : "",
               rows[i].m.settle_t, rows[i].m.rms_end, rows[i].m.max_end,
               rows[i].m.stable ? "STABLE" : "UNSTABLE");
        Trace_Free(&tr);
    }

    snprintf(path, sizeof(path), "%s/e5_summary.csv", dir);
    csv = fopen(path, "w");
    if (!csv)
    {
        return 2;
    }
    fprintf(csv, "jitter_ms,settle_t,end_rms,max_end,stable\n");
    for (i = 0; i < n; i++)
    {
        fprintf(csv, "%.1f,%.3f,%.4f,%.4f,%d\n", rows[i].key,
                rows[i].m.settle_t, rows[i].m.rms_end, rows[i].m.max_end,
                rows[i].m.stable);
    }
    fclose(csv);

    printf("[E5] 周期越确定收敛越干净（0ms 与 ±5ms 的差距即『确定性周期』的价值）\n");
    /* 验收：0 抖动必须稳定；最差档（±10ms）相对无抖动必须在收敛时间与
     * 稳态精度上同时劣化——证明周期确定性的量化价值。 */
    if (check && !(rows[0].m.stable &&
                   (rows[4].m.settle_t < 0 || rows[0].m.settle_t < rows[4].m.settle_t) &&
                   rows[4].m.rms_end > rows[0].m.rms_end))
    {
        fprintf(stderr, "E5 FAIL\n");
        rc = 1;
    }
    return rc;
}

static int RunAll(const char *dir, int check)
{
    int fail = 0;
    fail |= DoE1(dir, check);
    fail |= DoE2(dir, check);
    fail |= DoE3(dir, check);
    fail |= DoE4(dir, check);
    fail |= DoE5(dir, check);
    return fail;
}

int main(int argc, char **argv)
{
    const char *exp = (argc > 1) ? argv[1] : "e1";
    const char *dir = (argc > 2) ? argv[2] : "sim_out";
    int check = 0;
    int i;

    for (i = 3; i < argc; i++)
    {
        if (strcmp(argv[i], "--check") == 0)
        {
            check = 1;
        }
    }
    /* argv[2] 也可能是 --check（省略输出目录）。 */
    if (argc > 2 && strcmp(argv[2], "--check") == 0)
    {
        dir = "sim_out";
        check = 1;
    }

    MKDIR(dir); /* 已存在时静默失败即可 */

    if (strcmp(exp, "e1") == 0)
    {
        return DoE1(dir, check);
    }
    if (strcmp(exp, "e2") == 0)
    {
        return DoE2(dir, check);
    }
    if (strcmp(exp, "e3") == 0)
    {
        return DoE3(dir, check);
    }
    if (strcmp(exp, "e4") == 0)
    {
        return DoE4(dir, check);
    }
    if (strcmp(exp, "e5") == 0)
    {
        return DoE5(dir, check);
    }
    if (strcmp(exp, "all") == 0)
    {
        return RunAll(dir, check);
    }
    fprintf(stderr, "unknown experiment '%s' (e1|e2|e3|e4|e5|all)\n", exp);
    return 2;
}
