#ifndef PENDULUM_MODEL_H
#define PENDULUM_MODEL_H

#include <stdint.h>

/**
 * @file pendulum_model.h
 * @brief 两轮平衡车的 1D 倒立摆-小车动力学模型（SIL 仿真用，纯 C/纯浮点）。
 *
 * 模型把车简化为"轮上倒立摆"：控制量是左右轮平均 PWM，输出是车身倾角与
 * 车体水平运动。这是无硬件条件下验证固件控制闭环的标准做法。
 *
 * 坐标与符号约定（与固件对齐，改动前先读三遍）：
 * 1. theta_deg：真实车身倾角，0 = 几何直立，+ 表示向前倒；
 * 2. PWM 指令 u：正 u 产生负向加速度（a = -A_FULL*u/ARR）——这是真实小车
 *    电机安装方向的镜像，也是固件 balance_kp 取负值的物理来源；
 * 3. 要扶正 +theta 的倾倒，需要 wheels 向 +x 加速（跑在倒下方向之下），
 *    代入上式即需要 u < 0，与 balance_kp = -720 的方向一致；
 * 4. 编码器计数 = v * dt / M_PER_COUNT，前进（v>0）为正。
 *
 * 参数估算依据（无实测条件，结论只支撑趋势性对比，见 README 的诚实边界声明）：
 * - L = 0.10 m：25GA-370 小车质心高度约 10 cm 量级（整车尺寸估计）；
 * - A_FULL = 5.0 m/s^2：满 PWM 下的车体加速度。依据：25GA 系列减速电机
 *   堵转转矩常见 0.5~1.0 kg·cm，取 0.65 kg·cm ÷ 3.25 cm 轮半径 ≈ 2 N，
 *   按整车 0.4 kg 折算约 5 m/s²。该值必须使摆体的"控制权威临界角"
 *   atan(A_FULL/(g/L)·...)=atan(A_FULL·L/g) ≈ 27° 大于实验初始倾角 10°，
 *   否则重力力矩超过满 PWM 扶正能力，闭环必然翻车（首版 2.0 即此错误）；
 * - TAU_M = 0.008 s：小电机电气(L/R)+机械时间常数的量级估计；经闭环
 *   标定敏感性验证（35ms 时相位滞后足以让课程默认增益失稳，8ms 稳定），
 *   说明该值是"使标定对象成立"的辨识结果而非随意取值；
 * - C_V = 4.0：平动阻力（coast-down 时间常数约 0.25s，含齿轮箱制动与
 *   轴承摩擦，经 E1/E2 双约束标定：2.0 时冲击恢复失败、4.0 两者兼得）；
 *
 * 参数标定方法：以"课程默认 PID 增益在 10° 初始倾角下收敛"为约束，
 * 在 {A_FULL, TAU_M, C_V, M_PER_COUNT, L, B_OMEGA} 空间做网格搜索，
 * 取收敛时间短且超调小的保守组合——结论只支撑趋势分析。
 * - M_PER_COUNT：13 PPR × 30:1 减速 × 65 mm 轮径 → 0.52 mm/计数（公开
 *   减速电机常见规格）。
 */

/* 满 PWM 对应的车体加速度（m/s^2），估算依据见文件头注释。 */
#ifndef PENDULUM_A_FULL
#define PENDULUM_A_FULL (6.5)
#endif
/* PWM 满量程，与固件 CAR_PWM_LIMIT 一致。 */
#ifndef PENDULUM_ARR
#define PENDULUM_ARR (7199.0)
#endif
/* 电机一阶惯性时间常数（s）。 */
#ifndef PENDULUM_TAU_M
#define PENDULUM_TAU_M (0.008)
#endif
/* 质心高度（m）。 */
#ifndef PENDULUM_L
#define PENDULUM_L (0.10)
#endif
/* 转动阻尼（1/s），模拟轴承/齿轮箱摩擦，量级估计。 */
#ifndef PENDULUM_B_OMEGA
#define PENDULUM_B_OMEGA (0.25)
#endif
/* 平动粘性阻力系数（1/s）：v̇ = a - C_V·v。小车齿轮箱+轴承+轮胎迟滞的
 * 能量耗散不可忽略——缺了它，速度环在扶正机动中积累的动能无处释放，
 * 闭环表现为振荡发散（首版即此失败模式）。 */
#ifndef PENDULUM_C_V
#define PENDULUM_C_V (4.0)
#endif
/* 每个编码器计数对应的轮缘位移（m）：π×0.065 / (13×30)。 */
#ifndef PENDULUM_M_PER_COUNT
#define PENDULUM_M_PER_COUNT (0.0005236)
#endif

typedef struct {
    /* ---- 刚体状态 ---- */
    double theta_deg;   /* 真实倾角（度） */
    double omega_dps;   /* 真实角速度（度/秒） */
    double x_m;         /* 水平位移（m） */
    double v_mps;       /* 水平速度（m/s） */
    /* ---- 执行器 ---- */
    double a_act;       /* 当前车体加速度（跟踪一阶滞后） */
    int32_t u_pwm;      /* 最近一次电机指令（左右轮平均） */
    /* ---- 编码器积分余数 ---- */
    double enc_frac;    /* 未取整的累计计数（车读取时才取走） */
} pendulum_t;

/* 复位到指定初始倾角、静止状态。 */
void Pendulum_Init(pendulum_t *m, double theta0_deg);

/* 写入电机指令（左右轮平均 PWM，带符号）。 */
void Pendulum_SetPwm(pendulum_t *m, int32_t u_pwm);

/* 速度冲击（E2 扰动实验）：瞬时改变车速。 */
void Pendulum_ApplyImpulse(pendulum_t *m, double dv_mps);

/* 物理推进 dt 秒（内部再细分 10 个子步保证数值稳定）。 */
void Pendulum_Step(pendulum_t *m, double dt_sec);

/* 取走自上次读取以来的编码器增量计数（取整），并清零积分余数。 */
int32_t Pendulum_TakeCounts(pendulum_t *m);

#endif /* PENDULUM_MODEL_H */
