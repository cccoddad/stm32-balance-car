/**
 * @file test_attitude.c
 * @brief Service/attitude_kalman.c 单元测试：卡尔曼滤波收敛性与鲁棒性。
 *
 * 覆盖用例：
 * - 恒定输入收敛（加速度角度阶跃 → 融合角收敛到真值）
 * - 纯陀螺漂移/零偏被 Q_bias 估计修正（长时间不发散）
 * - dt 变化不发散、极端输入不产生 NaN/Inf
 *
 * 滤波增益由 Q/R 决定，收敛步数是统计性的——断言用宽松容差 + 长序列。
 */
#include "test_util.h"
#include "attitude_kalman.h"
#include "Car_Config.h"

/* 连续更新 n 拍（模拟 n 个采样周期）。 */
static void RunSteps(kalman_t *k, float accel, float gyro, int n)
{
    int i;
    for (i = 0; i < n; i++)
    {
        (void)Kalman_Update(k, accel, gyro);
    }
}

static void Test_Init(void)
{
    kalman_t k;
    Kalman_Init(&k);

    CHECK(k.angle == 0.0f);
    CHECK(k.Q_bias == 0.0f);
    CHECK_NEAR(k.dt, CAR_SAMPLE_PERIOD_S, 1e-9); /* dt 必须绑定采样周期宏 */
    CHECK(k.P00 == 1.0f);
    CHECK(k.P11 == 1.0f);
    CHECK(k.P01 == 0.0f);
    CHECK(k.Q_angle == CAR_KALMAN_Q_ANGLE);
    CHECK(k.R_angle == CAR_KALMAN_R_ANGLE);
}

static void Test_ConstantInput_Converge(void)
{
    kalman_t k;
    Kalman_Init(&k);

    /* 静态 10 度、无角速度：融合角应收敛到 10。 */
    RunSteps(&k, 10.0f, 0.0f, 1000);
    CHECK_NEAR(k.angle, 10.0, 0.01);

    /* 阶跃到 -20 度：同样收敛。
     * 实测（probe）：30 度大阶跃伴随零偏状态耦合，500 拍只到 -21.36，
     * 5000 拍后误差 < 1e-4 —— 容差与步数按实测数据标定。 */
    RunSteps(&k, -20.0f, 0.0f, 5000);
    CHECK_NEAR(k.angle, -20.0, 0.01);

    /* 角速度输出 = 陀螺 - 零偏，静态时应接近 0。 */
    CHECK_NEAR(k.angle_dot, 0.0, 0.01);
}

static void Test_Track_Speed(void)
{
    kalman_t k;
    Kalman_Init(&k);

    /* 初始误差 10 度：10 拍内应已明显向目标移动（不是原地不动）。 */
    RunSteps(&k, 10.0f, 0.0f, 10);
    CHECK(k.angle > 1.0f);  /* 至少走了 10% */
    CHECK(k.angle < 10.0f); /* 还没冲过头（欠阻尼单调逼近） */
}

static void Test_GyroDrift_Corrected(void)
{
    kalman_t k;
    float early;
    Kalman_Init(&k);

    /* 陀螺恒定 +2 度/秒但车实际静止（加速度报 0 度）：
     * 纯积分 20 秒会漂到 +40 度；滤波器应把偏差归因给零偏，
     * 长期融合角保持在 0 度附近。 */
    RunSteps(&k, 0.0f, 2.0f, 200); /* 2 秒，先看早期漂移形态 */
    early = k.angle;
    CHECK(early > 0.0f); /* 早期确实被陀螺带着走 */

    RunSteps(&k, 0.0f, 2.0f, 1800); /* 累计 20 秒 */
    CHECK(fabs(k.angle) < 2.0f);     /* 漂移被压住（纯积分会到 40） */
    CHECK(fabs(k.angle) < fabs(early) + 0.01f); /* 不比早期更差 */
    CHECK(isfinite(k.Q_bias));       /* 零偏估计有效 */
}

static void Test_DtVariation_NoDiverge(void)
{
    kalman_t k;
    int i;
    Kalman_Init(&k);

    /* 采样周期改半（等价于问题清单 #3：dt 与真实周期不符的场景），
     * 算法不能发散、不能出 NaN。 */
    k.dt = CAR_SAMPLE_PERIOD_S * 0.5f;
    for (i = 0; i < 2000; i++)
    {
        (void)Kalman_Update(&k, 5.0f, 1.0f);
    }
    CHECK(isfinite(k.angle));
    CHECK(isfinite(k.P00));
    CHECK(isfinite(k.Q_bias));
}

static void Test_ExtremeInput_NoNaN(void)
{
    kalman_t k;
    Kalman_Init(&k);

    /* 传感器饱和值 + 剧烈抖动：数值必须保持有限。 */
    RunSteps(&k, 32767.0f, 32767.0f, 100);
    CHECK(isfinite(k.angle));
    RunSteps(&k, -32767.0f, -32767.0f, 100);
    CHECK(isfinite(k.angle));
    CHECK(isfinite(k.P00));
    CHECK(isfinite(k.P11));
    CHECK(isfinite(k.Q_bias));
    /* 200 拍饱和角速度纯积分的物理量级 ~65000 度；远超此值说明数值爆炸。 */
    CHECK(fabs(k.angle) < 1000000.0f);
}

static void Test_BiasState_Recovered(void)
{
    kalman_t k;
    Kalman_Init(&k);

    /* 恒定 10 度静态：零偏估计应收敛回 0（无偏场景不产生虚假零偏）。 */
    RunSteps(&k, 10.0f, 0.0f, 2000);
    CHECK(fabs(k.Q_bias) < 0.1f);
    CHECK_NEAR(k.angle, 10.0, 0.01);
}

int main(void)
{
    Test_Init();
    Test_ConstantInput_Converge();
    Test_Track_Speed();
    Test_GyroDrift_Corrected();
    Test_DtVariation_NoDiverge();
    Test_ExtremeInput_NoNaN();
    Test_BiasState_Recovered();
    TEST_SUMMARY("attitude_kalman");
}
