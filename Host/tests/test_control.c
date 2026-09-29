/**
 * @file test_control.c
 * @brief Service/control_pid.c 单元测试：三环方向性、限幅、积分/滤波状态。
 *
 * 覆盖升级方案 Phase 2 规定的用例类别：
 * - 直立环：符号方向正确、kd 阻尼方向正确、输出限幅
 * - 速度环：低通首拍值、积分限幅生效、遥控注入生效、复位
 * - 转向环：零输入为零、比例关系、限幅
 * - 通用限幅边界
 *
 * 所有断言均为整型精确比较（控制律是确定性整数/定点运算），不用容差。
 */
#include "test_util.h"
#include "control_pid.h"

static car_params_t NewParams(void)
{
    car_params_t p = CAR_PARAMS_DEFAULT;
    return p;
}

/* ---------- 直立环 PD ---------- */

static void Test_Balance_Direction(void)
{
    car_params_t p = NewParams();
    p.balance_kp = -100.0f;
    p.balance_kd = 1.0f;
    p.pwm_limit = 1000;

    /* kp 为负：车身向正方向倾斜（angle > target）→ 输出为负（往回扶）。 */
    CHECK(PID_Balance(&p, 5.0f, 0.0f, 0) == -500);
    /* 反向倾斜 → 输出为正。 */
    CHECK(PID_Balance(&p, -5.0f, 0.0f, 0) == 500);
    /* 目标角偏置：angle == target 时比例项为 0。 */
    CHECK(PID_Balance(&p, -1.0f, -1.0f, 0) == 0);
}

static void Test_Balance_Kd_Damping(void)
{
    car_params_t p = NewParams();
    p.balance_kp = -100.0f;
    p.balance_kd = 1.0f;
    p.pwm_limit = 1000;

    /* 角速度项独立生效：angle 恰好在目标位，输出完全来自 kd*gy。 */
    CHECK(PID_Balance(&p, 0.0f, 0.0f, 10) == 10);
    CHECK(PID_Balance(&p, 0.0f, 0.0f, -10) == -10);
    /* 比例 + 微分叠加。 */
    CHECK(PID_Balance(&p, 5.0f, 0.0f, 10) == -500 + 10);
}

static void Test_Balance_OutputClamp(void)
{
    car_params_t p = NewParams();
    p.balance_kp = -720.0f;
    p.balance_kd = 0.72f;
    p.pwm_limit = 1000;

    /* 大角度：-720*100 = -72000 → 夹到 -1000。 */
    CHECK(PID_Balance(&p, 100.0f, 0.0f, 0) == -1000);
    /* 反向大角度 → 夹到 +1000。 */
    CHECK(PID_Balance(&p, -100.0f, 0.0f, 0) == 1000);
    /* 限幅为 0 → 输出恒 0。 */
    p.pwm_limit = 0;
    CHECK(PID_Balance(&p, 100.0f, 0.0f, 0) == 0);
}

/* ---------- 速度环 PI ---------- */

static void Test_Velocity_ZeroInput(void)
{
    car_params_t p = NewParams();
    velocity_pid_state_t st;
    PID_VelocityStateReset(&st);

    CHECK(PID_Velocity(&st, &p, 0, 0, 0) == 0);
}

static void Test_Velocity_LpfFirstStep(void)
{
    car_params_t p = NewParams();
    velocity_pid_state_t st;
    int out;
    PID_VelocityStateReset(&st);

    p.velocity_kp = 10.0f;
    p.velocity_ki = 1.0f;
    p.velocity_i_limit = 10000;
    p.pwm_limit = 100000;

    /* 首拍低通：0.8*0 + 0.2*100 = 20；积分同为 20。
     * out = 10*20 + 1*20 = 220（精确整数运算）。 */
    out = PID_Velocity(&st, &p, 100, 0, 0);
    CHECK(out == 220);

    /* 第二拍：lpf = int(0.8*20 + 0.2*100) = 36，积分 = 20+36 = 56。
     * out = 10*36 + 56 = 416。 */
    out = PID_Velocity(&st, &p, 100, 0, 0);
    CHECK(out == 416);
}

static void Test_Velocity_IntegralClamp(void)
{
    car_params_t p = NewParams();
    velocity_pid_state_t st;
    int out = 0;
    int i;
    PID_VelocityStateReset(&st);

    /* kp=0 使输出只剩积分项，便于精确断言积分限幅。 */
    p.velocity_kp = 0.0f;
    p.velocity_ki = 1.0f;
    p.velocity_i_limit = 100;
    p.pwm_limit = 100000;

    /* 正向持续激励 500 拍：积分必然封顶在 +100。 */
    for (i = 0; i < 500; i++)
    {
        out = PID_Velocity(&st, &p, 100, 0, 0);
    }
    CHECK(out == 100);

    /* 换向持续激励 1000 拍：积分必然封顶在 -100。 */
    for (i = 0; i < 1000; i++)
    {
        out = PID_Velocity(&st, &p, -100, 0, 0);
    }
    CHECK(out == -100);
}

static void Test_Velocity_RemoteInject(void)
{
    car_params_t p = NewParams();
    velocity_pid_state_t st;
    int out = 0;
    int i;
    PID_VelocityStateReset(&st);

    p.velocity_kp = 0.0f;
    p.velocity_ki = 1.0f;
    p.velocity_i_limit = 10000;
    p.pwm_limit = 100000;

    /* 轮子静止但注入前进目标 remote=50：积分被目标持续消耗 → 输出变负
     * （速度环会驱动车往前跑）。10 拍后积分 = -500。 */
    for (i = 0; i < 10; i++)
    {
        out = PID_Velocity(&st, &p, 0, 0, 50);
    }
    CHECK(out == -500);
}

static void Test_Velocity_OutputClamp(void)
{
    car_params_t p = NewParams();
    velocity_pid_state_t st;
    PID_VelocityStateReset(&st);

    p.velocity_kp = 1000.0f;
    p.velocity_ki = 0.0f;
    p.velocity_i_limit = 100;
    p.pwm_limit = 500;

    /* 首拍 lpf=20 → 1000*20=20000 → 夹到 500。 */
    CHECK(PID_Velocity(&st, &p, 100, 0, 0) == 500);
    /* 反向 → 夹到 -500。 */
    PID_VelocityStateReset(&st);
    CHECK(PID_Velocity(&st, &p, -100, 0, 0) == -500);
}

static void Test_Velocity_Reset(void)
{
    car_params_t p = NewParams();
    velocity_pid_state_t st;
    int i;
    PID_VelocityStateReset(&st);

    for (i = 0; i < 50; i++)
    {
        (void)PID_Velocity(&st, &p, 100, 100, 0);
    }
    PID_VelocityStateReset(&st);
    /* 状态清零后零输入输出 0。 */
    CHECK(PID_Velocity(&st, &p, 0, 0, 0) == 0);
}

/* ---------- 转向环 P ---------- */

static void Test_Turn_Basic(void)
{
    car_params_t p = NewParams();
    p.turn_kp = 2.0f;
    p.pwm_limit = 1000;

    CHECK(PID_Turn(&p, 0) == 0);
    CHECK(PID_Turn(&p, 10) == 20);
    CHECK(PID_Turn(&p, -10) == -20);
    /* 限幅：2*30000=60000 → 1000。 */
    CHECK(PID_Turn(&p, 30000) == 1000);
    CHECK(PID_Turn(&p, -30000) == -1000);
}

/* ---------- 通用限幅 ---------- */

static void Test_Clamp_Boundary(void)
{
    CHECK(PID_Clamp(5, 10) == 5);
    CHECK(PID_Clamp(10, 10) == 10);   /* 上边界不越界 */
    CHECK(PID_Clamp(15, 10) == 10);
    CHECK(PID_Clamp(-15, 10) == -10);
    CHECK(PID_Clamp(-10, 10) == -10); /* 下边界不越界 */
    CHECK(PID_Clamp(7, 0) == 0);      /* limit=0 恒 0 */
    CHECK(PID_Clamp(-7, 0) == 0);
}

int main(void)
{
    Test_Balance_Direction();
    Test_Balance_Kd_Damping();
    Test_Balance_OutputClamp();
    Test_Velocity_ZeroInput();
    Test_Velocity_LpfFirstStep();
    Test_Velocity_IntegralClamp();
    Test_Velocity_RemoteInject();
    Test_Velocity_OutputClamp();
    Test_Velocity_Reset();
    Test_Turn_Basic();
    Test_Clamp_Boundary();
    TEST_SUMMARY("control_pid");
}
