#include "pendulum_model.h"
#include <math.h>

#define PENDULUM_DT_SUB (10) /* 每个控制周期内细分的物理子步数 */

void Pendulum_Init(pendulum_t *m, double theta0_deg)
{
    m->theta_deg = theta0_deg;
    m->omega_dps = 0.0;
    m->x_m = 0.0;
    m->v_mps = 0.0;
    m->a_act = 0.0;
    m->u_pwm = 0;
    m->enc_frac = 0.0;
}

void Pendulum_SetPwm(pendulum_t *m, int32_t u_pwm)
{
    m->u_pwm = u_pwm;
}

void Pendulum_ApplyImpulse(pendulum_t *m, double dv_mps)
{
    m->v_mps += dv_mps;
}

/**
 * @brief 单个物理子步：半隐式欧拉积分。
 *
 * 倒立摆方程（theta 以弧度计，omega 以 rad/s 计）：
 *   omega_dot = (g/L)*sin(theta) - (a_act/L)*cos(theta) - B*omega
 * 第一项是重力让车倒下的趋势，第二项是车体加速度带来的"反向扶正"项
 * （跑在倒下方向之下就能扶正——平衡车的全部原理），第三项是机械阻尼。
 */
static void Pendulum_SubStep(pendulum_t *m, double dt)
{
    double theta_rad = m->theta_deg * M_PI / 180.0;
    double omega_rad = m->omega_dps * M_PI / 180.0;
    double omega_dot;

    /* 电机指令 → 目标加速度（负号 = 真实小车电机安装方向），再过一阶惯性。 */
    double a_cmd = -PENDULUM_A_FULL * ((double)m->u_pwm / PENDULUM_ARR);
    m->a_act += (a_cmd - m->a_act) * dt / PENDULUM_TAU_M;

    /* 摆体动力学。 */
    omega_dot = (9.81 / PENDULUM_L) * sin(theta_rad)
                - (m->a_act / PENDULUM_L) * cos(theta_rad)
                - PENDULUM_B_OMEGA * omega_rad;

    /* 半隐式：先更新角速度再更新角度，数值稳定性更好。 */
    omega_rad += omega_dot * dt;
    theta_rad += omega_rad * dt;

    /* 车体平动（含粘性阻力耗散）。 */
    m->v_mps += (m->a_act - PENDULUM_C_V * m->v_mps) * dt;
    m->x_m += m->v_mps * dt;

    /* 编码器累计（前进为正）。 */
    m->enc_frac += m->v_mps * dt / PENDULUM_M_PER_COUNT;

    m->theta_deg = theta_rad * 180.0 / M_PI;
    m->omega_dps = omega_rad * 180.0 / M_PI;
}

void Pendulum_Step(pendulum_t *m, double dt_sec)
{
    int i;
    double h = dt_sec / (double)PENDULUM_DT_SUB;
    if (dt_sec <= 0.0)
    {
        return;
    }
    for (i = 0; i < PENDULUM_DT_SUB; i++)
    {
        Pendulum_SubStep(m, h);
    }
}

int32_t Pendulum_TakeCounts(pendulum_t *m)
{
    /* 向零取整（C 浮点转整型语义），并把余数留到下个周期。 */
    int32_t counts = (int32_t)m->enc_frac;
    m->enc_frac -= (double)counts;
    return counts;
}
