#include "control_pid.h"

/**
 * @brief 把浮点控制量转换为限幅后的整型输出。
 * @param out 浮点控制量（可能超出硬件范围）。
 * @param limit 绝对值上限，对应 PWM 可用范围。
 * @return 夹到 [-limit, limit] 之后的整型值。
 *
 * 先在浮点域比较再转整型，避免超大浮点值转换成 int32 时出现未定义行为。
 */
static int32_t Pid_FloatToClampedInt(float out, int32_t limit)
{
    if (out > (float)limit)
    {
        return limit;
    }
    if (out < -(float)limit)
    {
        return -limit;
    }
    return (int32_t)out;
}

int32_t PID_Clamp(int32_t value, int32_t limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void PID_VelocityStateReset(velocity_pid_state_t *st)
{
    st->i_acc = 0;
    st->lpf_last = 0;
}

int32_t PID_Balance(const car_params_t *p, float angle, float target_angle, short gy)
{
    /* 1. 角度偏差：当前角度减目标角度。 */
    float bias_angle = angle - target_angle;
    /* 2. PD 输出：比例项扶正，微分项（角速度原始值）阻尼。 */
    float out = p->balance_kp * bias_angle + p->balance_kd * (float)gy;
    return Pid_FloatToClampedInt(out, p->pwm_limit);
}

int32_t PID_Velocity(velocity_pid_state_t *st, const car_params_t *p,
                     int encoder_a, int encoder_b, int remote_move)
{
    int32_t bias_velocity = 0;

    /* 1. 速度偏差：目标速度为 0，直接使用两轮增量和。 */
    bias_velocity = (int32_t)(encoder_a + encoder_b);

    /* 2. 一阶低通滤波，减小编码器抖动对外环的影响：
     *    本次滤波值 = 0.8 * 上次滤波值 + 0.2 * 本次原始值。
     *    结果截断为整型，与原实现的 int 语义保持一致。 */
    bias_velocity = (int32_t)(0.8 * (double)st->lpf_last + 0.2 * (double)bias_velocity);
    st->lpf_last = bias_velocity;

    /* 3. 积分累计。遥控前后目标直接改积分，等效于改变速度目标。 */
    st->i_acc += bias_velocity;
    st->i_acc -= (int32_t)remote_move;

    /* 4. 积分限幅：防止积分饱和导致恢复变慢甚至失控。 */
    if (st->i_acc > p->velocity_i_limit)
    {
        st->i_acc = p->velocity_i_limit;
    }
    else if (st->i_acc < -p->velocity_i_limit)
    {
        st->i_acc = -p->velocity_i_limit;
    }

    /* 5. 速度环 PI 输出并限幅。周期固定时采样周期可并入 ki 统一调节。 */
    float out = p->velocity_kp * (float)bias_velocity +
                p->velocity_ki * (float)st->i_acc;
    return Pid_FloatToClampedInt(out, p->pwm_limit);
}

int32_t PID_Turn(const car_params_t *p, short gz)
{
    /* 目标 Z 轴角速度为 0，偏差即当前 gz。 */
    float out = p->turn_kp * (float)gz;
    return Pid_FloatToClampedInt(out, p->pwm_limit);
}
