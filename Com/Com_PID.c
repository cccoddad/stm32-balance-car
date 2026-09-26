#include "Com_PID.h"

/**
 * @brief 直立环 PD 控制。
 * @param kp 直立环比例系数，角度偏差越大，输出越大。
 * @param kd 直立环微分系数，用陀螺仪角速度抑制快速倾倒和振荡。
 * @param kalman_angle 卡尔曼滤波后的当前车身倾角。
 * @param balance_angle 期望平衡角，也就是小车静止直立时的目标角度。
 * @param gy Y 轴陀螺仪原始值，本函数沿用原始值参与微分项计算。
 * @return 直立环输出 PWM 分量，后续会与速度环、转向环输出叠加。
 */
int Com_PID_Balance(float kp, float kd, float kalman_angle, float balance_angle, short gy)
{
    /* 1. 计算角度偏差：当前角度减目标角度。 */
    float bias_angle = 0.0f;
    bias_angle = kalman_angle - balance_angle;
    /* 2. PD 输出：比例项负责扶正，微分项负责阻尼。 */
    return kp * bias_angle + kd * gy;
}

/**
 * @brief 速度环 PI 控制。
 * @param kp 速度环比例系数，影响当前轮速偏差的修正强度。
 * @param ki 速度环积分系数，影响长期位移/速度偏差的修正强度。
 * @param encoder_a A 电机本周期编码器增量。
 * @param encoder_b B 电机本周期编码器增量。
 * @param remove_move 遥控前后运动的控制量，用来人为改变速度环目标。
 * @return 速度环输出 PWM 分量，用于修正小车长期前冲或后退。
 *
 * 平衡车的速度环通常作为直立环的外环或辅助环使用。这里把两个编码器增量相加
 * 得到整体前后速度，再经过低通滤波和积分限幅，避免速度环输出过猛影响直立环。
 */
int Com_PID_Velocity(float kp, float ki, int encoder_a, int encoder_b,int remove_move)
{
    int bias_velocity = 0;
    static int least_velocity = 0; // 速度积分累计值，用于记录长期速度/位移偏差。
    static int last_velocity = 0;  // 上一次滤波后的速度偏差，用于一阶低通滤波。
    /* 1. 计算速度偏差：目标速度为 0，因此直接使用两个轮子的增量和。 */
    bias_velocity = (encoder_a + encoder_b) - 0;

    /* 2. 对速度偏差做一阶低通滤波，减小编码器抖动对外环的影响。 */
    /*    本次滤波值 = k * 上次滤波值 + (1 - k) * 本次原始值。 */
    bias_velocity = 0.8 * last_velocity + 0.2 * bias_velocity;
    /* 保存本次滤波结果，下一次计算继续使用。 */
    last_velocity = bias_velocity;

    /* 3. 累加积分项。这里没有显式乘以采样周期，采样周期固定时可通过 ki 统一调节。 */
    least_velocity += bias_velocity;
    /* 遥控前后运动通过改变积分累计值，使小车主动向前或向后移动。 */
    least_velocity -= remove_move;

    /* 4. 积分限幅。所有积分控制都要防止积分饱和，否则会导致小车恢复很慢甚至失控。 */
    /*    编码器读数是带符号 16 位范围，这里把积分限制在较保守的 +/-10000。 */
    if(least_velocity > 10000)
    {
        least_velocity = 10000;
    }
    else if(least_velocity < -10000)
    {
        least_velocity = -10000;
    }

    /* 5. 速度环 PI 输出。 */
    return kp * bias_velocity + ki * least_velocity;
}

/**
 * @brief 转向环 P 控制。
 * @param kp 转向环比例系数。
 * @param gz Z 轴陀螺仪角速度原始值，反映车身左右自转速度。
 * @return 转向环输出 PWM 分量，后续以差速形式叠加到左右电机。
 *
 * 当没有遥控转向时，转向环用于抑制车身绕 Z 轴自然旋转；主动遥控转向时，
 * 上层逻辑会跳过该环，避免它抵消人为转向指令。
 */
int Com_PID_Turn(float kp,short gz)
{
    /* 目标 Z 轴角速度为 0，因此偏差就是当前 gz。 */
    return kp*(gz-0);
}
