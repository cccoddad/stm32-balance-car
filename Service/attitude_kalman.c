#include "attitude_kalman.h"
#include "Car_Config.h"

void Kalman_Init(kalman_t *k)
{
    k->angle = 0.0f;
    k->angle_dot = 0.0f;
    k->Q_bias = 0.0f;
    k->Q_angle = CAR_KALMAN_Q_ANGLE;
    k->Q_gyro = CAR_KALMAN_Q_GYRO;
    k->R_angle = CAR_KALMAN_R_ANGLE;
    /* dt 由 Car_Config.h 统一定义，必须等于采样任务的实际周期，
     * 否则预测步 angle += ω·dt 的增益出错，收敛变慢甚至发散。 */
    k->dt = CAR_SAMPLE_PERIOD_S;
    /* 初始误差协方差取单位阵：对初始状态不做任何先验信任。 */
    k->P00 = 1.0f;
    k->P01 = 0.0f;
    k->P10 = 0.0f;
    k->P11 = 1.0f;
}

float Kalman_Update(kalman_t *k, float accel_angle, float gyro_rate)
{
    /* 本算法量测矩阵 H = [1, 0]，故 H·P·Hᵀ = P00，观测残差系数恒为 1。 */
    float angle_err;
    float pc0, pc1, e;
    float k0, k1;
    float t0, t1;
    float pdot0, pdot1, pdot2, pdot3;

    /* ---- 1. 预测步：用扣除零偏后的角速度积分预测倾角 ---- */
    k->angle += (gyro_rate - k->Q_bias) * k->dt;

    /* 误差协方差的先验预测：P = P + (Q - P相关项)·dt。 */
    pdot0 = k->Q_angle - k->P01 - k->P10;
    pdot1 = -k->P11;
    pdot2 = -k->P11;
    pdot3 = k->Q_gyro;
    k->P00 += pdot0 * k->dt;
    k->P01 += pdot1 * k->dt;
    k->P10 += pdot2 * k->dt;
    k->P11 += pdot3 * k->dt;

    /* ---- 2. 量测修正步：用加速度计角度修正预测 ---- */
    angle_err = accel_angle - k->angle; /* 测量残差 */

    pc0 = k->P00; /* C₀=1 时 H·P·Hᵀ 的两个分量 */
    pc1 = k->P10;

    e = k->R_angle + pc0; /* 残差协方差 S = R + H·P·Hᵀ */

    k0 = pc0 / e; /* 卡尔曼增益 */
    k1 = pc1 / e;

    t0 = pc0;
    t1 = k->P01;

    /* 后验误差协方差：P = (I - K·H)·P */
    k->P00 -= k0 * t0;
    k->P01 -= k0 * t1;
    k->P10 -= k1 * t0;
    k->P11 -= k1 * t1;

    /* ---- 3. 后验状态估计 ---- */
    k->angle += k0 * angle_err;  /* 修正倾角 */
    k->Q_bias += k1 * angle_err; /* 修正陀螺零偏，抑制长期漂移 */
    k->angle_dot = gyro_rate - k->Q_bias;

    return k->angle;
}
