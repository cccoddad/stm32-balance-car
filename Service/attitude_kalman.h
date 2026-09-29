#ifndef __ATTITUDE_KALMAN_H
#define __ATTITUDE_KALMAN_H

/**
 * @file attitude_kalman.h
 * @brief 二状态一阶卡尔曼滤波（倾角 + 陀螺零偏），纯 C 实现，零硬件依赖。
 *
 * P1 重构要点：原先滤波状态全部是模块级全局变量（angle、Q_bias、PP 等），
 * 只能存在一个实例且无法在 Host 侧隔离测试；现在全部收敛进 kalman_t，
 * 由调用方实例化并持有，同一份代码可在 PC 上并行跑多个实例做对比测试。
 */

typedef struct {
    float angle;     /* 融合后的车身倾角（度） */
    float angle_dot; /* 扣除零偏后的角速度（度/秒） */
    float Q_bias;    /* 陀螺零偏估计值 */
    float Q_angle;   /* 角度过程噪声协方差 */
    float Q_gyro;    /* 零偏过程噪声协方差 */
    float R_angle;   /* 加速度测量噪声协方差 */
    float dt;        /* 滤波周期（秒），必须等于真实采样周期 */
    /* 误差协方差矩阵 P = [[P00, P01], [P10, P11]] */
    float P00, P01, P10, P11;
} kalman_t;

/**
 * @brief 初始化滤波器：状态清零、协方差单位阵、Q/R/dt 取 Car_Config.h 默认值。
 * @param k 滤波器实例。
 */
void Kalman_Init(kalman_t *k);

/**
 * @brief 执行一次滤波更新（预测 + 量测修正）。
 * @param k 滤波器实例。
 * @param accel_angle 加速度计解算的倾角：低频稳定但受震动影响。
 * @param gyro_rate 陀螺角速度：高频响应快但存在零偏漂移。
 * @return 融合后的倾角（度），同时写入 k->angle。
 */
float Kalman_Update(kalman_t *k, float accel_angle, float gyro_rate);

#endif /* __ATTITUDE_KALMAN_H */
