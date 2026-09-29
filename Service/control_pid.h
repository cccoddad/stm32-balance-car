#ifndef __CONTROL_PID_H
#define __CONTROL_PID_H

/* 直立环 PD：根据倾角误差和 Y 轴角速度输出扶正 PWM 分量。 */
int Com_PID_Balance(float kp, float kd, float kalman_angle, float balance_angle, short gy);

/* 速度环 PI：根据两个编码器增量和遥控前后量输出速度修正 PWM 分量。 */
int Com_PID_Velocity(float kp, float ki, int encoder_a, int encoder_b,int remove_move);

/* 转向环 P：根据 Z 轴角速度输出差速修正 PWM 分量。 */
int Com_PID_Turn(float kp, short gz);

#endif
