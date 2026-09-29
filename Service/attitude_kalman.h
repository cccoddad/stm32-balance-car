#ifndef __ATTITUDE_KALMAN_H
#define __ATTITUDE_KALMAN_H

/* 一阶卡尔曼滤波：输入加速度倾角和陀螺仪角速度，输出全局变量 angle。 */
void Com_Filter_Kalman(float Accel,float Gyro);

#endif
