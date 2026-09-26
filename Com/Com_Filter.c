#include "Com_Filter.h"

/* 卡尔曼滤波参数：用于融合加速度计倾角和陀螺仪角速度。 */
float K1 =0.02; 
float angle, angle_dot;     
float Q_angle=0.001;    // 角度过程噪声协方差，数值越大表示越不相信模型预测。
float Q_gyro=0.003;     // 陀螺仪零偏过程噪声协方差，影响对陀螺仪漂移的修正速度。
float R_angle=0.5;      // 加速度测量噪声协方差，数值越大表示越不相信加速度计角度。
float dt=0.005;         // 滤波计算周期，单位为秒，需要和实际调用周期尽量一致。                 
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

/**
 * @brief 一阶卡尔曼滤波，融合加速度倾角与陀螺仪角速度。
 * @param Accel 由加速度计解算得到的倾角，低频稳定但容易受震动影响。
 * @param Gyro 陀螺仪角速度，高频响应快但长期会有零偏漂移。
 *
 * 算法思路：
 * 1. 先用陀螺仪角速度预测当前角度。
 * 2. 再用加速度计角度修正预测结果。
 * 3. 同时估计陀螺仪零偏 Q_bias，减小长期漂移。
 */
void Com_Filter_Kalman(float Accel,float Gyro)      
{
    angle+=(Gyro - Q_bias) * dt; // 先验估计：用角速度积分预测当前角度。
    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // 先验估计误差协方差的微分。

    Pdot[1]=-PP[1][1];
    Pdot[2]=-PP[1][1];
    Pdot[3]=Q_gyro;
    PP[0][0] += Pdot[0] * dt;   // 对协方差微分进行积分，得到新的误差协方差。
    PP[0][1] += Pdot[1] * dt;
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;
        
    Angle_err = Accel - angle;  // 测量残差：加速度角度与预测角度的差值。
    
    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];
    
    E = R_angle + C_0 * PCt_0;
    
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    
    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       // 更新后验估计误差协方差。
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;
        
    angle   += K_0 * Angle_err;  // 后验估计：修正后的最终角度。
    Q_bias  += K_1 * Angle_err;  // 后验估计：修正陀螺仪零偏。
    angle_dot   = Gyro - Q_bias; // 输出角速度，已扣除估计到的零偏。
}

