#ifndef __PORT_SIM_H
#define __PORT_SIM_H

#include <stdint.h>

/**
 * @file port_sim.h
 * @brief Port 层的主机仿真实现控制接口（仅 Host 构建编译，固件不用）。
 *
 * 与 port_hal.c 实现同一组 Port 接口，内部把传感器/执行器接到
 * Host/sim/pendulum_model.c 倒立摆模型上。主循环（main_sil.c）通过本头
 * 文件提供的控制接口做实验配置：复位、施加扰动、冻结台架、读取真值。
 */

/* 传感器安装偏角（度）：几何直立（theta=0）时 IMU 读数恰为固件默认
 * 目标平衡角 -1.0°，模拟真实小车"机械安装后车不一定 0 度直立"。 */
#define PORT_SIM_MOUNT_OFFSET_DEG (-1.0)

/* 复位仿真世界：初始倾角、随机种子。 */
void PortSim_Reset(double theta0_deg, unsigned int seed);

/* 物理推进 dt 秒（固件每个控制周期结束后由主循环调用）。 */
void PortSim_Step(double dt_sec);

/* E2 扰动：给车体一个瞬时速度冲击（m/s）。 */
void PortSim_ApplyImpulse(double dv_mps);

/* E3 台架模式：enable=1 冻结摆体动力学（θ 恒为 theta_deg），
 * 传感器噪声与陀螺零偏继续演化；enable=0 解冻。 */
void PortSim_Freeze(int enable, double theta_deg);

/* 读取真值（度 / m/s / m）。 */
void PortSim_GetTruth(double *theta_deg, double *v_mps, double *x_m);

/* 设置仿真电池电压（port_battery_read 返回值），用于欠压保护实验。 */
void PortSim_SetBattery(float volts);

/* 最近一次 port_motor_set 的左右轮 PWM。 */
int32_t PortSim_GetLastPwmA(void);
int32_t PortSim_GetLastPwmB(void);

/* 模型当前的实际车体加速度（m/s^2，已过一阶惯性），诊断用。 */
double PortSim_GetAccel(void);

/* 最近一次 port_imu_read 生成的融合前原始观测（与固件输入完全同源），
 * 供 E3 的互补滤波/纯积分对比实验使用。 */
void PortSim_GetLastSensors(float *accel_angle, float *gyro_y);

#endif /* __PORT_SIM_H */
