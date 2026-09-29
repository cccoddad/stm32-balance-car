#ifndef __PORT_IMU_H
#define __PORT_IMU_H

/**
 * @file port_imu.h
 * @brief 惯性测量单元（MPU6050）的硬件抽象接口。
 *
 * 签名只使用标准 C 类型，不出现任何 HAL/BSP 类型：
 * - 实机实现 port_hal.c 转发到 BSP/bsp_imu.c；
 * - 主机仿真实现 port_sim.c（Phase 3）用倒立摆模型生成数据。
 * Service/App 只认本头文件，这就是"同一份算法双端编译"的解耦点。
 */

/* 六轴原始数据（ADC 原始计数，未换算物理单位）。 */
typedef struct {
    short ax, ay, az; /* 加速度三轴 */
    short gx, gy, gz; /* 角速度三轴 */
} port_imu_data_t;

/* 初始化传感器（量程/采样率/低通滤波）。 */
void port_imu_init(void);

/* 读取一帧六轴数据。 */
void port_imu_read(port_imu_data_t *out);

#endif /* __PORT_IMU_H */
