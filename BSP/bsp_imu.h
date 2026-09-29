#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include "i2c.h"

#define MPU_IIC_ADDR 0x68       // MPU6050 默认 7 位 I2C 地址，HAL 使用时会左移 1 位。
#define MPU_SELF_TESTX_REG      0X0D    // X 轴自检寄存器。
#define MPU_SELF_TESTY_REG      0X0E    // Y 轴自检寄存器。
#define MPU_SELF_TESTZ_REG      0X0F    // Z 轴自检寄存器。
#define MPU_SELF_TESTA_REG      0X10    // 加速度计自检寄存器。
#define MPU_SAMPLE_RATE_REG     0X19    // 采样率分频寄存器。
#define MPU_CFG_REG             0X1A    // 配置寄存器，主要用于数字低通滤波器。
#define MPU_GYRO_CFG_REG        0X1B    // 陀螺仪量程配置寄存器。
#define MPU_ACCEL_CFG_REG       0X1C    // 加速度计量程配置寄存器。
#define MPU_MOTION_DET_REG      0X1F    // 运动检测阈值配置寄存器。
#define MPU_FIFO_EN_REG         0X23    // FIFO 使能寄存器。
#define MPU_I2CMST_CTRL_REG     0X24    // 第二 I2C 主机控制寄存器。
#define MPU_I2CSLV0_ADDR_REG    0X25    // 第二 I2C 从机 0 设备地址寄存器。
#define MPU_I2CSLV0_REG         0X26    // 第二 I2C 从机 0 数据地址寄存器。
#define MPU_I2CSLV0_CTRL_REG    0X27    // 第二 I2C 从机 0 控制寄存器。
#define MPU_I2CSLV1_ADDR_REG    0X28    // 第二 I2C 从机 1 设备地址寄存器。
#define MPU_I2CSLV1_REG         0X29    // 第二 I2C 从机 1 数据地址寄存器。
#define MPU_I2CSLV1_CTRL_REG    0X2A    // 第二 I2C 从机 1 控制寄存器。
#define MPU_I2CSLV2_ADDR_REG    0X2B    // 第二 I2C 从机 2 设备地址寄存器。
#define MPU_I2CSLV2_REG         0X2C    // 第二 I2C 从机 2 数据地址寄存器。
#define MPU_I2CSLV2_CTRL_REG    0X2D    // 第二 I2C 从机 2 控制寄存器。
#define MPU_I2CSLV3_ADDR_REG    0X2E    // 第二 I2C 从机 3 设备地址寄存器。
#define MPU_I2CSLV3_REG         0X2F    // 第二 I2C 从机 3 数据地址寄存器。
#define MPU_I2CSLV3_CTRL_REG    0X30    // 第二 I2C 从机 3 控制寄存器。
#define MPU_I2CSLV4_ADDR_REG    0X31    // 第二 I2C 从机 4 设备地址寄存器。
#define MPU_I2CSLV4_REG         0X32    // 第二 I2C 从机 4 数据地址寄存器。
#define MPU_I2CSLV4_DO_REG      0X33    // 第二 I2C 从机 4 写数据寄存器。
#define MPU_I2CSLV4_CTRL_REG    0X34    // 第二 I2C 从机 4 控制寄存器。
#define MPU_I2CSLV4_DI_REG      0X35    // 第二 I2C 从机 4 读数据寄存器。

#define MPU_I2CMST_STA_REG      0X36    // 第二 I2C 主机状态寄存器。
#define MPU_INTBP_CFG_REG       0X37    // 中断/旁路配置寄存器。
#define MPU_INT_EN_REG          0X38    // 中断使能寄存器。
#define MPU_INT_STA_REG         0X3A    // 中断状态寄存器。

#define MPU_ACCEL_XOUTH_REG     0X3B    // X 轴加速度高 8 位寄存器。
#define MPU_ACCEL_XOUTL_REG     0X3C    // X 轴加速度低 8 位寄存器。
#define MPU_ACCEL_YOUTH_REG     0X3D    // Y 轴加速度高 8 位寄存器。
#define MPU_ACCEL_YOUTL_REG     0X3E    // Y 轴加速度低 8 位寄存器。
#define MPU_ACCEL_ZOUTH_REG     0X3F    // Z 轴加速度高 8 位寄存器。
#define MPU_ACCEL_ZOUTL_REG     0X40    // Z 轴加速度低 8 位寄存器。

#define MPU_TEMP_OUTH_REG       0X41    // 温度值高 8 位寄存器。
#define MPU_TEMP_OUTL_REG       0X42    // 温度值低 8 位寄存器。

#define MPU_GYRO_XOUTH_REG      0X43    // X 轴陀螺仪高 8 位寄存器。
#define MPU_GYRO_XOUTL_REG      0X44    // X 轴陀螺仪低 8 位寄存器。
#define MPU_GYRO_YOUTH_REG      0X45    // Y 轴陀螺仪高 8 位寄存器。
#define MPU_GYRO_YOUTL_REG      0X46    // Y 轴陀螺仪低 8 位寄存器。
#define MPU_GYRO_ZOUTH_REG      0X47    // Z 轴陀螺仪高 8 位寄存器。
#define MPU_GYRO_ZOUTL_REG      0X48    // Z 轴陀螺仪低 8 位寄存器。

#define MPU_I2CSLV0_DO_REG      0X63    // 第二 I2C 从机 0 数据输出寄存器。
#define MPU_I2CSLV1_DO_REG      0X64    // 第二 I2C 从机 1 数据输出寄存器。
#define MPU_I2CSLV2_DO_REG      0X65    // 第二 I2C 从机 2 数据输出寄存器。
#define MPU_I2CSLV3_DO_REG      0X66    // 第二 I2C 从机 3 数据输出寄存器。

#define MPU_I2CMST_DELAY_REG    0X67    // 第二 I2C 主机延时管理寄存器。
#define MPU_SIGPATH_RST_REG     0X68    // 信号通道复位寄存器。
#define MPU_MDETECT_CTRL_REG    0X69    // 运动检测控制寄存器。
#define MPU_USER_CTRL_REG       0X6A    // 用户控制寄存器，控制 FIFO 和第二 I2C 等功能。
#define MPU_PWR_MGMT1_REG       0X6B    // 电源管理寄存器 1，控制复位、睡眠和时钟源。
#define MPU_PWR_MGMT2_REG       0X6C    // 电源管理寄存器 2，控制各轴传感器待机状态。
#define MPU_FIFO_CNTH_REG       0X72    // FIFO 计数高 8 位寄存器。
#define MPU_FIFO_CNTL_REG       0X73    // FIFO 计数低 8 位寄存器。
#define MPU_FIFO_RW_REG         0X74    // FIFO 数据读写寄存器。
#define MPU_DEVICE_ID_REG       0X75    // 设备 ID 寄存器，正常应读到 0x68。


/* 初始化 MPU6050 的量程、采样率、滤波器和工作模式。 */
void BSP_IMU_Init(void);

/* 读取陀螺仪三轴原始数据。 */
void BSP_IMU_ReadGyro(short *gx, short *gy, short *gz);

/* 读取加速度计三轴原始数据。 */
void BSP_IMU_ReadAccel(short *ax, short *ay, short *az);

#endif
