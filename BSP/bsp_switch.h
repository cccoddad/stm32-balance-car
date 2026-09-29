#ifndef BSP_SWITCH_H
#define BSP_SWITCH_H

/**
 * @file bsp_switch.h
 * @brief 驱动实现编译开关：0 = HAL 版（默认），1 = 寄存器版。
 *
 * 两套实现符号完全相同（BSP_IMU_* / BSP_Motor_*），由各自 .c 文件用
 * #if 互斥包裹，任一时刻只有一套参与链接：
 *   HAL  版：bsp_imu.c / bsp_motor.c      —— 业务代码调用 HAL API
 *   寄存器版：bsp_imu_reg.c / bsp_motor_reg.c —— 直接操作 RCC/GPIO/TIM/I2C 寄存器，
 *            其中 MPU6050 走软件模拟 I2C 时序（起始/地址/应答/停止全部手写）
 *
 * 切换方式（二选一）：
 *   1. 修改本文件的默认值；
 *   2. Keil 工程 C/C++ → Preprocessor Symbols 里加 BSP_USE_REG=1（不动仓库）。
 *
 * 切换后需全量重编译（配置不同时不可混用 .o）。
 */

#ifndef BSP_USE_REG
#define BSP_USE_REG 0
#endif

#endif /* BSP_SWITCH_H */
