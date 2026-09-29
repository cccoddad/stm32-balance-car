#ifndef __APP_CAR_H
#define __APP_CAR_H

/**
 * @file App_Car.h
 * @brief 平衡车应用编排层接口。
 *
 * P1 重构后的依赖关系：App 只依赖 Service（纯算法）+ Port（硬件抽象）+
 * Car_Config（纯宏配置），不再直接包含任何厂商驱动头文件——
 * 验收标准：对 App/ 与 Service/ 做字符串扫描，无任何 HAL 前缀接口调用。
 */

#include "Car_Config.h"
#include "control_pid.h"
#include "attitude_kalman.h"
#include "protocol.h"
#include "port_imu.h"
#include "port_encoder.h"
#include "port_motor.h"
#include "port_battery.h"
#include "port_display.h"
#include "port_uart.h"

/* 全局运行参数实例（定义在 App_Car.c，初始化为 CAR_PARAMS_DEFAULT）。
 * 串口协议 @PID,BKP,x# 在线修改的就是它。 */
extern car_params_t g_car_params;

/* 应用层初始化：滤波器/协议状态机复位、注册串口回调、经 Port 初始化 IMU。
 * 必须在调度器启动前调用（由 App_Task_Init 负责）。 */
void App_Car_Init(void);

/* 采集 MPU6050 姿态数据并读取编码器增量，供 PID 控制周期使用。 */
void App_Car_GetAngle(void);

/* 将电池电压、编码器计数和当前倾角显示到 OLED。 */
void App_Car_Display(void);

/* 执行一次平衡车三环控制，并把最终 PWM 输出给电机。 */
void App_Car_PID(void);

/* 读取卡尔曼滤波后的当前倾角（度）：OLED/串口日志/SIL 仿真的统一取值入口。 */
float App_Car_GetAttitude(void);

#endif /* __APP_CAR_H */
