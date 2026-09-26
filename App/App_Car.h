#ifndef __APP_CAR_H
#define __APP_CAR_H


#include "Int_MPU6050.h"
#include "Com_Filter.h"
#include "math.h"
#include "adc.h"
#include "oled.h"
#include "Int_Encoder.h"
#include "Com_PID.h"
#include "Int_TB6612.h"
#include "usart.h"
#include "stm32f1xx_hal_uart.h"

/* 圆周率常量，用于将 atan2 计算得到的弧度转换为角度。 */
#define PI 3.14159265

/* 采集 MPU6050 姿态数据并读取编码器增量，供 PID 控制周期使用。 */
void App_Car_GetAngle(void);

/* 将电池电压、编码器计数和当前倾角显示到 OLED。 */
void App_Car_Display(void);

/* 执行一次平衡车三环控制，并把最终 PWM 输出给电机驱动。 */
void App_Car_PID(void);

#endif
