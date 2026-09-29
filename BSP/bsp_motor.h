#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "tim.h"



#define GO 0       // 软件定义的正转方向。
#define BACK 1     // 软件定义的反转方向。
#define STOP 2     // 刹车/停止状态。

#define AIN1_L HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET) // A 电机方向引脚 AIN1 输出低电平。
#define AIN1_H HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET)   // A 电机方向引脚 AIN1 输出高电平。

#define AIN2_L HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET) // A 电机方向引脚 AIN2 输出低电平。
#define AIN2_H HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET)   // A 电机方向引脚 AIN2 输出高电平。

#define BIN1_L HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET) // B 电机方向引脚 BIN1 输出低电平。
#define BIN1_H HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET)   // B 电机方向引脚 BIN1 输出高电平。

#define BIN2_L HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET) // B 电机方向引脚 BIN2 输出低电平。
#define BIN2_H HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET)   // B 电机方向引脚 BIN2 输出高电平。


/* 根据 PID 输出的带符号 PWM，设置左右电机方向与占空比。 */
void Int_TB6612_SetPWM(int pwma, int pwmb);

#endif
