#include "bsp_motor.h"





/**
 * @brief 设置 A 电机方向。
 * @param direct 期望方向，可取 GO、BACK 或 STOP。
 *
 * TB6612 使用两个方向引脚控制一个电机。这里的正反转只代表软件定义方向，
 * 如果实车方向相反，可以调整接线或交换 GO/BACK 对应的电平。
 */
void Int_TB6612_MotorA(uint8_t direct)
{
    if(direct == GO)
    {
        /* 正转：AIN1=0，AIN2=1。 */
       AIN1_L;
       AIN2_H;
    }
    else if(direct == BACK)
    {
        /* 反转：AIN1=1，AIN2=0。 */
        AIN1_H;
        AIN2_L;
    }
    else 
    {
        /* 其他情况进入短刹车：AIN1=1，AIN2=1。 */
        AIN1_H;
        AIN2_H;
    }
}


/**
 * @brief 设置 B 电机方向。
 * @param direct 期望方向，可取 GO、BACK 或 STOP。
 *
 * B 电机与 A 电机控制方式一致，只是使用 BIN1/BIN2 方向引脚。
 */
void Int_TB6612_MotorB(uint8_t direct)
{
    if(direct == GO)
    {
        /* 正转：BIN1=0，BIN2=1。 */
       BIN1_L;
       BIN2_H;
    }
    else if(direct == BACK)
    {
        /* 反转：BIN1=1，BIN2=0。 */
        BIN1_H;
        BIN2_L;
    }
    else 
    {
        /* 其他情况进入短刹车：BIN1=1，BIN2=1。 */
        BIN1_H;
        BIN2_H;
    }
}


/**
 * @brief 根据带符号 PWM 同时设置两个电机的方向和占空比。
 * @param pwma A 电机控制量，正负号表示方向，绝对值表示 PWM 占空比。
 * @param pwmb B 电机控制量，正负号表示方向，绝对值表示 PWM 占空比。
 *
 * PID 输出的是带符号控制量，但 TB6612 需要“方向引脚 + PWM 占空比”两部分。
 * 所以本函数先根据正负号设置方向，再把负数取绝对值后写入定时器比较寄存器。
 */
void Int_TB6612_SetPWM(int pwma,int pwmb)
{
    /* 带符号 PWM 的处理步骤：先处理方向，再设置 PWM 占空比。 */


    /* 1. 处理 A 电机。 */
    /* 1.1 根据符号设置方向。 */
    if(pwma > 0)
    {
        /* 正数表示 A 电机正转。 */
        Int_TB6612_MotorA(GO);
    }
    else if(pwma < 0)
    {
        /* 负数表示 A 电机反转。 */
        Int_TB6612_MotorA(BACK);
        /* 定时器比较寄存器需要非负占空比，因此先取绝对值。 */
        pwma = -pwma;
    }
    else 
    {
        /* 控制量为 0 时进入刹车。 */
        Int_TB6612_MotorA(STOP);
    }
    /* 1.2 将 PWM 占空比写入 TIM4_CH4 的比较寄存器。 */
    // TIM4 ->CCR4 = pwma;
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,pwma);


    /* 2. 处理 B 电机。 */
    /* 2.1 根据符号设置方向。 */
    if(pwmb > 0)
    {
        /* 正数表示 B 电机正转。 */
        Int_TB6612_MotorB(GO);
    }
    else if(pwmb < 0)
    {
        /* 负数表示 B 电机反转。 */
        Int_TB6612_MotorB(BACK);
        /* 定时器比较寄存器需要非负占空比，因此先取绝对值。 */
        pwmb = -pwmb;
    }
    else 
    {
        /* 控制量为 0 时进入刹车。 */
        Int_TB6612_MotorB(STOP);
    }
    /* 2.2 将 PWM 占空比写入 TIM4_CH3 的比较寄存器。 */
    // TIM4 ->CCR3 = pwmb;
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,pwmb);
}



