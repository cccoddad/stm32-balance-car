#ifndef __PORT_MOTOR_H
#define __PORT_MOTOR_H

#include <stdint.h>

/**
 * @file port_motor.h
 * @brief 电机驱动（TB6612 + 直流减速电机）的硬件抽象接口。
 */

/* 设置左右轮带符号 PWM：正负表示转向，绝对值表示占空比（已限幅）。 */
void port_motor_set(int16_t pwm_a, int16_t pwm_b);

#endif /* __PORT_MOTOR_H */
