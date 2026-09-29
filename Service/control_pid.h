#ifndef __CONTROL_PID_H
#define __CONTROL_PID_H

#include "Car_Config.h"

/**
 * @file control_pid.h
 * @brief 三环 PID（直立 PD / 速度 PI / 转向 P），纯 C 实现，零硬件依赖。
 *
 * P1 重构要点：
 * 1. 参数统一由 car_params_t 结构体传入（const 指针），不再依赖散落的全局变量；
 * 2. 速度环的积分累计与低通滤波状态收敛进 velocity_pid_state_t，由调用方持有，
 *    去掉函数内 static 状态 → 同一份代码未来可在 Host 侧并行实例化测试；
 * 3. 每环输出均按 pwm_limit 限幅，积分按 velocity_i_limit 限幅（三级限幅：
 *    单环输出、积分、三环叠加后统一限幅）。
 */

/* 速度环 PI 的内部状态（积分累计 + 一阶低通上拍），由调用方实例化并持有。 */
typedef struct {
    int32_t i_acc;    /* 积分累计值（含遥控前后目标偏移） */
    int32_t lpf_last; /* 一阶低通滤波的上一次输出 */
} velocity_pid_state_t;

/* 通用限幅：把 value 夹到 [-limit, limit]。 */
int32_t PID_Clamp(int32_t value, int32_t limit);

/* 复位速度环状态（积分与滤波清零），初始化与异常恢复时使用。 */
void PID_VelocityStateReset(velocity_pid_state_t *st);

/**
 * @brief 直立环 PD 控制。
 * @param p 运行参数集合（balance_kp / balance_kd / balance_angle / pwm_limit）。
 * @param angle 卡尔曼滤波后的当前车身倾角。
 * @param target_angle 期望平衡角。
 * @param gy Y 轴陀螺仪原始值，沿用原始值参与微分项（阻尼）计算。
 * @return 限幅后的直立环 PWM 分量。
 *
 * 直立环只用 PD 不用 I：积分会让车身"追角度"而过冲，稳态偏差由速度环吸收。
 */
int32_t PID_Balance(const car_params_t *p, float angle, float target_angle, short gy);

/**
 * @brief 速度环 PI 控制。
 * @param st 速度环状态（积分/滤波），调用方持有，跨周期保持。
 * @param p 运行参数集合（velocity_kp / velocity_ki / velocity_i_limit / pwm_limit）。
 * @param encoder_a A 电机本周期编码器增量。
 * @param encoder_b B 电机本周期编码器增量（方向已统一）。
 * @param remote_move 遥控前后控制量，通过修改积分累计值改变速度目标。
 * @return 限幅后的速度环 PWM 分量。
 */
int32_t PID_Velocity(velocity_pid_state_t *st, const car_params_t *p,
                     int encoder_a, int encoder_b, int remote_move);

/**
 * @brief 转向环 P 控制。
 * @param p 运行参数集合（turn_kp / pwm_limit）。
 * @param gz Z 轴陀螺仪角速度原始值，目标为 0（抑制自转）。
 * @return 限幅后的转向环差速 PWM 分量。
 */
int32_t PID_Turn(const car_params_t *p, short gz);

#endif /* __CONTROL_PID_H */
