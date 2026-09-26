#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/**
 * @file Car_Config.h
 * @brief 平衡车全局配置：控制周期、输出限幅等"一处定义、多处引用"的常量。
 *
 * 引入目的：原工程中采样周期写死在 vTaskDelayUntil 的实参里、卡尔曼 dt 写死在
 * Com_Filter.c 的全局变量里，两者必须人为保持一致，一旦改动不同步就会让滤波
 * 增益出错（预测步 angle += ω*dt 与真实采样间隔不符）。集中定义后由编译期绑定。
 */

/* 采样/控制周期。与 FreeRTOSConfig.h 的 configTICK_RATE_HZ=1000 配合，
 * 延时函数可直接用毫秒数作为 tick 数。 */
#define CAR_SAMPLE_PERIOD_MS (10u)

/* 换算成秒，供卡尔曼滤波等连续时间模型使用。 */
#define CAR_SAMPLE_PERIOD_S  ((float)CAR_SAMPLE_PERIOD_MS / 1000.0f)

/* OLED 显示刷新周期，明显低于控制周期，避免刷屏占用实时控制时间。 */
#define CAR_DISPLAY_PERIOD_MS (50u)

/* 电机 PWM 输出限幅。TIM4 配置为 PSC=0、ARR=7199，PWM 分辨率 0~7199；
 * 三环 PID 叠加结果必须限制在该范围内，否则写入比较寄存器的值会超范围。 */
#define CAR_PWM_LIMIT (7199)

/* 遥控转向差速限幅（左右轮差值的绝对值上限）。 */
#define CAR_REMOTE_TURN_LIMIT (500)

/* 速度环积分限幅，防止积分饱和导致恢复变慢甚至失控。 */
#define CAR_VELOCITY_I_LIMIT (10000)

/* 倾角保护阈值（度）：超过该角度判定为已摔倒，应切断电机输出。 */
#define CAR_TILT_PROTECT_DEG (45.0f)

#endif /* CAR_CONFIG_H */
