#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#include <stdint.h>

/**
 * @file Car_Config.h
 * @brief 平衡车全局配置：控制周期、限幅、任务优先级、默认参数与运行参数结构体。
 *
 * 引入目的：原工程中采样周期写死在 vTaskDelayUntil 的实参里、卡尔曼 dt 写死在
 * 滤波模块的全局变量里，两者必须人为保持一致，一旦改动不同步就会让滤波
 * 增益出错（预测步 angle += ω*dt 与真实采样间隔不符）。集中定义后由编译期绑定。
 *
 * 重构时本文件同时承载"参数中心化"：所有 PID 参数收敛到 car_params_t
 * 结构体（实例定义在 App_Car.c），默认值用本文件的宏初始化，运行期可经
 * 串口协议在线修改走"待写槽位 + 临界区"（实现见 App_Car.c）。
 */

/* ======== 控制周期 ======== */

/* 采样/控制周期。与 FreeRTOSConfig.h 的 configTICK_RATE_HZ=1000 配合，
 * 延时函数可直接用毫秒数作为 tick 数。 */
#define CAR_SAMPLE_PERIOD_MS (10u)

/* 换算成秒，供卡尔曼滤波等连续时间模型使用。 */
#define CAR_SAMPLE_PERIOD_S  ((float)CAR_SAMPLE_PERIOD_MS / 1000.0f)

/* OLED 显示刷新周期，明显低于控制周期，避免刷屏占用实时控制时间。 */
#define CAR_DISPLAY_PERIOD_MS (50u)

/* ======== 限幅与保护 ======== */

/* 电机 PWM 输出限幅。TIM4 配置为 PSC=0、ARR=7199，PWM 分辨率 0~7199；
 * 三环 PID 叠加结果必须限制在该范围内，否则写入比较寄存器的值会超范围。 */
#define CAR_PWM_LIMIT (7199)

/* 遥控转向差速限幅（左右轮差值的绝对值上限）。 */
#define CAR_REMOTE_TURN_LIMIT (500)

/* 速度环积分限幅，防止积分饱和导致恢复变慢甚至失控。 */
#define CAR_VELOCITY_I_LIMIT (10000)

/* 倾角保护阈值（度）：与目标平衡角的偏差超过该值判定为已摔倒，切断电机输出。
 * 保护是"锁存式"的：触发后必须满足下面的恢复条件持续 CAR_TILT_RECOVER_HOLD_MS
 * 才重新使能——原因见 App_Car_PID 注释（翻滚时加速度角 atan2 回绕会让
 * 融合角在阈值附近抖动，普通电平式保护会反复启停）。 */
#define CAR_TILT_PROTECT_DEG (45.0f)
/* 恢复条件：接近直立（度）且车身基本静止（陀螺绝对值 deg/s 上限）。 */
#define CAR_TILT_RECOVER_DEG (10.0f)
#define CAR_TILT_RECOVER_GYRO_DPS (30.0f)
/* 恢复条件需持续的静止时间（按采样周期折算成计数）。 */
#define CAR_TILT_RECOVER_HOLD_COUNT ((unsigned)(500u / CAR_SAMPLE_PERIOD_MS))

/* ======== 欠压保护（3S 锂电池） ======== */
/* 进入/退出降功率的电压阈值（3.2V×3=9.6V 为截止，带迟滞防止在阈值附近反复切换）。 */
#define CAR_UNDERVOLT_ENTER_V (9.6f)
#define CAR_UNDERVOLT_EXIT_V  (10.2f)
/* 降功率时的 PWM 限幅除数：半功率运行（保住控制不趴窝优先于性能）。 */
#define CAR_UNDERVOLT_PWM_SCALE_DIV (2)

/* ======== 卡尔曼滤波默认参数 ======== */
/* 角度过程噪声协方差：越大越不相信模型预测。 */
#define CAR_KALMAN_Q_ANGLE (0.001f)
/* 陀螺零偏过程噪声协方差：影响零偏估计的收敛速度。 */
#define CAR_KALMAN_Q_GYRO  (0.003f)
/* 加速度测量噪声协方差：越大越不相信加速度计角度。 */
#define CAR_KALMAN_R_ANGLE (0.5f)

/* ======== PID 默认参数（调参唯一入口） ======== */
/* 直立环：负号由电机安装方向与角度方向决定；kd 用角速度原始值阻尼。 */
#define CAR_BALANCE_KP     (-720.0f)
#define CAR_BALANCE_KD     (0.72f)
#define CAR_BALANCE_ANGLE  (-1.0f) /* 机械安装后的目标平衡角 */
/* 速度环：目标速度为 0，偏差即两轮增量和。 */
#define CAR_VELOCITY_KP    (170.0f)
#define CAR_VELOCITY_KI    (0.85f)
/* 转向环：输入为 Z 轴角速度原始值，抑制自转。 */
#define CAR_TURN_KP        (0.5f)

/* ======== FreeRTOS 任务优先级（数值越大越高） ======== */
/* 控制任务最高：保证 10ms 控制周期不被显示刷屏等低实时性工作打断，
 * 这是"控制环与 OLED 解耦"（问题清单 #13）的修复手段之一。 */
#define CAR_TASK_PRIO_PID   (5u)
/* 采样任务次之：采集完成后通知控制任务，控制任务随即抢占运行。 */
#define CAR_TASK_PRIO_DATA  (4u)
/* 显示任务低于控制链路，周期抖动只会影响显示不影响闭环。 */
#define CAR_TASK_PRIO_DISP  (2u)
#define CAR_TASK_PRIO_START (1u)

/* ======== 调试输出开关 ======== */
/* 置 1 时控制任务周期性打印实测周期统计（min/max）到 USART1（printf 重定向）。 */
#define CAR_PERIOD_STATS_LOG (1)

/**
 * @brief 运行参数集合：三环 PID 系数 + 限幅/保护阈值。
 *
 * 定义在配置头文件（纯宏头、零依赖），Service 层与 App 层都可安全包含。
 * 实例 g_car_params 定义在 App_Car.c，串口协议在线修改的也是这个实例。
 */
typedef struct {
    float balance_kp;        /* 直立环比例系数 */
    float balance_kd;        /* 直立环微分系数 */
    float balance_angle;     /* 目标平衡角（度） */
    float velocity_kp;       /* 速度环比例系数 */
    float velocity_ki;       /* 速度环积分系数 */
    float turn_kp;           /* 转向环比例系数 */
    int32_t pwm_limit;       /* 三环输出统一限幅（绝对值上限） */
    int32_t velocity_i_limit;/* 速度环积分限幅 */
    int32_t remote_turn_limit;/* 遥控转向差速限幅 */
    float tilt_protect_deg;  /* 倾角保护阈值（度），已在 App_Car_PID 锁存保护中使用 */
} car_params_t;

/* 默认参数初始化器：与旧版全局变量初值保持一致，保证语义等价。 */
#define CAR_PARAMS_DEFAULT                                                 \
    {                                                                      \
        .balance_kp = CAR_BALANCE_KP, .balance_kd = CAR_BALANCE_KD,        \
        .balance_angle = CAR_BALANCE_ANGLE, .velocity_kp = CAR_VELOCITY_KP,\
        .velocity_ki = CAR_VELOCITY_KI, .turn_kp = CAR_TURN_KP,            \
        .pwm_limit = CAR_PWM_LIMIT,                                        \
        .velocity_i_limit = CAR_VELOCITY_I_LIMIT,                          \
        .remote_turn_limit = CAR_REMOTE_TURN_LIMIT,                        \
        .tilt_protect_deg = CAR_TILT_PROTECT_DEG,                          \
    }

#endif /* CAR_CONFIG_H */
