/**
 * @file port_sim.c
 * @brief Port 层的主机仿真实现：六组 Port 接口全部落到倒立摆模型上。
 *
 * 与 port_hal.c 二选一编译（固件工程只挂 port_hal.c）——这就是"同一份
 * 算法代码，换个 Port 就从实机切到仿真"的可测试性设计落点。
 *
 * 传感器模拟包含四类真实效应：
 * 1. 安装偏角（几何直立时读数为 -1.0°，与实车机械安装对应）；
 * 2. 加速度计角度高斯噪声（振动的等效体现）；
 * 3. 陀螺零偏 + 随机游走（MPU6050 零偏稳定性）；
 * 4. 16 位 ADC 量化（角度→加速度计数、角速度→陀螺计数）。
 *
 * 随机数用确定性 LCG + Box-Muller，保证实验可复现。
 */
#include "port_imu.h"
#include "port_encoder.h"
#include "port_motor.h"
#include "port_battery.h"
#include "port_display.h"
#include "port_uart.h"
#include "port_sim.h"
#include "port_wdt.h"
#include "sim/pendulum_model.h"

#include <math.h>

/* ======== 传感器噪声参数（量级按 MPU6050 公开指标估计） ======== */
#define SIM_ANGLE_NOISE_STD (0.4)  /* 加速度计角度噪声标准差（度） */
#define SIM_GYRO_NOISE_STD  (0.25) /* 陀螺白噪声标准差（度/秒） */
#define SIM_GYRO_BIAS0      (1.5)  /* 初始零偏（度/秒） */
#define SIM_GYRO_BIAS_RW    (0.01) /* 零偏随机游走步长（度/秒/次读取） */
#define SIM_ACC_SCALE       (1000.0) /* 角度→加速度"计数"的比例（atan2 只看比值） */
#define SIM_GYRO_SCALE      (16.4)   /* 与固件换算系数互为倒数 */

/* ======== 仿真世界状态 ======== */
static pendulum_t s_model;
static int s_frozen = 0;
static double s_frozen_theta = 0.0;
static double s_gyro_bias = SIM_GYRO_BIAS0;
static float s_battery_volts = 11.1f;
static int16_t s_last_pwm_a, s_last_pwm_b;
static float s_last_accel_angle, s_last_gyro_y;

/* 确定性随机数：LCG + Box-Muller 生成高斯。 */
static uint32_t s_rng = 0x2468ACE1u;
static double RandU(void) /* (0,1) */
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return ((double)((s_rng >> 8) & 0x00FFFFFFu) + 1.0) / 16777217.0;
}
static double RandGauss(double std)
{
    /* Box-Muller：两个均匀分布生成一个高斯（这里取第一个）。 */
    double u1 = RandU();
    double u2 = RandU();
    return std * sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

void PortSim_Reset(double theta0_deg, unsigned int seed)
{
    Pendulum_Init(&s_model, theta0_deg);
    s_rng = seed;
    s_frozen = 0;
    s_frozen_theta = 0.0;
    s_gyro_bias = SIM_GYRO_BIAS0;
    s_battery_volts = 11.1f;
    s_last_pwm_a = 0;
    s_last_pwm_b = 0;
    s_last_accel_angle = (float)(theta0_deg + PORT_SIM_MOUNT_OFFSET_DEG);
    s_last_gyro_y = 0.0f;
}

void PortSim_Step(double dt_sec)
{
    if (s_frozen)
    {
        /* 台架模式：摆体静止，只做零偏随机游走（传感器侧在读取时也会继续）。 */
        s_gyro_bias += RandGauss(SIM_GYRO_BIAS_RW);
        return;
    }
    Pendulum_Step(&s_model, dt_sec);
}

void PortSim_ApplyImpulse(double dv_mps)
{
    Pendulum_ApplyImpulse(&s_model, dv_mps);
}

void PortSim_Freeze(int enable, double theta_deg)
{
    s_frozen = enable;
    s_frozen_theta = theta_deg;
    if (enable)
    {
        s_model.theta_deg = theta_deg;
        s_model.omega_dps = 0.0;
        s_model.v_mps = 0.0;
        s_model.a_act = 0.0;
        s_model.u_pwm = 0;
    }
}

void PortSim_GetTruth(double *theta_deg, double *v_mps, double *x_m)
{
    *theta_deg = s_model.theta_deg;
    *v_mps = s_model.v_mps;
    *x_m = s_model.x_m;
}

void PortSim_SetBattery(float volts) { s_battery_volts = volts; }

int32_t PortSim_GetLastPwmA(void) { return s_last_pwm_a; }
int32_t PortSim_GetLastPwmB(void) { return s_last_pwm_b; }

double PortSim_GetAccel(void) { return s_model.a_act; }

void PortSim_GetLastSensors(float *accel_angle, float *gyro_y)
{
    *accel_angle = s_last_accel_angle;
    *gyro_y = s_last_gyro_y;
}

/* ======== Port 接口实现 ======== */

void port_imu_init(void)
{
    /* 模型无独立外设初始化，复位由 PortSim_Reset 负责。 */
}

void port_imu_read(port_imu_data_t *out)
{
    double theta_meas = s_model.theta_deg + PORT_SIM_MOUNT_OFFSET_DEG;
    double omega_meas = s_model.omega_dps;
    double accel_angle;
    double gy;

    if (s_frozen)
    {
        theta_meas = s_frozen_theta + PORT_SIM_MOUNT_OFFSET_DEG;
        omega_meas = 0.0;
    }

    /* 1. 加速度计通道：倾角 + 高斯噪声 → 三轴等效计数。 */
    theta_meas += RandGauss(SIM_ANGLE_NOISE_STD);
    out->ax = (short)(SIM_ACC_SCALE * sin(theta_meas * M_PI / 180.0));
    out->az = (short)(SIM_ACC_SCALE * cos(theta_meas * M_PI / 180.0));
    out->ay = 0;
    /* 与固件 App_Car_GetAngle 完全相同的换算，作为 E3 对比基准。 */
    accel_angle = atan2((double)out->ax, (double)out->az) * 180.0 / M_PI;

    /* 2. 陀螺通道：真值 + 零偏（随机游走）+ 白噪声 → 16 位计数。 */
    s_gyro_bias += RandGauss(SIM_GYRO_BIAS_RW);
    omega_meas += s_gyro_bias + RandGauss(SIM_GYRO_NOISE_STD);
    gy = -SIM_GYRO_SCALE * omega_meas;
    if (gy > 32767.0)
    {
        gy = 32767.0;
    }
    if (gy < -32768.0)
    {
        gy = -32768.0;
    }
    out->gy = (short)gy;
    /* Z 轴只给等效噪声（1D 模型无偏航，但转向环照样吃原始值）。 */
    out->gx = (short)(SIM_GYRO_SCALE * RandGauss(SIM_GYRO_NOISE_STD));
    out->gz = (short)(SIM_GYRO_SCALE * RandGauss(SIM_GYRO_NOISE_STD));

    s_last_accel_angle = (float)accel_angle;
    s_last_gyro_y = (float)(-(double)out->gy / SIM_GYRO_SCALE);
}

void port_encoder_read(int32_t *ea, int32_t *eb)
{
    /* A 轮前进为正；B 轮取负返回——固件 App_Car_GetAngle 再取负统一方向，
     * 两轮相加得到车速（与实车双轮求和的语义一致）。 */
    int32_t counts = Pendulum_TakeCounts(&s_model);
    *ea = counts;
    *eb = -counts;
}

void port_motor_init(void)
{
    /* 仿真世界无外设初始化。 */
}

void port_motor_set(int16_t pwm_a, int16_t pwm_b)
{
    s_last_pwm_a = pwm_a;
    s_last_pwm_b = pwm_b;
    /* 1D 模型取左右平均作为纵向驱动。 */
    Pendulum_SetPwm(&s_model, (int32_t)(pwm_a + pwm_b) / 2);
}

float port_battery_read(void)
{
    /* 标称 3S 11.1V；欠压实验用 PortSim_SetBattery 覆盖。 */
    return s_battery_volts;
}

void port_display_show(float bat_v, int32_t ea, int32_t eb, float angle)
{
    /* 仿真无屏幕：参数未使用。 */
    (void)bat_v;
    (void)ea;
    (void)eb;
    (void)angle;
}

void port_uart_init(void)
{
    /* 仿真不启中断接收；实验如需遥控可直接调用注册的回调。 */
}

void port_wdt_init(void)
{
    /* PC 上没有复位语义。 */
}

void port_wdt_feed(void)
{
}

void port_uart_set_rx_cb(port_uart_rx_cb_t cb)
{
    (void)cb;
}
