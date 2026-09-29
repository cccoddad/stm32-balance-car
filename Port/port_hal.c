/**
 * @file port_hal.c
 * @brief Port 层的实机实现：把抽象接口转发到 BSP 驱动（HAL 版）。
 *
 * 这是固件侧唯一的"粘合代码"。Phase 3 会提供同目录的 port_sim.c，
 * 用倒立摆物理模型实现同名接口，让 Service/App 算法在 PC 上闭环运行，
 * 两份实现按编译开关二选一。
 */
#include "port_imu.h"
#include "port_encoder.h"
#include "port_motor.h"
#include "port_battery.h"
#include "port_display.h"
#include "port_uart.h"

#include "bsp_imu.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_adc.h"
#include "bsp_uart.h"
#include "oled.h"

#include <stdio.h>

/* ======== IMU ======== */

void port_imu_init(void)
{
    BSP_IMU_Init();
}

void port_imu_read(port_imu_data_t *out)
{
    BSP_IMU_ReadAccel(&out->ax, &out->ay, &out->az);
    BSP_IMU_ReadGyro(&out->gx, &out->gy, &out->gz);
}

/* ======== 编码器 ======== */

void port_encoder_read(int32_t *ea, int32_t *eb)
{
    /* TIM2 接左轮、TIM3 接右轮，编号即通道选择参数。 */
    *ea = BSP_Encoder_ReadCount(2);
    *eb = BSP_Encoder_ReadCount(3);
}

/* ======== 电机 ======== */

void port_motor_init(void)
{
    BSP_Motor_Init();
}

void port_motor_set(int16_t pwm_a, int16_t pwm_b)
{
    BSP_Motor_SetPWM(pwm_a, pwm_b);
}

/* ======== 电池电压 ======== */

float port_battery_read(void)
{
    return BSP_ADC_ReadBattery();
}

/* ======== OLED 显示 ======== */

void port_display_show(float bat_v, int32_t ea, int32_t eb, float angle)
{
    /* 显示缓冲统一留 8 字节：覆盖最坏格式化长度（含符号、小数位）再加结束符，
     * 配合 snprintf 使用，杜绝 sprintf 无边界写入（问题清单 #6 的修复落点）。 */
    char bat_str[8];
    char ea_str[8];
    char eb_str[8];
    char angle_str[8];

    snprintf(bat_str, sizeof(bat_str), "%3.1f", (double)bat_v);
    OLED_ShowString(32, 0, bat_str, 16, 1);

    snprintf(ea_str, sizeof(ea_str), "%6d", (int)ea);
    snprintf(eb_str, sizeof(eb_str), "%6d", (int)eb);
    OLED_ShowString(24, 16, ea_str, 16, 1); /* 行首标签 "EA:" 由 main.c 静态绘制 */
    OLED_ShowString(24, 32, eb_str, 16, 1); /* 行首标签 "EB:" */

    snprintf(angle_str, sizeof(angle_str), "%5.1f", (double)angle);
    OLED_ShowString(48, 48, angle_str, 16, 1); /* 行首标签 "Angle:" */

    OLED_Refresh();
}

/* ======== 串口接收 ======== */

void port_uart_init(void)
{
    BSP_UART2_Init();
}

void port_uart_set_rx_cb(port_uart_rx_cb_t cb)
{
    /* 两侧回调签名一致（void (*)(uint8_t)），直接透传。 */
    BSP_UART2_SetRxCallback(cb);
}
