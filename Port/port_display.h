#ifndef __PORT_DISPLAY_H
#define __PORT_DISPLAY_H

#include <stdint.h>

/**
 * @file port_display.h
 * @brief 调试显示（0.96 寸 OLED）的硬件抽象接口。
 *
 * 一次提交刷新整屏状态（电压 / 左右编码器 / 倾角），格式化与刷屏时机
 * 都封装在实现侧，调用方只描述"显示什么"。
 */

/* 刷新显示状态：电池电压（V）、左右轮编码器增量、车身倾角（度）。 */
void port_display_show(float bat_v, int32_t ea, int32_t eb, float angle);

#endif /* __PORT_DISPLAY_H */
