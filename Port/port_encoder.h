#ifndef __PORT_ENCODER_H
#define __PORT_ENCODER_H

#include <stdint.h>

/**
 * @file port_encoder.h
 * @brief 霍尔编码器的硬件抽象接口。
 *
 * 返回值为本控制周期的计数增量（读取后计数器清零），近似表示轮速。
 * 符号仅反映计数方向，两轮安装方向的统一由上层（App）处理。
 */

/* 读取左右轮编码器增量。 */
void port_encoder_read(int32_t *ea, int32_t *eb);

#endif /* __PORT_ENCODER_H */
