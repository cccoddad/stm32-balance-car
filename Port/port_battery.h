#ifndef __PORT_BATTERY_H
#define __PORT_BATTERY_H

/**
 * @file port_battery.h
 * @brief 电池电压测量的硬件抽象接口。
 */

/* 读取电池电压（伏特），采样失败时返回 0。 */
float port_battery_read(void);

#endif /* __PORT_BATTERY_H */
