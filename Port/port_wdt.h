#ifndef __PORT_WDT_H
#define __PORT_WDT_H

/**
 * @brief 看门狗（故障自恢复的最后防线）的硬件抽象接口。
 * 实机 = 独立看门狗 IWDG；仿真 = 空实现（PC 上无需复位语义）。
 */

/* 启动看门狗（在应用初始化的最后调用）。 */
void port_wdt_init(void);

/* 喂狗（控制任务每周期调用）。 */
void port_wdt_feed(void);

#endif /* __PORT_WDT_H */
