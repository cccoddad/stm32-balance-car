#ifndef __BSP_IWDG_H
#define __BSP_IWDG_H

/**
 * @brief 独立看门狗初始化：LSI 40kHz，预分频 /64，重装 0xFFF。
 * 超时 ≈ 0xFFF / (40000/64) ≈ 2.56s。启动后必须周期喂狗，
 * 否则硬件复位（无复位引脚调试时表现为"神秘重启"，是最后防线）。
 */
void BSP_IWDG_Init(void);

/* 喂狗（写 IWDG_KR = 0xAAAA 重装计数器）。 */
void BSP_IWDG_Feed(void);

#endif /* __BSP_IWDG_H */
