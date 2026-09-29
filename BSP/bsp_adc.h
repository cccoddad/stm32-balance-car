#ifndef __BSP_ADC_H
#define __BSP_ADC_H

/**
 * @brief 读取电池电压（伏特）。
 *
 * 电阻分压后进 ADC1，换算时乘以 4 还原实际电压。
 * 单次转换模式：启动后必须等转换完成（Poll）才能取值。
 */
float BSP_ADC_ReadBattery(void);

#endif /* __BSP_ADC_H */
