#include "bsp_adc.h"
#include "adc.h"

float BSP_ADC_ReadBattery(void)
{
    float bat_vol = 0.0f;

    /* ADC 为单次转换模式：启动后必须等待转换完成（PollForConversion）才能取值，
     * 否则读到的是上一次的残留结果甚至 0（问题清单 #4 的修复落点）。 */
    if (HAL_ADC_Start(&hadc1) == HAL_OK)
    {
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
            /* 12 位 ADC：3.3V 参考对应 4095；分压电阻 1:3，实际电压需乘 4。 */
            bat_vol = ((float)HAL_ADC_GetValue(&hadc1) * 3.3f / 4095.0f) * 4.0f;
        }
    }
    return bat_vol;
}
