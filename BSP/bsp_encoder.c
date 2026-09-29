#include "bsp_encoder.h"



/**
 * @brief 读取定时器编码器模式下的计数增量。
 * @param timx 要读取的定时器编号，本工程使用 TIM2 和 TIM3 分别接两个编码器。
 * @return 本周期读取到的带符号计数值，正负号表示转动方向。
 *
 * 编码器模式下，定时器硬件会根据 AB 相自动增减 CNT。读取后立即清零，
 * 下一次再读到的就是两个采样周期之间的增量，可近似表示轮速。
 */
int BSP_Encoder_ReadCount(uint8_t timx)
{
    int encoder_value = 0;
    switch (timx)
    {
    case 2:
    {
        /* 读取 TIM2 的计数值，并转换成 short，使 16 位计数器溢出结果按有符号数解释。 */
        // encoder_value = (short)TIM2->CNT;
        // TIM2->CNT = 0;
        encoder_value = (short)__HAL_TIM_GET_COUNTER(&htim2);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        break;
    }
    case 3:
    {
        /* 读取 TIM3 的计数值，并转换成 short，使 16 位计数器溢出结果按有符号数解释。 */
        // encoder_value = (short)TIM3->CNT;
        // TIM3->CNT = 0;
        encoder_value = (short)__HAL_TIM_GET_COUNTER(&htim3);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        break;
    }
    default:
        break;
    }

    /* 两个电机安装方向可能相反，方向统一放在上层处理，便于根据实车安装调整。 */
    return encoder_value;
}
