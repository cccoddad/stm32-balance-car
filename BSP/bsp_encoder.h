#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "tim.h"


/* 读取指定定时器的编码器计数增量，并在读取后清零计数器。 */
int BSP_Encoder_ReadCount(uint8_t timx);

#endif
