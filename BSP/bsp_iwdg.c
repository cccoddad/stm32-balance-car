#include "bsp_iwdg.h"
#include "stm32f1xx.h"

/**
 * @file bsp_iwdg.c
 * @brief 独立看门狗（寄存器直写，HAL/寄存器两套配置通用，无需编译开关）。
 *
 * 寄存器依据 RM0008 独立看门狗章节：
 *   IWDG_KR  0x5555 = 解锁 PR/RLR 写保护；0xAAAA = 重装并启动看门狗；0xCCCC = 启动
 *   IWDG_PR  预分频（4=64 分频）
 *   IWDG_RLR 重装载值（0xFFF）
 */
void BSP_IWDG_Init(void)
{
    IWDG->KR = 0x5555u;  /* 解锁寄存器写保护 */
    IWDG->PR = 4u;       /* LSI/64 → 计数时钟 625Hz */
    IWDG->RLR = 0xFFFu;  /* 4095/625Hz ≈ 2.56s 超时 */
    IWDG->KR = 0xAAAAu;  /* 装载重装值 */
    IWDG->KR = 0xCCCCu;  /* 启动 */
}

void BSP_IWDG_Feed(void)
{
    IWDG->KR = 0xAAAAu;
}
