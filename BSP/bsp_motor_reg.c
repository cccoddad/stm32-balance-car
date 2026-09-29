#include "bsp_switch.h"

#if BSP_USE_REG

/**
 * @file bsp_motor_reg.c
 * @brief TB6612 电机驱动的寄存器版：TIM4 PWM + GPIO 方向全部直接配寄存器。
 *
 * 与 bsp_motor.c（HAL 版）符号完全一致，靠 bsp_switch.h 互斥编译。
 * 配置逐项对照 HAL 版 MX_TIM4_Init / MspPostInit（保证两版行为等价）：
 *   TIM4: PSC=0, ARR=7199, 向上计数, PWM1 模式, CH3/CH4 输出高极性,
 *         输出比较预装使能, 自动重装预装关闭（与 HAL 版一致）,
 *         引脚 PB8(CH3)/PB9(CH4) 复用推挽输出（F1 的 CNF=10, MODE=11）
 *   方向脚: AIN1=PB14 AIN2=PB15 BIN1=PB13 BIN2=PB12，开漏/推挽输出高电平
 *
 * 寄存器依据：STM32F10xxx 参考手册 RM0008（RCC_APB1ENR / TIM4 / GPIOB_CRH）。
 */
#include "bsp_motor.h"
#include "main.h"

/* 方向脚位（与 main.h 中 CubeMX 生成的定义一致，这里用位掩码直接操作 BSRR）。 */
#define M_AIN1_PIN GPIO_PIN_14
#define M_AIN2_PIN GPIO_PIN_15
#define M_BIN1_PIN GPIO_PIN_13
#define M_BIN2_PIN GPIO_PIN_12

/* BSRR 写位：低 16 位置位、高 16 位清零（原子，无读改写竞态）。 */
static void Pin_Set(uint16_t pin)
{
    GPIOB->BSRR = pin;
}
static void Pin_Clr(uint16_t pin)
{
    GPIOB->BSRR = (uint32_t)pin << 16;
}

/**
 * @brief 寄存器级完整初始化 TIM4 PWM 与方向引脚。
 *
 * 在 App 层初始化（port_motor_init）时调用；HAL 版实现为空（main 里
 * CubeMX 的 MX_TIM4_Init + PWM_Start 已完成等价配置），两版对外一致。
 */
void BSP_Motor_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* ---- 1. 方向脚：PB12~15 推挽输出（CNF=00, MODE=11 → CRH 半字节 0x03） ---- */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    {
        uint32_t crh = GPIOB->CRH;
        crh &= 0x0000FFFFu; /* 清掉 pin12~15 的 CNF+MODE */
        crh |= 0x33330000u; /* 四个半字节全部推挽输出 50MHz */
        GPIOB->CRH = crh;
    }
    Pin_Clr(M_AIN1_PIN);
    Pin_Clr(M_AIN2_PIN);
    Pin_Clr(M_BIN1_PIN);
    Pin_Clr(M_BIN2_PIN);
    (void)gpio;

    /* ---- 2. TIM4 时钟与基础配置 ---- */
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    (void)RCC->APB1ENR; /* 等待时钟使能生效（RM0008：读操作起同步作用） */

    TIM4->CR1 = 0;                    /* 先停计数，清 ARPE/方向等配置 */
    TIM4->PSC = 0;                    /* 与 HAL 版 Prescaler=1-1 一致 */
    TIM4->ARR = 7199;                 /* 与 HAL 版 Period=7200-1 一致 */
    TIM4->CNT = 0;
    TIM4->CCR3 = 0;                   /* 初始占空比 0 */
    TIM4->CCR4 = 0;

    /* ---- 3. CH3/CH4：PWM1 模式 + 预装使能（对照 HAL 的 OCMODE_PWM1） ---- */
    TIM4->CCMR2 &= ~((TIM_CCMR2_OC3M << 0) | TIM_CCMR2_OC3PE |
                     (TIM_CCMR2_OC4M << 0) | TIM_CCMR2_OC4PE);
    TIM4->CCMR2 |= (6u << 4) | TIM_CCMR2_OC3PE;  /* OC3M=110(PWM1), OC3PE=1 */
    TIM4->CCMR2 |= (6u << 12) | TIM_CCMR2_OC4PE; /* OC4M=110(PWM1), OC4PE=1 */

    /* ---- 4. 使能输出，极性高（CC3P/CC4P 保持 0） ---- */
    TIM4->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;

    /* ---- 5. 装载预装寄存器并启动（等效 HAL_TIM_PWM_Start） ---- */
    TIM4->EGR = TIM_EGR_UG;
    TIM4->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief 根据带符号 PWM 设置两轮方向与占空比（寄存器版，逻辑与 HAL 版逐行对应）。
 *
 * 方向脚状态表（对照 bsp_motor.h 的 GO/BACK/STOP 注释）：
 *   GO(正转): AIN1=0 AIN2=1 | BACK(反转): AIN1=1 AIN2=0 | STOP(刹车): 全 1
 */
void BSP_Motor_SetPWM(int pwma, int pwmb)
{
    /* ---- A 电机（方向脚 + TIM4_CH4 占空比） ---- */
    if (pwma > 0)
    {
        Pin_Clr(M_AIN1_PIN);
        Pin_Set(M_AIN2_PIN);
    }
    else if (pwma < 0)
    {
        Pin_Set(M_AIN1_PIN);
        Pin_Clr(M_AIN2_PIN);
        pwma = -pwma;
    }
    else
    {
        Pin_Set(M_AIN1_PIN);
        Pin_Set(M_AIN2_PIN); /* 短刹车 */
    }
    if (pwma > 7199)
    {
        pwma = 7199; /* 调用方已限幅，这里再兜底防 CCR 写脏 */
    }
    TIM4->CCR4 = (uint32_t)pwma;

    /* ---- B 电机（方向脚 + TIM4_CH3 占空比） ---- */
    if (pwmb > 0)
    {
        Pin_Clr(M_BIN1_PIN);
        Pin_Set(M_BIN2_PIN);
    }
    else if (pwmb < 0)
    {
        Pin_Set(M_BIN1_PIN);
        Pin_Clr(M_BIN2_PIN);
        pwmb = -pwmb;
    }
    else
    {
        Pin_Set(M_BIN1_PIN);
        Pin_Set(M_BIN2_PIN);
    }
    if (pwmb > 7199)
    {
        pwmb = 7199;
    }
    TIM4->CCR3 = (uint32_t)pwmb;
}

#endif /* BSP_USE_REG */
