#include "bsp_uart.h"
#include "usart.h"

/* 单字节接收缓冲：每次中断收满 1 字节触发回调，随后立即重启接收。 */
static uint8_t s_rx_byte;
static bsp_uart_rx_cb_t s_rx_cb;

void BSP_UART2_Init(void)
{
    /* 不清除已注册回调：接收可能被多次启动（main 板级初始化一次、
     * App_Car_Init 一次），重复启动时 HAL 返回 BUSY，既存接收继续有效。 */
    HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
}

void BSP_UART2_SetRxCallback(bsp_uart_rx_cb_t cb)
{
    s_rx_cb = cb;
}

/**
 * @brief HAL UART 接收完成回调（由 USART2 中断上下文调用）。
 *
 * 处理顺序：先交给上层回调消费该字节，再重启下一次中断接收。
 * 只处理 USART2：本工程仅其使用中断接收模式，USART1 只做 printf 输出。
 *
 * 注意：回调运行在中断上下文，上层实现必须是短小、不阻塞的（协议状态机
 * 恰好满足：纯内存操作，无延时、无打印）。
 */
/* cppcheck-suppress constParameterPointer -- 参数 const 与否由 HAL 回调原型决定，不可改 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (s_rx_cb != 0)
        {
            s_rx_cb(s_rx_byte);
        }
        HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
    }
}
