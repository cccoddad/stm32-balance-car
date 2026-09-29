#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stdint.h>

/**
 * @file bsp_uart.h
 * @brief USART2（蓝牙 ECB01）中断接收驱动。
 *
 * 职责：持有接收缓冲、在 HAL 接收完成回调里把字节转交给上层注册的回调、
 * 自动重启下一次单字节中断接收。上层（Port/App）不接触任何 HAL 类型。
 */

/* 收到 1 字节时被调用的回调类型。 */
typedef void (*bsp_uart_rx_cb_t)(uint8_t byte);

/* 启动 USART2 单字节中断接收（复位内部状态，可重复调用）。 */
void BSP_UART2_Init(void);

/* 注册字节接收回调；传 NULL 表示丢弃收到的字节。 */
void BSP_UART2_SetRxCallback(bsp_uart_rx_cb_t cb);

#endif /* __BSP_UART_H */
