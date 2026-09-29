#ifndef __PORT_UART_H
#define __PORT_UART_H

#include <stdint.h>

/**
 * @file port_uart.h
 * @brief 调试/遥控串口（蓝牙）的硬件抽象接口。
 *
 * 字节流以回调形式向上抛出，回调运行在中断上下文，实现必须短小不阻塞
 * （协议状态机即满足：纯内存操作）。
 */

/* 收到 1 字节时被调用的回调类型。 */
typedef void (*port_uart_rx_cb_t)(uint8_t byte);

/* 启动串口接收。 */
void port_uart_init(void);

/* 注册字节接收回调（NULL 表示丢弃）。 */
void port_uart_set_rx_cb(port_uart_rx_cb_t cb);

#endif /* __PORT_UART_H */
