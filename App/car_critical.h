#ifndef CAR_CRITICAL_H
#define CAR_CRITICAL_H

#include <stdint.h>

/**
 * @file car_critical.h
 * @brief 临界区可移植封装：固件走 FreeRTOS，主机仿真（SIL/单测）是空操作。
 *
 * 为什么需要：App 层的"参数在线修改"要在中断与控制任务之间保证
 * 多字段成组可见（idx+value 不能被读到一半）。固件上的正确原语是
 * 临界区（mutex 不能在中断里用）；而主机侧 SIL 是单线程程序，
 * 不存在并发，空操作即可——但同一份源码必须两边都能编译。
 *
 * 用法：
 *   CAR_ENTER(); ... CAR_EXIT();                  // 任务上下文
 *   uint32_t s = CAR_ENTER_ISR(); ... CAR_EXIT_ISR(s);  // 中断上下文
 *
 * 主机编译由构建系统定义 CAR_HOST_BUILD（见 Host/CMakeLists.txt）。
 */

#ifdef CAR_HOST_BUILD

#define CAR_ENTER() ((void)0)
#define CAR_EXIT() ((void)0)
#define CAR_ENTER_ISR() (0u)
#define CAR_EXIT_ISR(saved) ((void)(saved))

#else /* 固件：FreeRTOS 临界区 */

#include "FreeRTOS.h"
#include "task.h"

#define CAR_ENTER() taskENTER_CRITICAL()
#define CAR_EXIT() taskEXIT_CRITICAL()
#define CAR_ENTER_ISR() taskENTER_CRITICAL_FROM_ISR()
#define CAR_EXIT_ISR(saved) taskEXIT_CRITICAL_FROM_ISR(saved)

#endif /* CAR_HOST_BUILD */

#endif /* CAR_CRITICAL_H */
