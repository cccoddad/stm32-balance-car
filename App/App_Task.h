#ifndef __APP_TASK_H
#define __APP_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "App_Car.h"

/**
 * @file App_Task.h
 * @brief FreeRTOS 任务划分与控制周期统计。
 *
 * 任务结构（优先级见 Car_Config.h）：
 *   PID 任务(5)  ← 任务通知 ← 采样任务(4)，10ms 周期；显示任务(2) 独立 50ms。
 *   控制链路（采样→控制）优先级高于显示，OLED 刷屏不再打断闭环周期。
 */

/* 控制周期实测统计（单位 tick，configTICK_RATE_HZ=1000 → 1 tick = 1ms）。 */
typedef struct {
    uint32_t samples;   /* 已统计的周期数 */
    uint32_t min_ticks; /* 相邻两次控制执行的最小间隔 */
    uint32_t max_ticks; /* 相邻两次控制执行的最大间隔 */
} control_period_stats_t;

/* 初始化并启动 FreeRTOS 应用任务（内部先执行 App_Car_Init）。 */
void App_Task_Init(void);

/* 读取控制周期统计快照（任意任务可调，用于调试观察）。 */
void App_Task_GetPeriodStats(control_period_stats_t *out);

#endif /* __APP_TASK_H */
