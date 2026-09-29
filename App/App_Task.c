#include "App_Task.h"
#include "App_Car.h"
#include "Car_Config.h"
#include "port_wdt.h"
#include "stm32f1xx.h" /* DWT/CoreDebug：运行时统计时基 */
#include <stdio.h>

TaskHandle_t start_task_handle;
#define START_TASK_STACK 128

/* 姿态与编码器采样任务：周期读传感器，完成后通知 PID 任务。 */
TaskHandle_t data_task_handle;
#define DATA_TASK_STACK 128

/* PID 控制任务：阻塞等通知，最高优先级，保证闭环周期。 */
TaskHandle_t pid_task_handle;
#define PID_TASK_STACK 128

/* OLED 显示任务：低频刷新调试信息，不参与实时控制闭环。 */
TaskHandle_t display_task_handle;
#define DISPLAY_TASK_STACK 128

void App_Task_Start(void *pvParameters);
void App_Task_GetData(void *pvParameters);
void App_Task_PID(void *pvParameters);
void App_Task_Display(void *pvParameters);

/* ======== 控制周期统计 ======== */
/* 单写（PID 任务）多读（任意任务）：三个字段都是 32 位，Cortex-M 单字读写
 * 原子，快照偶有撕裂不影响调试观察。 */
static control_period_stats_t s_period_stats;

void App_Task_GetPeriodStats(control_period_stats_t *out)
{
    *out = s_period_stats;
}

/* ======== 运行时统计时基：DWT 周期计数器 ======== */
/* FreeRTOSConfig.h 的 portGET_RUN_TIME_COUNTER_VALUE 指向这里。
 * 72MHz 下 32 位计数约 59.6s 回绕——统计表按快照查看，已在配置头注明局限。 */
void App_RunTimeStatsInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* 打开 DWT 等调试单元 */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            /* 启用周期计数 */
}

unsigned long App_GetCycles(void)
{
    return DWT->CYCCNT;
}

/* 状态打印缓冲：静态分配（显示任务栈只有 128 word，放不下）。
 * 运行时统计表一行约 60 字节 × 4 任务 + 表头，384 字节足够。 */
static char s_status_buf[384];

/**
 * @brief 打印完整状态块（阻塞式，只能在任务上下文调用）。
 *
 * 数据来源全部是"谁拥有谁打印"：周期统计来自本文件、角度/电压来自
 * App_Car 访问器、栈水位来自 FreeRTOS API 直接读各任务 TCB。
 */
void App_Task_PrintStatus(void)
{
    control_period_stats_t st;
    App_Task_GetPeriodStats(&st);

    printf("[STAT] attitude=%+6.1f deg  battery=%4.1f V\r\n",
           (double)App_Car_GetAttitude(), (double)App_Car_GetBattery());
    printf("[STAT] period: n=%u min=%u max=%u tick (expect 10)\r\n",
           (unsigned)st.samples, (unsigned)st.min_ticks, (unsigned)st.max_ticks);
    printf("[STAT] stack free (word): data=%u pid=%u disp=%u\r\n",
           (unsigned)uxTaskGetStackHighWaterMark(data_task_handle),
           (unsigned)uxTaskGetStackHighWaterMark(pid_task_handle),
           (unsigned)uxTaskGetStackHighWaterMark(display_task_handle));
    printf("[STAT] params: BKP=%.1f BKD=%.3f BANG=%.1f VKP=%.1f VKI=%.3f TKP=%.2f\r\n",
           (double)g_car_params.balance_kp, (double)g_car_params.balance_kd,
           (double)g_car_params.balance_angle, (double)g_car_params.velocity_kp,
           (double)g_car_params.velocity_ki, (double)g_car_params.turn_kp);
    /* 运行时统计：按累计 CPU 周期折算百分比（快照视图）。 */
    s_status_buf[0] = '\0';
    vTaskGetRunTimeStats(s_status_buf);
    printf("[STAT] cpu%% (name: cycles%%)\r\n%s", s_status_buf);
}

void App_Task_Init(void)
{
    /* 1. 应用层初始化（滤波/协议/IMU/串口回调/看门狗启动）与
     *    运行时统计时基，都必须在调度器启动前完成。 */
    App_Car_Init();
    App_RunTimeStatsInit();

    /* 2. 创建启动任务。启动任务只负责继续创建其他业务任务。 */
    xTaskCreate(
        (TaskFunction_t)App_Task_Start,
        (char *)"App_Task_Start",
        (configSTACK_DEPTH_TYPE)START_TASK_STACK,
        (void *)NULL,
        (UBaseType_t)CAR_TASK_PRIO_START,
        (TaskHandle_t *)&start_task_handle);

    /* 3. 启动 FreeRTOS 调度器，之后程序由各个任务接管运行。 */
    vTaskStartScheduler();
}

/**
 * @brief 启动任务：在临界区内创建业务任务。
 *
 * 进入临界区避免任务创建过程中被调度打断。三个业务任务创建完成后，
 * 启动任务删除自己，节省 RAM 和调度开销。
 *
 * 注意：必须先退出临界区再调用 vTaskDelete(NULL)。vTaskDelete 会把自己移出
 * 就绪列表并请求一次上下文切换，若在临界区内自删除，任务栈在临界区解除前
 * 仍会被继续执行（本函数剩余代码 + 临界区退出序列），属于依赖实现细节的
 * 不安全写法——一旦 TCB/栈被提前回收就会踩到已释放内存。
 */
void App_Task_Start(void *pvParameters)
{
    taskENTER_CRITICAL();
    xTaskCreate(
        (TaskFunction_t)App_Task_GetData,
        (char *)"App_Task_GetData",
        (configSTACK_DEPTH_TYPE)DATA_TASK_STACK,
        (void *)NULL,
        (UBaseType_t)CAR_TASK_PRIO_DATA,
        (TaskHandle_t *)&data_task_handle);
    xTaskCreate(
        (TaskFunction_t)App_Task_PID,
        (char *)"App_Task_PID",
        (configSTACK_DEPTH_TYPE)PID_TASK_STACK,
        (void *)NULL,
        (UBaseType_t)CAR_TASK_PRIO_PID,
        (TaskHandle_t *)&pid_task_handle);
    xTaskCreate(
        (TaskFunction_t)App_Task_Display,
        (char *)"App_Task_Display",
        (configSTACK_DEPTH_TYPE)DISPLAY_TASK_STACK,
        (void *)NULL,
        (UBaseType_t)CAR_TASK_PRIO_DISP,
        (TaskHandle_t *)&display_task_handle);
    taskEXIT_CRITICAL();

    /* 临界区已解除，此时自删除是安全的：删除后本任务不会再被执行。 */
    vTaskDelete(NULL);
}

/**
 * @brief 姿态数据采集任务（10ms 周期）。
 *
 * 每 CAR_SAMPLE_PERIOD_MS 读一次倾角、角速度和编码器增量，完成后用任务
 * 通知唤醒 PID 任务，形成"采样 → 控制"的同步节奏。
 * 周期与卡尔曼 dt（Car_Config.h 统一定义）保持一致，否则滤波增益出错。
 */
void App_Task_GetData(void *pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    while (1)
    {
        App_Car_GetAngle();
        /* 数据采集完成后通知 PID 任务。PID 优先级更高，会立即抢占运行。 */
        xTaskNotifyGive(pid_task_handle);
        vTaskDelayUntil(&pxPreviousWakeTime, CAR_SAMPLE_PERIOD_MS);
    }
}

/**
 * @brief PID 控制任务（最高优先级）。
 *
 * 不主动延时，阻塞等待采样任务的通知，保证每次控制都用最新一组数据。
 * 同时统计相邻两次执行的实际间隔（min/max），作为控制周期确定性的证据：
 * 数据任务用 vTaskDelayUntil 绝对延时调度，其通知节奏即控制周期基准，
 * 本任务的唤醒间隔偏离 10ms 的程度反映调度抖动。
 */
void App_Task_PID(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_log = last_wake;

    while (1)
    {
        /* 等待采样任务通知；pdTRUE 表示取走通知后自动清零。 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        TickType_t now = xTaskGetTickCount();
        uint32_t delta = (uint32_t)(now - last_wake); /* 无符号减法自带回绕处理 */
        last_wake = now;

        if (s_period_stats.samples == 0u)
        {
            s_period_stats.min_ticks = delta;
            s_period_stats.max_ticks = delta;
        }
        else
        {
            if (delta < s_period_stats.min_ticks)
            {
                s_period_stats.min_ticks = delta;
            }
            if (delta > s_period_stats.max_ticks)
            {
                s_period_stats.max_ticks = delta;
            }
        }
        s_period_stats.samples++;

        /* 周期性打印统计（可由 Car_Config.h 关闭）。
         * 10s 一次、几十字节的阻塞发送 < 控制周期，通知计数器会把
         * 期间到来的通知挂起，不造成丢周期，只在当拍产生一次晚唤醒。 */
#if CAR_PERIOD_STATS_LOG
        if ((uint32_t)(now - last_log) >= 10000u)
        {
            last_log = now;
            printf("[CTRL] period: n=%u min=%u max=%u ticks\r\n",
                   (unsigned)s_period_stats.samples,
                   (unsigned)s_period_stats.min_ticks,
                   (unsigned)s_period_stats.max_ticks);
        }
#endif
        App_Car_PID();

        /* 喂狗：控制任务活着 = 整个闭环活着。看门狗超时 2.56s，
         * 本任务若卡死（死锁/硬件挂死）将触发硬件复位。 */
        port_wdt_feed();
    }
}

/**
 * @brief OLED 显示任务（50ms 周期）。
 *
 * 优先级低于采样/控制链路，刷屏（SPI 全屏刷新耗时较长）不再影响闭环。
 */
void App_Task_Display(void *pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while (1)
    {
        App_Car_Display();

        /* 收到 @ST# 请求则打印状态块（中断里只置位，阻塞式 printf 在这里执行，
         * 打印期间被更高优先级的控制任务抢占是安全的）。 */
        if (App_Car_ConsumeStatusReq())
        {
            App_Task_PrintStatus();
        }

        vTaskDelayUntil(&pxPreviousWakeTime, CAR_DISPLAY_PERIOD_MS);
    }
}

/* 栈溢出时被踩掉的任务名，供调试器/故障现场读取。 */
volatile char g_stack_overflow_task[configMAX_TASK_NAME_LEN];

/**
 * @brief 栈溢出钩子，由 FreeRTOS 在 configCHECK_FOR_STACK_OVERFLOW 检测到溢出时调用。
 * @param xTask 发生溢出的任务句柄。
 * @param pcTaskName 发生溢出的任务名。
 *
 * 该钩子在任务切换路径中被调用，此时系统状态已不可信，因此这里只做三件事：
 * 1. 把任务名抄进全局变量（调试器可直接读，不依赖串口是否可用）；
 * 2. 关中断，避免带着被踩坏的栈继续被调度；
 * 3. 死循环停机，等待看门狗复位或调试器接管。
 * 刻意不用阻塞式打印（printf 经重定向走串口发送）：故障现场里可能二次死锁。
 */
/* cppcheck-suppress constParameterPointer -- 签名必须与 FreeRTOS.h 中的钩子原型一致 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    volatile char *dst = g_stack_overflow_task;
    (void)xTask;

    while (*pcTaskName != '\0' && (dst - g_stack_overflow_task) < (configMAX_TASK_NAME_LEN - 1))
    {
        *dst++ = *pcTaskName++;
    }
    *dst = '\0';

    portDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}
