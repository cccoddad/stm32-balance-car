#include "App_Task.h"

TaskHandle_t start_task_handle;
#define START_TASK_STACK 128
#define START_TASK_PRIORITY 1
void App_Task_Start(void *pvParameters);

/* 姿态与编码器采样任务配置：周期读取传感器数据，作为 PID 输入。 */
TaskHandle_t data_task_handle;
#define DATA_TASK_STACK 128
#define DATA_TASK_PRIORITY 3
void App_Task_GetData(void *pvParameters);

/* PID 控制任务配置：收到采样任务通知后立即运行一次控制计算。 */
TaskHandle_t pid_task_handle;
#define PID_TASK_STACK 128
#define PID_TASK_PRIORITY 3
void App_Task_PID(void *pvParameters);

/* OLED 显示任务配置：低频刷新调试信息，不参与实时控制闭环。 */
TaskHandle_t display_task_handle;
#define DISPLAY_TASK_STACK 128
#define DISPLAY_TASK_PRIORITY 3
void App_Task_Display(void *pvParameters);

void App_Task_Init(void)
{
    /* 1. 创建启动任务。启动任务只负责继续创建其他业务任务。 */
    xTaskCreate(
        (TaskFunction_t)App_Task_Start,
        (char *)"App_Task_Start",
        (configSTACK_DEPTH_TYPE)START_TASK_STACK,
        (void *)NULL,
        (UBaseType_t)START_TASK_PRIORITY,
        (TaskHandle_t *)&start_task_handle);

    /* 2. 启动 FreeRTOS 调度器，之后程序由各个任务接管运行。 */
    vTaskStartScheduler();
}

/**
 * @brief 启动任务：在临界区内创建业务任务。
 * @param pvParameters FreeRTOS 任务参数，本工程未使用。
 *
 * 进入临界区可以避免任务创建过程中被调度打断。三个业务任务创建完成后，
 * 启动任务删除自己，节省 RAM 和调度开销。
 */
void App_Task_Start(void *pvParameters)
{
    taskENTER_CRITICAL();
    xTaskCreate(
        (TaskFunction_t)App_Task_GetData,
        (char *)"App_Task_GetData",
        (configSTACK_DEPTH_TYPE)DATA_TASK_STACK,
        (void *)NULL,
        (UBaseType_t)DATA_TASK_PRIORITY,
        (TaskHandle_t *)&data_task_handle);
    xTaskCreate(
        (TaskFunction_t)App_Task_PID,
        (char *)"App_Task_PID",
        (configSTACK_DEPTH_TYPE)PID_TASK_STACK,
        (void *)NULL,
        (UBaseType_t)PID_TASK_PRIORITY,
        (TaskHandle_t *)&pid_task_handle);
    xTaskCreate(
        (TaskFunction_t)App_Task_Display,
        (char *)"App_Task_Display",
        (configSTACK_DEPTH_TYPE)DISPLAY_TASK_STACK,
        (void *)NULL,
        (UBaseType_t)DISPLAY_TASK_PRIORITY,
        (TaskHandle_t *)&display_task_handle);
    vTaskDelete(NULL);
    taskEXIT_CRITICAL();
}

/**
 * @brief 姿态数据采集任务。
 * @param pvParameters FreeRTOS 任务参数，本工程未使用。
 *
 * 任务每 10 个系统 tick 运行一次，读取倾角、角速度和编码器增量。
 * 采样完成后通过任务通知唤醒 PID 任务，使控制计算紧跟最新数据执行。
 */
void App_Task_GetData(void *pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    while(1)
    {
        App_Car_GetAngle();
        /* 数据采集完成后通知 PID 任务，形成“采样 -> 控制”的同步节奏。 */
        xTaskNotifyGive(pid_task_handle);
        vTaskDelayUntil(&pxPreviousWakeTime,10);
    }

}

/**
 * @brief PID 控制任务。
 * @param pvParameters FreeRTOS 任务参数，本工程未使用。
 *
 * 该任务不主动延时，而是阻塞等待采样任务的通知。这样可以保证每次控制计算
 * 都使用刚刚采集到的一组传感器数据。
 */
void App_Task_PID(void *pvParameters)
{
    while(1)
    {
        /* 等待采样任务通知；pdTRUE 表示取走通知后自动清零。 */
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        App_Car_PID();
    }
}

/**
 * @brief OLED 显示任务。
 * @param pvParameters FreeRTOS 任务参数，本工程未使用。
 *
 * 显示刷新周期为 50 个系统 tick，比控制环低，避免 SPI 刷屏占用过多实时控制时间。
 */
void App_Task_Display(void *pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while(1)
    {
        App_Car_Display();
        vTaskDelayUntil(&pxPreviousWakeTime,50);
    }

}
