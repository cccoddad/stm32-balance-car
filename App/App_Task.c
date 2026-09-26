#include "App_Task.h"
#include "Car_Config.h"

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
 *
 * 注意：必须先退出临界区再调用 vTaskDelete(NULL)。vTaskDelete 会把自己移出
 * 就绪列表并请求一次上下文切换，若在临界区内自删除，任务栈在临界区解除前
 * 仍会被继续执行（本函数剩余代码 + 临界区退出序列），属于依赖实现细节的
 * 不安全写法；一旦 TCB/栈被提前回收就会踩到已释放内存。
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
    taskEXIT_CRITICAL();

    /* 临界区已解除，此时自删除是安全的：删除后本任务不会再被执行。 */
    vTaskDelete(NULL);
}

/**
 * @brief 姿态数据采集任务。
 * @param pvParameters FreeRTOS 任务参数，本工程未使用。
 *
 * 任务每 CAR_SAMPLE_PERIOD_MS（当前 10ms）运行一次，读取倾角、角速度和编码器增量。
 * 采样完成后通过任务通知唤醒 PID 任务，使控制计算紧跟最新数据执行。
 *
 * 周期必须与卡尔曼滤波的 dt（Car_Config.h 统一定义）保持一致，否则滤波增益出错。
 */
void App_Task_GetData(void *pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    while(1)
    {
        App_Car_GetAngle();
        /* 数据采集完成后通知 PID 任务，形成“采样 -> 控制”的同步节奏。 */
        xTaskNotifyGive(pid_task_handle);
        vTaskDelayUntil(&pxPreviousWakeTime, CAR_SAMPLE_PERIOD_MS);
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
 * 显示刷新周期为 CAR_DISPLAY_PERIOD_MS（当前 50ms），比控制环低，避免 SPI
 * 刷屏占用过多实时控制时间。
 */
void App_Task_Display(void *pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while(1)
    {
        App_Car_Display();
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
 * 刻意不用 printf/HAL_UART_Transmit：故障现场里阻塞式打印可能二次死锁。
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
