#include "App_Car.h"
#include <math.h>
#include <string.h>

/* 圆周率常量，把 atan2 的弧度换算为角度。 */
#define PI 3.14159265

/* ======== 运行参数 ======== */
/* 实例唯一、编译期用 Car_Config.h 默认值初始化；运行期可经协议在线修改。 */
car_params_t g_car_params = CAR_PARAMS_DEFAULT;

/* ======== 模块状态（static 收敛作用域，同时便于调试器观察） ======== */
/* 卡尔曼滤波器实例：倾角/零偏/协方差全部内聚，不再散落全局变量。 */
static kalman_t s_kalman;
/* 速度环状态（积分 + 低通），由本层持有跨周期保持。 */
static velocity_pid_state_t s_vel_state;
/* 本周期六轴原始数据。 */
static port_imu_data_t s_imu;
/* 本周期编码器增量（eb 方向已统一）。 */
static int32_t s_enc_a, s_enc_b;
/* 加速度换算倾角与陀螺换算角速度（滤波输入，保留供观察）。 */
static float s_accel_angle;
static float s_gyro_y;
/* 串口协议解析状态机。 */
static proto_parser_t s_parser;

/* 遥控状态：由串口字节回调（中断上下文）更新，控制任务读取。
 * 任一方向指令都会清掉其它方向（互斥），语义与旧版单字符 switch 一致。 */
static uint8_t flag_up, flag_down, flag_left, flag_right;
/* 遥控前后控制量（速度环目标偏移）与转向差速累加量。 */
static int remote_move, remote_turn;

/* 前向声明：协议消息处理（先于注册回调定义）。 */
static void App_Car_HandleMsg(const proto_msg_t *msg);
static void App_Car_OnRxByte(uint8_t byte);

/**
 * @brief 应用层初始化（调度器启动前调用）。
 *
 * 顺序要求：先复位解析器，再启动串口接收并注册回调，避免回调打到
 * 未初始化的状态机上。
 */
void App_Car_Init(void)
{
    Kalman_Init(&s_kalman);
    PID_VelocityStateReset(&s_vel_state);
    Proto_Init(&s_parser);

    port_imu_init();
    port_uart_init();
    port_uart_set_rx_cb(App_Car_OnRxByte);
}

/**
 * @brief 获取平衡车姿态与轮速数据。
 *
 * 一次控制周期的完整采样：读六轴 → 加速度算静态倾角 → 陀螺得角速度 →
 * 卡尔曼融合 → 同频读取编码器增量。
 */
void App_Car_GetAngle(void)
{
    /* 1. 读取 MPU6050 的三轴加速度和三轴角速度原始数据。 */
    port_imu_read(&s_imu);

    /* 2. 通过 X/Z 轴加速度计算车身倾角。atan2 返回弧度，需要换算为角度。 */
    s_accel_angle = atan2(s_imu.ax, s_imu.az) * 180 / PI;

    /* 3. 陀螺仪量程 +/-2000 度/秒，比例系数 65536 / 4000 = 16.4。 */
    /*    取反是为了让角速度方向与加速度计算的倾角方向一致。 */
    s_gyro_y = -s_imu.gy / 16.4;

    /* 4. 卡尔曼融合，减小加速度噪声与陀螺漂移。结果存入 s_kalman.angle。 */
    Kalman_Update(&s_kalman, s_accel_angle, s_gyro_y);

    /* 5. 读取编码器增量。B 轮取反是为了统一两个电机的正方向。 */
    port_encoder_read(&s_enc_a, &s_enc_b);
    s_enc_b = -s_enc_b;
}

/**
 * @brief 刷新 OLED 显示（电压 / 编码器 / 倾角）。
 *
 * 采集与格式化经 Port 层完成，本层只负责"什么时候显示、显示什么"。
 */
void App_Car_Display(void)
{
    float bat_vol = port_battery_read();
    port_display_show(bat_vol, s_enc_a, s_enc_b, s_kalman.angle);
}

/**
 * @brief 执行一次平衡车控制运算，并输出到电机。
 *
 * 控制结构由三部分叠加：
 * 1. 直立环 PD：主要扶正力矩；
 * 2. 速度环 PI：修正长期前后移动趋势（遥控前后改其目标）；
 * 3. 转向环 P / 遥控差速：控制绕 Z 轴旋转。
 * 三环叠加后统一限幅再交 Port 输出。
 */
void App_Car_PID(void)
{
    int32_t balance_out;
    int32_t velocity_out;
    int32_t turn_out = 0;
    int32_t pwma, pwmb;

    /* 1. 直立环：以卡尔曼倾角为输入。 */
    balance_out = PID_Balance(&g_car_params, s_kalman.angle,
                              g_car_params.balance_angle, s_imu.gy);

    /* 2. 遥控前后 → 速度环目标偏移；无指令时清零，避免残留量影响自动平衡。 */
    if (flag_up)
    {
        remote_move = 50;
    }
    else if (flag_down)
    {
        remote_move = -50;
    }
    else
    {
        remote_move = 0;
    }
    velocity_out = PID_Velocity(&s_vel_state, &g_car_params,
                                (int)s_enc_a, (int)s_enc_b, remote_move);

    /* 3. 转向：遥控时累加差速；不遥控时用转向环抑制 Z 轴自转。 */
    if (flag_left)
    {
        remote_turn += -20;
    }
    else if (flag_right)
    {
        remote_turn += 20;
    }
    else
    {
        remote_turn = 0;
        /* 只有不主动转向时才启用转向环，避免抵消遥控意图。 */
        turn_out = PID_Turn(&g_car_params, s_imu.gz);
    }
    /* 持续按住会不断累加差速，需要限幅保护。 */
    remote_turn = PID_Clamp(remote_turn, g_car_params.remote_turn_limit);

    /* 4. 三环叠加：turn_out 与 remote_turn 以相反符号分配给左右轮形成差速，
     *    叠加结果统一限幅后输出，防止超出 PWM 硬件范围（ARR=7199）。 */
    pwma = PID_Clamp(balance_out + velocity_out + turn_out + remote_turn,
                     g_car_params.pwm_limit);
    pwmb = PID_Clamp(balance_out + velocity_out - turn_out - remote_turn,
                     g_car_params.pwm_limit);
    port_motor_set((int16_t)pwma, (int16_t)pwmb);
}

/**
 * @brief 串口字节回调（中断上下文）：驱动协议状态机，产出指令即处理。
 */
static void App_Car_OnRxByte(uint8_t byte)
{
    proto_msg_t msg;
    if (Proto_Feed(&s_parser, byte, &msg))
    {
        App_Car_HandleMsg(&msg);
    }
}

/**
 * @brief 把一条协议指令落到运行状态/参数上。
 *
 * 并发说明：本函数运行在 USART2 中断上下文，与控制任务并发。
 * 标志位是单字节、参数是单精度浮点（32 位对齐单字存储），在 Cortex-M3
 * 上均为原子写，不存在撕裂读写；多字段联动修改的互斥保护列入 Phase 4
 * （互斥锁 + 优先级继承）。
 */
static void App_Car_HandleMsg(const proto_msg_t *msg)
{
    switch (msg->cmd)
    {
    case PROTO_MOVE:
        /* 与旧版一致：任何方向指令同时清掉其它方向（互斥）。 */
        flag_up = (uint8_t)(msg->arg1[0] == 'U');
        flag_down = (uint8_t)(msg->arg1[0] == 'D');
        flag_left = 0;
        flag_right = 0;
        break;
    case PROTO_TURN:
        flag_left = (uint8_t)(msg->arg1[0] == 'L');
        flag_right = (uint8_t)(msg->arg1[0] == 'R');
        flag_up = 0;
        flag_down = 0;
        break;
    case PROTO_PID:
    {
        /* 在线调参：@PID,<参数名>,<数值># ，数值非法则整条忽略。 */
        float value;
        if (!Proto_ParseFloat(msg->arg2, &value))
        {
            break;
        }
        if (strcmp(msg->arg1, "BKP") == 0)
        {
            g_car_params.balance_kp = value;
        }
        else if (strcmp(msg->arg1, "BKD") == 0)
        {
            g_car_params.balance_kd = value;
        }
        else if (strcmp(msg->arg1, "BANG") == 0)
        {
            g_car_params.balance_angle = value;
        }
        else if (strcmp(msg->arg1, "VKP") == 0)
        {
            g_car_params.velocity_kp = value;
        }
        else if (strcmp(msg->arg1, "VKI") == 0)
        {
            g_car_params.velocity_ki = value;
        }
        else if (strcmp(msg->arg1, "TKP") == 0)
        {
            g_car_params.turn_kp = value;
        }
        break;
    }
    default:
        break;
    }
}
