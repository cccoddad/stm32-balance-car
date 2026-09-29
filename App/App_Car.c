#include "App_Car.h"
#include "Car_Config.h"
#include "car_critical.h"
#include <math.h>
#include <string.h>

/* 圆周率常量，把 atan2 的弧度换算为角度。 */
#define PI 3.14159265

/* ======== 运行参数 ======== */
/* 实例唯一、编译期用 Car_Config.h 默认值初始化；运行期经协议在线修改
 * （修改路径：UART 中断 → 待写槽位 → 控制任务在临界区内消费，见下文）。 */
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
/* 电池电压：控制任务（唯一所有者）每周期采样，显示任务只读。 */
static volatile float s_battery_v;
/* 欠压模式（带迟滞，见 Car_Config.h 阈值）。 */
static volatile uint8_t s_undervolt;
/* 倾角保护锁存状态与恢复驻留计数（见 App_Car_PID 中的说明）。 */
static uint8_t s_tilt_latched;
static uint16_t s_tilt_recover_cnt;
/* 状态查询请求：中断里只置位，打印延迟到显示任务（printf 是阻塞操作，
 * 绝不能在中断上下文执行——这是"中断只做标记、任务负责干活"的通用模式）。 */
static volatile uint8_t s_status_req;

/* 遥控状态：由串口字节回调（中断上下文）更新，控制任务读取。
 * 单字节读写在 Cortex-M3 上天然原子，不需要额外保护；
 * 任一方向指令清掉其它方向（互斥），语义与旧版单字符 switch 一致。 */
static uint8_t flag_up, flag_down, flag_left, flag_right;
static int remote_move, remote_turn;

/* ======== 参数在线修改：中断写入 → 控制任务消费 ======== */
/* 多字段（idx+value）必须成组可见，单靠"32位原子"不够——用临界区。
 * 中断侧用 *_FROM_ISR 变体，任务侧用普通临界区；互斥锁（mutex）不能在
 * 中断里用（FreeRTOS 明确禁止 FromISR 拿 mutex），所以这里选临界区。 */
typedef struct {
    uint8_t idx;  /* 0..5 对应 BKP/BKD/BANG/VKP/VKI/TKP */
    float value;
    volatile uint8_t valid;
} param_write_t;
static param_write_t s_param_write;

/* 前向声明：协议消息处理（先于注册回调定义）。 */
static void App_Car_HandleMsg(const proto_msg_t *msg);
static void App_Car_OnRxByte(uint8_t byte);

/**
 * @brief 应用层初始化（调度器启动前调用）。
 *
 * 顺序要求：先复位解析器，再启动串口接收并注册回调，避免回调打到
 * 未初始化的状态机上；看门狗放最后——启动后控制任务立即开始喂狗。
 */
void App_Car_Init(void)
{
    Kalman_Init(&s_kalman);
    PID_VelocityStateReset(&s_vel_state);
    Proto_Init(&s_parser);
    s_battery_v = 12.0f;
    s_undervolt = 0;
    s_status_req = 0;
    s_param_write.valid = 0;
    s_tilt_latched = 0;
    s_tilt_recover_cnt = 0;

    port_imu_init();
    port_motor_init();
    port_uart_init();
    port_uart_set_rx_cb(App_Car_OnRxByte);
    port_wdt_init();
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
 * 电压显示控制任务采样好的共享值（ADC 归控制任务独占，避免双任务并发转换）。
 */
void App_Car_Display(void)
{
    port_display_show(s_battery_v, s_enc_a, s_enc_b, s_kalman.angle);
}

float App_Car_GetBattery(void)
{
    return s_battery_v;
}

float App_Car_GetAttitude(void)
{
    return s_kalman.angle;
}

uint8_t App_Car_ConsumeStatusReq(void)
{
    /* 单字节读-清，天然原子。 */
    if (s_status_req)
    {
        s_status_req = 0;
        return 1;
    }
    return 0;
}

/**
 * @brief 执行一次平衡车控制运算，并输出到电机。
 *
 * 流程：安全保护（倾角越限停车）→ 欠压检测（降功率）→ 参数消费 →
 * 三环控制（直立 PD + 速度 PI + 转向/遥控差速）→ 统一限幅输出。
 */
void App_Car_PID(void)
{
    int32_t balance_out;
    int32_t velocity_out;
    int32_t turn_out = 0;
    int32_t pwma;
    int32_t pwmb;
    car_params_t p; /* 本周期使用的参数快照（可能被欠压模式修正） */
    float bat;
    float tilt_err;

    /* ---- 1. 倾角保护（锁存式） ------------------------------------------
     * 触发：|融合角 - 目标角| > 45° = 已摔倒 → 刹车并清空控制状态。
     * 为什么锁存：小车连续翻滚时加速度角经 atan2 回绕（231° 读成 -129°），
     * 融合角会被量测拉回、在阈值附近来回穿越——电平式保护会以几十 Hz
     * 反复启停（SIL E6 首版实测到的失效模式）。锁存后只有"接近直立且
     * 车身静止"持续 0.5s 才解锁：翻滚时陀螺数百 deg/s 永远不满足，
     * 人把车扶正放稳 0.5s 后自动恢复——这正是期望的产品行为。 */
    tilt_err = fabsf(s_kalman.angle - g_car_params.balance_angle);
    if (!s_tilt_latched && tilt_err > CAR_TILT_PROTECT_DEG)
    {
        s_tilt_latched = 1;
        s_tilt_recover_cnt = 0;
    }
    if (s_tilt_latched)
    {
        if (tilt_err < CAR_TILT_RECOVER_DEG &&
            fabsf(s_gyro_y) < CAR_TILT_RECOVER_GYRO_DPS)
        {
            if (++s_tilt_recover_cnt >= CAR_TILT_RECOVER_HOLD_COUNT)
            {
                s_tilt_latched = 0; /* 扶正放稳 0.5s，解除锁定 */
            }
        }
        else
        {
            s_tilt_recover_cnt = 0;
        }
        port_motor_set(0, 0);
        PID_VelocityStateReset(&s_vel_state);
        remote_move = 0;
        remote_turn = 0;
        return;
    }

    /* ---- 2. 欠压检测（带迟滞）：ADC 无效值(<0.5V)不进入欠压，避免
     *        采样失败误判。进入后半功率，回升到退出阈值才恢复。 ---- */
    bat = port_battery_read();
    if (bat > 0.5f)
    {
        s_battery_v = bat;
        if (!s_undervolt && bat < CAR_UNDERVOLT_ENTER_V)
        {
            s_undervolt = 1;
        }
        else if (s_undervolt && bat > CAR_UNDERVOLT_EXIT_V)
        {
            s_undervolt = 0;
        }
    }

    /* ---- 3. 消费串口在线调参（临界区保证 idx/value 成组可见） ---- */
    if (s_param_write.valid)
    {
        uint8_t idx;
        float value;
        CAR_ENTER();
        idx = s_param_write.idx;
        value = s_param_write.value;
        s_param_write.valid = 0;
        CAR_EXIT();

        switch (idx)
        {
        case 0: g_car_params.balance_kp = value; break;
        case 1: g_car_params.balance_kd = value; break;
        case 2: g_car_params.balance_angle = value; break;
        case 3: g_car_params.velocity_kp = value; break;
        case 4: g_car_params.velocity_ki = value; break;
        case 5: g_car_params.turn_kp = value; break;
        default: break;
        }
    }

    /* ---- 4. 本周期参数快照：欠压时统一降半功率 ---- */
    p = g_car_params;
    if (s_undervolt)
    {
        p.pwm_limit = CAR_PWM_LIMIT / CAR_UNDERVOLT_PWM_SCALE_DIV;
    }

    /* ---- 5. 三环控制 ---- */
    /* 直立环：以卡尔曼倾角为输入。 */
    balance_out = PID_Balance(&p, s_kalman.angle, p.balance_angle, s_imu.gy);

    /* 遥控前后 → 速度环目标偏移；无指令时清零，避免残留量影响自动平衡。 */
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
    velocity_out = PID_Velocity(&s_vel_state, &p,
                                (int)s_enc_a, (int)s_enc_b, remote_move);

    /* 转向：遥控时累加差速；不遥控时用转向环抑制 Z 轴自转。 */
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
        turn_out = PID_Turn(&p, s_imu.gz);
    }
    remote_turn = PID_Clamp(remote_turn, p.remote_turn_limit);

    /* 三环叠加：turn_out 与 remote_turn 以相反符号分配给左右轮形成差速，
     * 叠加结果统一限幅后输出，防止超出 PWM 硬件范围（ARR=7199）。 */
    pwma = PID_Clamp(balance_out + velocity_out + turn_out + remote_turn, p.pwm_limit);
    pwmb = PID_Clamp(balance_out + velocity_out - turn_out - remote_turn, p.pwm_limit);
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

/* 参数名 → 索引；未知返回 0xFF。 */
static uint8_t ParamNameToIdx(const char *name)
{
    if (strcmp(name, "BKP") == 0) return 0;
    if (strcmp(name, "BKD") == 0) return 1;
    if (strcmp(name, "BANG") == 0) return 2;
    if (strcmp(name, "VKP") == 0) return 3;
    if (strcmp(name, "VKI") == 0) return 4;
    if (strcmp(name, "TKP") == 0) return 5;
    return 0xFF;
}

/**
 * @brief 把一条协议指令落到运行状态/参数上（中断上下文）。
 *
 * 并发说明见 s_param_write 定义处：方向标志是单字节原子写；
 * 参数走"待写槽位 + 临界区"，由控制任务消费；
 * @ST# 只置请求位，打印延迟到显示任务执行。
 */
static void App_Car_HandleMsg(const proto_msg_t *msg)
{
    switch (msg->cmd)
    {
    case PROTO_MOVE:
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
        float value;
        uint8_t idx;
        if (!Proto_ParseFloat(msg->arg2, &value))
        {
            break;
        }
        idx = ParamNameToIdx(msg->arg1);
        if (idx == 0xFF)
        {
            break;
        }
        /* 临界区（ISR 形态）：与控制任务的消费互斥，保证 idx/value 成组发布。 */
        {
            uint32_t saved = CAR_ENTER_ISR();
            s_param_write.idx = idx;
            s_param_write.value = value;
            s_param_write.valid = 1;
            CAR_EXIT_ISR(saved);
        }
        break;
    }
    case PROTO_STATUS:
        s_status_req = 1; /* 中断只置位，打印在显示任务 */
        break;
    default:
        break;
    }
}
