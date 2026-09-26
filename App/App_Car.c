#include "App_Car.h"
#include "Car_Config.h"
#include <stdio.h>

short gx, gy, gz;
short ax, ay, az;

float accel_angle;  // 由加速度计换算得到的车身倾角，单位为度。
float gyro_y;       // Y 轴角速度，使用陀螺仪原始值换算得到，单位为度/秒。
extern float angle; // 卡尔曼滤波后的最终倾角，定义在滤波模块中。

int ea, eb;

/* 显示缓冲区统一留 8 字节：覆盖最坏格式化长度（含符号、小数位）再加结束符，
 * 配合 snprintf 使用，杜绝 sprintf 无边界写入。 */
char bat_str[8];   // 电池电压：如 "12.3"。
char ea_str[8];    // A 编码器：如 "-32768"。
char eb_str[8];    // B 编码器：如 "-32768"。
char angle_str[8]; // 倾角：如 "-100.5"。

/* 直立环 PID 参数：负责把车身拉回目标平衡角。 */
float balance_kp = -720.0;  // 直立环比例系数，影响扶正力度；符号由电机安装方向和角度方向决定。
float balance_kd = 0.72;    // 直立环微分系数，利用角速度抑制快速倒下和高频振荡。
float balance_angle = -1.0; // 机械安装后的目标平衡角，车不一定在 0 度时刚好直立。
/* 速度环 PID 参数：负责让小车整体速度趋近目标速度，防止长时间向一个方向跑偏。 */
float velocity_kp = 170.0; // 速度环比例系数，决定当前速度偏差对输出的影响。
float velocity_ki = 0.85;  // 速度环积分系数，决定长期速度偏差的修正强度。
/* 转向环 PID 参数：负责在没有遥控转向时抑制 Z 轴自转。 */
float turn_kp = 0.5; // 转向环比例系数，输入为 Z 轴角速度。

/* 遥控运动标志位，由串口接收回调根据 U/D/L/R/S 指令更新。 */
uint8_t flag_up = 0, flag_down = 0, flag_left = 0, flag_right = 0;
int remote_move = 0; // 遥控前进/后退控制量，作为速度环的目标偏移。
int remote_turn = 0; // 遥控左转/右转控制量，直接叠加到左右电机差速上。

/**
 * @brief 获取平衡车姿态与轮速数据。
 *
 * 该函数完成一次控制周期所需的核心采样：先读取 MPU6050 的加速度和角速度，
 * 再通过加速度计算静态倾角，通过陀螺仪得到动态角速度，最后用卡尔曼滤波融合出
 * 更稳定的车身倾角。同时读取两个编码器的增量值，使姿态数据和速度数据保持同频。
 */
void App_Car_GetAngle(void)
{
    /* 1. 读取 MPU6050 的三轴加速度和三轴角速度原始数据。 */
    Int_MPU6050_Get_Accel(&ax, &ay, &az);
    Int_MPU6050_Get_Gyro(&gx, &gy, &gz);

    /* 2. 通过 X/Z 轴加速度计算车身倾角。atan2 返回弧度，需要换算为角度。 */
    accel_angle = atan2(ax, az) * 180 / PI;

    /* 3. 陀螺仪量程设置为 +/-2000 度/秒，比例系数为 65536 / 4000 = 16.4。 */
    /*    这里取反是为了让角速度方向与加速度计算出的倾角方向保持一致。 */
    gyro_y = -gy / 16.4;

    /* 4. 使用卡尔曼滤波融合倾角和角速度，减小加速度噪声与陀螺仪漂移。 */
    Com_Filter_Kalman(accel_angle, gyro_y);

    // printf("accel_angle=%.1f\r\n", accel_angle);
    // printf("gyro_y=%.1f\r\n", gyro_y);
    // printf("angle=%.1f\r\n", angle);

    /* 5. 读取编码器增量。B 轮取反是为了统一两个电机的正方向。 */
    ea = Int_Encoder_ReadCounter(2);
    eb = -Int_Encoder_ReadCounter(3);
}

/**
 * @brief 刷新 OLED 显示内容。
 *
 * 显示电池电压、两个编码器的本周期计数值和卡尔曼滤波后的倾角。
 * OLED 使用显存缓冲，字符串写入显存后需要调用 OLED_Refresh 才会真正显示。
 */
void App_Car_Display(void)
{
    /* 1. 采集并显示电池电压。电阻分压后进入 ADC，因此需要乘以 4 还原实际电压。 */
    float bat_vol = 0.0f;
    /* ADC 为单次转换模式：启动后必须等待转换完成（PollForConversion）才能取值，
     * 否则读到的是上一次的残留结果甚至 0。 */
    if (HAL_ADC_Start(&hadc1) == HAL_OK)
    {
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
            bat_vol = ((float)HAL_ADC_GetValue(&hadc1) * 3.3f / 4095.0f) * 4.0f;
        }
    }

    snprintf(bat_str, sizeof(bat_str), "%3.1f", bat_vol);
    OLED_ShowString(32, 0, bat_str, 16, 1);

    /* 2. 显示左右编码器增量，用于观察电机转速和方向是否一致。 */
    snprintf(ea_str, sizeof(ea_str), "%6d", ea);
    snprintf(eb_str, sizeof(eb_str), "%6d", eb);
    OLED_ShowString(24, 16, ea_str, 16, 1); // 前面已经显示“EA:”，所以从 x=3*8=24 的位置开始写数值。
    OLED_ShowString(24, 32, eb_str, 16, 1); // 前面已经显示“EB:”，第三行 y 坐标为 32。

    /* 3. 显示当前倾角，用来辅助调试直立环目标角和滤波效果。 */
    snprintf(angle_str, sizeof(angle_str), "%5.1f", angle);
    OLED_ShowString(48, 48, angle_str, 16, 1); // 前面已经显示“Angle:”，所以从 x=6*8=48 的位置开始写数值。

    /* 4. 将显存内容一次性刷新到 OLED 屏幕。 */
    OLED_Refresh();
}

/**
 * @brief 把控制量限制在 PWM 允许范围内。
 * @param value 三环叠加后的原始控制量。
 * @return 限制到 [-CAR_PWM_LIMIT, CAR_PWM_LIMIT] 之后的值。
 *
 * 三环输出叠加后可能远超 TIM4 的计数范围（ARR=7199），必须统一限幅：
 * 既防止写入比较寄存器的值超出硬件范围，也避免"饱和后反向迟滞"带来的
 * 积分饱和现象（配合速度环的积分限幅一起工作）。
 */
static int App_Car_LimitPwm(int value)
{
    if (value > CAR_PWM_LIMIT)
    {
        return CAR_PWM_LIMIT;
    }
    if (value < -CAR_PWM_LIMIT)
    {
        return -CAR_PWM_LIMIT;
    }
    return value;
}

/**
 * @brief 执行一次平衡车控制运算，并输出到 TB6612 电机驱动。
 *
 * 控制结构由三部分叠加：
 * 1. 直立环：根据当前角度和目标平衡角输出主要扶正力矩。
 * 2. 速度环：根据编码器速度偏差修正车身前后移动趋势。
 * 3. 转向环/遥控转向：通过左右电机差速控制车身绕 Z 轴旋转。
 */
void App_Car_PID(void)
{
    int balance_out = 0;
    int velocity_out = 0;
    int turn_out = 0;
    int pwma = 0, pwmb = 0;
    /* 1. 直立环控制：输出是左右电机共同的基础 PWM。 */
    balance_out = Com_PID_Balance(balance_kp, balance_kd, angle, balance_angle, gy);

    /* 2. 速度环控制：遥控前后指令通过 remote_move 改变速度环目标。 */
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
        /* 没有前后遥控时清零目标偏移，避免残留控制量影响自动平衡。 */
        remote_move = 0;
    }
    velocity_out = Com_PID_Velocity(velocity_kp, velocity_ki, ea, eb, remote_move);

    /* 3. 转向控制：遥控转向时直接累加差速；不转向时用 Z 轴角速度抑制自转。 */
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
        /* 没有左右遥控时清零遥控差速。 */
        remote_turn = 0;
        /* 只有不主动转向时才启用转向环，避免抵消遥控转向意图。 */
        turn_out = Com_PID_Turn(turn_kp, gz);
    }
    /* 遥控按键持续按住会让差速不断累加，因此需要限幅保护。 */
    if (remote_turn > CAR_REMOTE_TURN_LIMIT)
    {
        remote_turn = CAR_REMOTE_TURN_LIMIT;
    }
    else if (remote_turn < -CAR_REMOTE_TURN_LIMIT)
    {
        remote_turn = -CAR_REMOTE_TURN_LIMIT;
    }

    /* 4. 叠加三环输出。turn_out 和 remote_turn 以相反符号分配给左右轮，形成差速。
     *    叠加结果统一限幅后再写入定时器，防止超出 PWM 硬件范围。 */
    pwma = App_Car_LimitPwm(balance_out + velocity_out + turn_out + remote_turn);
    pwmb = App_Car_LimitPwm(balance_out + velocity_out - turn_out - remote_turn);
    Int_TB6612_SetPWM(pwma, pwmb);
}

/**
 * @brief USART2 接收完成回调，用于处理遥控指令。
 *
 * 每次串口中断接收 1 个字符：
 * U 表示前进，D 表示后退，L 表示左转，R 表示右转，S 表示停止。
 * 回调末尾重新开启中断接收，保证下一次字符仍能进入该回调。
 */
extern uint8_t buff[1];
/* cppcheck-suppress constParameterPointer -- 参数 const 与否由 HAL 回调原型决定，不可改 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        switch (buff[0])
        {
        case 'U':
            flag_up = 1, flag_down = 0, flag_left = 0, flag_right = 0;
            break;
        case 'D':
            flag_up = 0, flag_down = 1, flag_left = 0, flag_right = 0;
            break;
        case 'L':
            flag_up = 0, flag_down = 0, flag_left = 1, flag_right = 0;
            break;
        case 'R':
            flag_up = 0, flag_down = 0, flag_left = 0, flag_right = 1;
            break;
        case 'S':
            flag_up = 0, flag_down = 0, flag_left = 0, flag_right = 0;
            break;
        default:
            flag_up = 0, flag_down = 0, flag_left = 0, flag_right = 0;
            break;
        }
    }
    HAL_UART_Receive_IT(&huart2, buff, 1);
}
