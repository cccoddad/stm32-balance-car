#include "bsp_switch.h"

#if BSP_USE_REG

/**
 * @file bsp_imu_reg.c
 * @brief MPU6050 驱动的寄存器版：软件模拟 I2C 时序（GPIO 位拍），无 HAL 总线调用。
 *
 * 与 bsp_imu.c（HAL 硬件 I2C 版）符号完全一致，靠 bsp_switch.h 互斥编译。
 *
 * 时序对照 STM32F1 参考手册与 MPU6050 产品说明书：
 *   起始条件：SCL 高电平期间 SDA 由高变低
 *   停止条件：SCL 高电平期间 SDA 由低变高
 *   每字节：8 个时钟位（SCL 高电平采样 SDA）+ 1 个应答位（主机读 SDA，0=ACK）
 *   读多字节：最后 1 位主机发 NACK 再停止，防止从机继续占用总线
 *
 * 引脚：I2C2 的 PB10(SCL) / PB11(SDA)，在初始化时从 AF 模式重配为
 * 开漏输出（外部 4.7k 上拉由模块板载）。
 */
#include "bsp_imu.h"
#include "main.h"

/* 前向声明：ReadReg 转发到 ReadRegs。 */
uint8_t BSP_IMU_ReadRegs(uint8_t reg_addr, uint8_t *receive_buff, uint8_t size);

/* ======== 位拍时序参数 ======== */
/* 半周期延时循环数：约 2.5us → ~100kHz。72MHz 下每轮循环约 4 个周期估算。
 * 若上板通信不稳，先调这个宏（数据手册 tHIGH/tLOW 最小值 0.6us 以上即可）。 */
#define I2C_DLY (40u)

#define I2C_SCL_PIN GPIO_PIN_10
#define I2C_SDA_PIN GPIO_PIN_11

/* ======== 底层位拍操作（GPIOB->BSRR/IDR，直接寄存器） ======== */
static void I2C_Delay(void)
{
    volatile uint32_t n = I2C_DLY;
    while (n--)
    {
        __NOP();
    }
}

static void SCL_H(void) { GPIOB->BSRR = I2C_SCL_PIN; }
static void SCL_L(void) { GPIOB->BSRR = (uint32_t)I2C_SCL_PIN << 16; }
static void SDA_H(void) { GPIOB->BSRR = I2C_SDA_PIN; }
static void SDA_L(void) { GPIOB->BSRR = (uint32_t)I2C_SDA_PIN << 16; }
/* 开漏输出写 1 即释放总线，直接读 IDR 即可采样从机/上拉电平。 */
static uint8_t SDA_R(void) { return (GPIOB->IDR & I2C_SDA_PIN) ? 1u : 0u; }

static void I2C_Start(void)
{
    SDA_H();
    SCL_H();
    I2C_Delay();
    SDA_L(); /* SCL 高时 SDA 下降 = 起始 */
    I2C_Delay();
    SCL_L();
    I2C_Delay();
}

static void I2C_Stop(void)
{
    SDA_L();
    I2C_Delay();
    SCL_H();
    I2C_Delay();
    SDA_H(); /* SCL 高时 SDA 上升 = 停止 */
    I2C_Delay();
}

/* 发送 1 字节并读应答：返回 1 = 收到 ACK。 */
static uint8_t I2C_WriteByte(uint8_t data)
{
    uint8_t i;
    uint8_t ack;
    for (i = 0; i < 8; i++)
    {
        if (data & 0x80u)
        {
            SDA_H();
        }
        else
        {
            SDA_L();
        }
        data <<= 1;
        I2C_Delay();
        SCL_H();
        I2C_Delay();
        SCL_L();
        I2C_Delay();
    }
    SDA_H(); /* 释放数据线，等从机拉低应答 */
    I2C_Delay();
    SCL_H();
    I2C_Delay();
    ack = (uint8_t)(SDA_R() == 0u); /* 从机 ACK 会把 SDA 拉低 */
    SCL_L();
    I2C_Delay();
    return ack;
}

/* 接收 1 字节；ack=1 回 ACK（继续），ack=0 回 NACK（最后一字节）。 */
static uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t i;
    uint8_t data = 0;
    SDA_H(); /* 释放总线，由从机驱动 */
    for (i = 0; i < 8; i++)
    {
        data <<= 1;
        SCL_H();
        I2C_Delay();
        data |= (uint8_t)SDA_R();
        SCL_L();
        I2C_Delay();
    }
    if (ack)
    {
        SDA_L();
    }
    else
    {
        SDA_H(); /* NACK：告诉从机这是最后一个字节 */
    }
    I2C_Delay();
    SCL_H();
    I2C_Delay();
    SCL_L();
    SDA_H(); /* 释放数据线 */
    I2C_Delay();
    return data;
}

/* ======== MPU6050 寄存器读写（与 HAL 版同名同语义，返回 0=成功） ======== */

uint8_t BSP_IMU_WriteReg(uint8_t reg_addr, uint8_t write_byte)
{
    I2C_Start();
    if (!I2C_WriteByte((uint8_t)(MPU_IIC_ADDR << 1))) /* 地址+W 位 */
    {
        I2C_Stop();
        return 1;
    }
    I2C_WriteByte(reg_addr);
    I2C_WriteByte(write_byte);
    I2C_Stop();
    return 0;
}

uint8_t BSP_IMU_WriteRegs(uint8_t reg_addr, uint8_t *write_bytes, uint8_t size)
{
    uint8_t i;
    I2C_Start();
    if (!I2C_WriteByte((uint8_t)(MPU_IIC_ADDR << 1)))
    {
        I2C_Stop();
        return 1;
    }
    I2C_WriteByte(reg_addr);
    for (i = 0; i < size; i++)
    {
        I2C_WriteByte(write_bytes[i]);
    }
    I2C_Stop();
    return 0;
}

uint8_t BSP_IMU_ReadReg(uint8_t reg_addr, uint8_t *receive_byte)
{
    return BSP_IMU_ReadRegs(reg_addr, receive_byte, 1);
}

uint8_t BSP_IMU_ReadRegs(uint8_t reg_addr, uint8_t *receive_buff, uint8_t size)
{
    uint8_t i;
    if (size == 0u)
    {
        return 0;
    }
    I2C_Start();
    if (!I2C_WriteByte((uint8_t)(MPU_IIC_ADDR << 1)))
    {
        I2C_Stop();
        return 1;
    }
    I2C_WriteByte(reg_addr);
    /* 重复起始 + 读地址：不释放总线，保证寄存器地址与读操作的原子性。 */
    I2C_Start();
    if (!I2C_WriteByte((uint8_t)((MPU_IIC_ADDR << 1) | 1u)))
    {
        I2C_Stop();
        return 1;
    }
    for (i = 0; i < (uint8_t)(size - 1u); i++)
    {
        receive_buff[i] = I2C_ReadByte(1); /* ACK 继续 */
    }
    receive_buff[size - 1u] = I2C_ReadByte(0); /* 最后一字节 NACK */
    I2C_Stop();
    return 0;
}

/* ======== 以下为与 bsp_imu.c 相同的高层逻辑（仅底层读写不同） ======== */

static void BSP_IMU_SetDlpf(uint16_t rate)
{
    /* 采样定理：采样率 >= 2 * 带宽，才不会发生明显混叠。 */
    uint8_t cfg = 0;
    rate = rate / 2;
    if (rate > 188)
    {
        cfg = 1;
    }
    else if (rate > 98)
    {
        cfg = 2;
    }
    else if (rate > 42)
    {
        cfg = 3;
    }
    else if (rate > 20)
    {
        cfg = 4;
    }
    else if (rate > 10)
    {
        cfg = 5;
    }
    else
    {
        cfg = 6;
    }
    BSP_IMU_WriteReg(MPU_CFG_REG, cfg);
}

static void BSP_IMU_SetGyroRate(uint16_t rate)
{
    uint8_t sample_div = 0;
    if (rate < 4)
    {
        rate = 4;
    }
    else if (rate > 1000)
    {
        rate = 1000;
    }
    sample_div = (uint8_t)(1000 / rate - 1);
    BSP_IMU_WriteReg(MPU_SAMPLE_RATE_REG, sample_div);
    BSP_IMU_SetDlpf(rate);
}

/**
 * @brief 初始化 MPU6050（寄存器版）。
 *
 * 先把 I2C2 外设关闭、PB10/11 从复用开漏重配为普通开漏输出（位拍模式），
 * 再执行与 HAL 版完全相同的初始化序列。
 */
void BSP_IMU_Init(void)
{
    uint8_t dev_id = 0;

    /* 1. 关闭硬件 I2C2（寄存器级），释放引脚控制权。 */
    I2C2->CR1 &= ~I2C_CR1_PE;
    RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;

    /* 2. PB10(SCL)/PB11(SDA) 配为开漏输出 50MHz（CRH：MODE=11, CNF=01 → 0x14）。
     *    等效 HAL_GPIO_Init 的 GPIO_MODE_OUTPUT_OD，这里直接写 CRH 半字节。 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    {
        uint32_t crh = GPIOB->CRH;
        crh &= ~(0x00FFu << 8); /* 清掉 pin10/pin11 的 CNF+MODE */
        crh |= (0x14u << 8) | (0x14u << 12);
        GPIOB->CRH = crh;
    }
    /* 初始释放总线。 */
    SCL_H();
    SDA_H();

    /* 3. 以下初始化序列与 bsp_imu.c（HAL 版）逐行对应，保证两版行为一致。 */
    /* 复位 → 唤醒。HAL_Delay 走的是系统时基（TIM1），不属于 I2C 总线抽象，
     * 保留使用；纯寄存器实现连时基也可换成 DWT 周计数，属可选加深项。 */
    BSP_IMU_WriteReg(MPU_PWR_MGMT1_REG, 0x80);
    HAL_Delay(300);
    BSP_IMU_WriteReg(MPU_PWR_MGMT1_REG, 0x00);

    /* 陀螺 ±2000 度/秒（FSR=3），加速度 ±2g（AFS=0）。 */
    BSP_IMU_WriteReg(MPU_GYRO_CFG_REG, 3 << 3);
    BSP_IMU_WriteReg(MPU_ACCEL_CFG_REG, 0 << 3);

    /* 关闭本工程不用的功能。 */
    BSP_IMU_WriteReg(MPU_INT_EN_REG, 0x00);
    BSP_IMU_WriteReg(MPU_USER_CTRL_REG, 0x00);
    BSP_IMU_WriteReg(MPU_FIFO_EN_REG, 0x00);

    /* 读器件 ID 验证位拍通信。 */
    BSP_IMU_ReadReg(MPU_DEVICE_ID_REG, &dev_id);
    if (dev_id == MPU_IIC_ADDR)
    {
        BSP_IMU_WriteReg(MPU_PWR_MGMT1_REG, 0x01); /* X 轴陀螺 PLL 时钟源 */
        BSP_IMU_SetGyroRate(100);
        BSP_IMU_WriteReg(MPU_PWR_MGMT2_REG, 0x00);
    }
}

void BSP_IMU_ReadGyro(short *gx, short *gy, short *gz)
{
    uint8_t buff[6];
    BSP_IMU_ReadRegs(MPU_GYRO_XOUTH_REG, buff, 6);
    *gx = (short)(((uint16_t)buff[0] << 8) | buff[1]);
    *gy = (short)(((uint16_t)buff[2] << 8) | buff[3]);
    *gz = (short)(((uint16_t)buff[4] << 8) | buff[5]);
}

void BSP_IMU_ReadAccel(short *ax, short *ay, short *az)
{
    uint8_t buff[6];
    BSP_IMU_ReadRegs(MPU_ACCEL_XOUTH_REG, buff, 6);
    *ax = (short)(((uint16_t)buff[0] << 8) | buff[1]);
    *ay = (short)(((uint16_t)buff[2] << 8) | buff[3]);
    *az = (short)(((uint16_t)buff[4] << 8) | buff[5]);
}

#endif /* BSP_USE_REG */
