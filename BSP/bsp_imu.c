#include "bsp_imu.h"

/**
 * @brief 从 MPU6050 指定寄存器读取 1 个字节。
 * @param reg_addr 要读取的寄存器地址。
 * @param receive_byte 用于保存读取结果的缓冲区指针。
 * @return HAL I2C 读操作状态，HAL_OK 表示读取成功。
 */
uint8_t Int_MPU6050_ReadByte(uint8_t reg_addr, uint8_t *receive_byte)
{
    return HAL_I2C_Mem_Read(&hi2c2, MPU_IIC_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, receive_byte, 1, 2000);
}

/**
 * @brief 从 MPU6050 指定起始寄存器连续读取多个字节。
 * @param reg_addr 起始寄存器地址。
 * @param receive_buff 用于保存连续读取数据的缓冲区。
 * @param size 需要读取的字节数。
 * @return HAL I2C 读操作状态，HAL_OK 表示读取成功。
 */
uint8_t Int_MPU6050_ReadBytes(uint8_t reg_addr, uint8_t *receive_buff, uint8_t size)
{
    return HAL_I2C_Mem_Read(&hi2c2, MPU_IIC_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, receive_buff, size, 2000);
}

/**
 * @brief 向 MPU6050 指定寄存器写入 1 个字节。
 * @param reg_addr 要写入的寄存器地址。
 * @param write_byte 要写入的配置值。
 * @return HAL I2C 写操作状态，HAL_OK 表示写入成功。
 */
uint8_t Int_MPU6050_WriteByte(uint8_t reg_addr, uint8_t write_byte)
{
    return HAL_I2C_Mem_Write(&hi2c2, MPU_IIC_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, &write_byte, 1, 2000);
}

/**
 * @brief 从指定起始寄存器开始向 MPU6050 连续写入多个字节。
 * @param reg_addr 起始寄存器地址。
 * @param write_bytes 待写入数据缓冲区。
 * @param size 需要写入的字节数。
 * @return HAL I2C 写操作状态，HAL_OK 表示写入成功。
 */
uint8_t Int_MPU6050_WriteBytes(uint8_t reg_addr, uint8_t *write_bytes, uint8_t size)
{
    return HAL_I2C_Mem_Write(&hi2c2, MPU_IIC_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, write_bytes, size, 2000);
}

/**
 * @brief 根据采样率设置 MPU6050 内部数字低通滤波器。
 * @param rate 期望采样率，单位为 Hz。
 *
 * 数字低通滤波器可以降低传感器高频噪声。根据采样定理，信号带宽应小于
 * 采样率的一半，因此这里先用 rate / 2 得到目标带宽，再选择最接近的配置档位。
 */
void Int_MPU6050_Set_DLPF_CFG(uint16_t rate)
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
    Int_MPU6050_WriteByte(MPU_CFG_REG, cfg << 0);
}

/**
 * @brief 设置陀螺仪采样率，并同步配置数字低通滤波器。
 * @param rate 期望采样率，单位为 Hz。
 */
void Int_MPU6050_SetGyroRate(uint16_t rate)
{
    /* 采样率 = 输出频率 / (1 + 分频值)，所以分频值 = 输出频率 / 采样率 - 1。 */
    uint8_t sample_div = 0;
    /* 1. 采样率限幅：MPU6050 支持的有效采样率有边界，本工程限制到 4~1000 Hz。 */
    if (rate < 4)
    {
        rate = 4;
    }
    else if (rate > 1000)
    {
        rate = 1000;
    }

    /* 2. 根据期望采样率计算分频值。这里按内部 1 kHz 输出频率配置。 */
    sample_div = 1000 / rate - 1;

    /* 3. 写入采样率分频寄存器。 */
    Int_MPU6050_WriteByte(MPU_SAMPLE_RATE_REG, sample_div);

    /* 4. 采样率变化后同步调整低通滤波器带宽。 */
    Int_MPU6050_Set_DLPF_CFG(rate);
}

/**
 * @brief 初始化 MPU6050。
 *
 * 初始化流程包括：复位并唤醒芯片、配置陀螺仪和加速度计量程、关闭本工程不用的
 * 中断/FIFO/第二 I2C 功能、确认设备 ID，然后设置时钟源和采样率。
 */
void Int_MPU6050_Init(void)
{
    uint8_t dev_id = 0;

    /* 1. 复位 MPU6050，等待内部状态稳定后再唤醒。 */
    Int_MPU6050_WriteByte(MPU_PWR_MGMT1_REG, 0x80);
    HAL_Delay(300);
    Int_MPU6050_WriteByte(MPU_PWR_MGMT1_REG, 0x00);

    /* 2. 陀螺仪量程设置为 +/-2000 度/秒，寄存器 FSR 配置值为 3。 */
    Int_MPU6050_WriteByte(MPU_GYRO_CFG_REG, 3 << 3);

    /* 3. 加速度计量程设置为 +/-2g，寄存器 AFS_SEL 配置值为 0。 */
    Int_MPU6050_WriteByte(MPU_ACCEL_CFG_REG, 0 << 3);

    /* 4. 关闭本工程暂时不用的功能，减少干扰和资源占用。 */
    Int_MPU6050_WriteByte(MPU_INT_EN_REG, 0x00);    // 关闭所有 MPU6050 中断输出。
    Int_MPU6050_WriteByte(MPU_USER_CTRL_REG, 0x00); // 关闭第二 I2C 主机功能。
    Int_MPU6050_WriteByte(MPU_FIFO_EN_REG, 0x00);   // 关闭 FIFO 缓冲。

    /* 5. 读取设备 ID，确认 I2C 通信和芯片地址正确。 */
    Int_MPU6050_ReadByte(MPU_DEVICE_ID_REG, &dev_id);
    if (dev_id == MPU_IIC_ADDR)
    {
        /* 5.1 使用 X 轴陀螺仪 PLL 作为时钟源，精度比内部 RC 振荡器更高。 */
        Int_MPU6050_WriteByte(MPU_PWR_MGMT1_REG, 0x01);
        /* 5.2 设置陀螺仪采样率和数字低通滤波器。 */
        Int_MPU6050_SetGyroRate(100);
        /* 5.3 让加速度计和陀螺仪退出待机模式，进入正常工作状态。 */
        Int_MPU6050_WriteByte(MPU_PWR_MGMT2_REG, 0x00);
    }
}

/**
 * @brief 读取三轴陀螺仪原始角速度数据。
 * @param gx X 轴角速度输出指针。
 * @param gy Y 轴角速度输出指针。
 * @param gz Z 轴角速度输出指针。
 *
 * MPU6050 每个轴的数据由高 8 位和低 8 位组成，拼接后转换为 short，
 * 即得到带符号的 16 位原始数据。
 */
void Int_MPU6050_Get_Gyro(short *gx, short *gy, short *gz)
{
    uint8_t buff[6];
    Int_MPU6050_ReadBytes(MPU_GYRO_XOUTH_REG, buff, 6);

    /*
        buff[0]: X 轴角速度高 8 位
        buff[1]: X 轴角速度低 8 位
        buff[2]: Y 轴角速度高 8 位
        buff[3]: Y 轴角速度低 8 位
        buff[4]: Z 轴角速度高 8 位
        buff[5]: Z 轴角速度低 8 位
     */
    *gx = ((short)buff[0] << 8) | buff[1];
    *gy = ((short)buff[2] << 8) | buff[3];
    *gz = ((short)buff[4] << 8) | buff[5];
}

/**
 * @brief 读取三轴加速度计原始加速度数据。
 * @param ax X 轴加速度输出指针。
 * @param ay Y 轴加速度输出指针。
 * @param az Z 轴加速度输出指针。
 *
 * 读取的是 MPU6050 原始 ADC 数值，不在这里换算成 g。姿态解算时只关心
 * 各轴之间的比例关系，因此可以直接使用原始值计算倾角。
 */
void Int_MPU6050_Get_Accel(short *ax, short *ay, short *az)
{
    uint8_t buff[6];
    Int_MPU6050_ReadBytes(MPU_ACCEL_XOUTH_REG, buff, 6);

    /*
        buff[0]: X 轴加速度高 8 位
        buff[1]: X 轴加速度低 8 位
        buff[2]: Y 轴加速度高 8 位
        buff[3]: Y 轴加速度低 8 位
        buff[4]: Z 轴加速度高 8 位
        buff[5]: Z 轴加速度低 8 位
     */
    *ax = ((short)buff[0] << 8) | buff[1];
    *ay = ((short)buff[2] << 8) | buff[3];
    *az = ((short)buff[4] << 8) | buff[5];
}
