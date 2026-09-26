#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"


uint8_t OLED_GRAM[144][8];

/* 设置 OLED 正常显示或反色显示。 */
void OLED_ColorTurn(uint8_t i)
{
    if (i == 0)
    {
        OLED_WR_Byte(0xA6, OLED_CMD); // 正常显示：显存中的 1 对应点亮像素。
    }
    if (i == 1)
    {
        OLED_WR_Byte(0xA7, OLED_CMD); // 反色显示：亮暗像素取反。
    }
}

/* 设置屏幕显示方向，可用于适配 OLED 模块的实际安装方向。 */
void OLED_DisplayTurn(uint8_t i)
{
    if (i == 0)
    {
        OLED_WR_Byte(0xC8, OLED_CMD); // 正常扫描方向。
        OLED_WR_Byte(0xA1, OLED_CMD);
    }
    if (i == 1)
    {
        OLED_WR_Byte(0xC0, OLED_CMD); // 上下/左右方向反转显示。
        OLED_WR_Byte(0xA0, OLED_CMD);
    }
}

/* 通过 SPI 向 OLED 写入 1 个命令或数据字节。 */
void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
    if (cmd)
        OLED_DC_Set();
    else
        OLED_DC_Clr();

    OLED_CS_Clr();
    HAL_SPI_Transmit(&hspi1,&dat,1,2000);
    OLED_CS_Set();

    OLED_DC_Set();
}

/* 开启 OLED 显示输出。 */
void OLED_DisPlay_On(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD); // 选择电荷泵控制命令。
    OLED_WR_Byte(0x14, OLED_CMD); // 开启电荷泵。
    OLED_WR_Byte(0xAF, OLED_CMD); // 打开屏幕显示。
}

/* 关闭 OLED 显示输出，可用于降低功耗。 */
void OLED_DisPlay_Off(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD); // 选择电荷泵控制命令。
    OLED_WR_Byte(0x10, OLED_CMD); // 关闭电荷泵。
    OLED_WR_Byte(0xAE, OLED_CMD); // 关闭屏幕显示。
}

/* 将 OLED_GRAM 显存缓冲区完整刷新到 OLED 屏幕。 */
void OLED_Refresh(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD); // 设置页地址，每页对应 8 个垂直像素。
        OLED_WR_Byte(0x00, OLED_CMD);     // 设置列地址低 4 位。
        OLED_WR_Byte(0x10, OLED_CMD);     // 设置列地址高 4 位。
        for (n = 0; n < 128; n++)
            OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
    }
}
/* 清空显存并刷新屏幕，相当于整屏熄灭。 */
void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        for (n = 0; n < 128; n++)
        {
            OLED_GRAM[n][i] = 0; // 清除该列该页的 8 个像素。
        }
    }
    OLED_Refresh(); // 清空显存后立即刷新到屏幕。
}

/* 在显存中画一个点。
 * x: 横坐标，范围 0~127。
 * y: 纵坐标，范围 0~63。
 * t: 1 表示点亮像素，0 表示清除像素。
 */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    uint8_t i, m, n;
    i = y / 8;
    m = y % 8;
    n = 1 << m;
    if (t)
    {
        OLED_GRAM[x][i] |= n;
    }
    else
    {
        OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
        OLED_GRAM[x][i] |= n;
        OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
    }
}

/* 在显存中画一条直线，使用增量误差法逐点绘制。
 * x1,y1: 起点坐标。
 * x2,y2: 终点坐标。
 * mode: 1 点亮线段，0 清除线段。
 */
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1; // 计算横向增量。
    delta_y = y2 - y1;
    uRow = x1; // 当前绘制点的横坐标。
    uCol = y1;
    if (delta_x > 0)
        incx = 1; // 横坐标递增。
    else if (delta_x == 0)
        incx = 0; // 垂直线，横坐标不变。
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0; // 水平线，纵坐标不变。
    else
    {
        incy = -1;
        delta_y = -delta_x;
    }
    if (delta_x > delta_y)
        distance = delta_x; // 以变化更大的轴作为步进基准。
    else
        distance = delta_y;
    for (t = 0; t < distance + 1; t++)
    {
        OLED_DrawPoint(uRow, uCol, mode); // 绘制当前点。
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}
/* 在显存中画圆。
 * x,y: 圆心坐标。
 * r: 圆半径。
 */
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r)
{
    int a, b, num;
    a = 0;
    b = r;
    while (2 * b * b >= r * r)
    {
        OLED_DrawPoint(x + a, y - b, 1);
        OLED_DrawPoint(x - a, y - b, 1);
        OLED_DrawPoint(x - a, y + b, 1);
        OLED_DrawPoint(x + a, y + b, 1);

        OLED_DrawPoint(x + b, y + a, 1);
        OLED_DrawPoint(x + b, y - a, 1);
        OLED_DrawPoint(x - b, y - a, 1);
        OLED_DrawPoint(x - b, y + a, 1);

        a++;
        num = (a * a + b * b) - r * r; // 判断当前点是否已经超出目标半径。
        if (num > 0)
        {
            b--;
            a--;
        }
    }
}

/* 在指定位置显示一个 ASCII 字符。
 * x,y: 字符左上角坐标。
 * size1: 字号，可选 8、12、16、24，对应不同点阵表。
 * mode: 0 反色显示，1 正常显示。
 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode)
{
    uint8_t i, m, temp, size2, chr1;
    uint8_t x0 = x, y0 = y;
    if (size1 == 8)
        size2 = 6;
    else
        size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2); // 计算一个字符点阵占用的字节数。
    chr1 = chr - ' ';                                              // ASCII 点阵表从空格字符开始存储。
    for (i = 0; i < size2; i++)
    {
        if (size1 == 8)
        {
            temp = asc2_0806[chr1][i];
        } // 使用 6x8 字体。
        else if (size1 == 12)
        {
            temp = asc2_1206[chr1][i];
        } // 使用 6x12 字体。
        else if (size1 == 16)
        {
            temp = asc2_1608[chr1][i];
        } // 使用 8x16 字体。
        else if (size1 == 24)
        {
            temp = asc2_2412[chr1][i];
        } // 使用 12x24 字体。
        else
            return;
        for (m = 0; m < 8; m++)
        {
            if (temp & 0x01)
                OLED_DrawPoint(x, y, mode);
            else
                OLED_DrawPoint(x, y, !mode);
            temp >>= 1;
            y++;
        }
        x++;
        if ((size1 != 8) && ((x - x0) == size1 / 2))
        {
            x = x0;
            y0 = y0 + 8;
        }
        y = y0;
    }
}

/* 显示 ASCII 字符串。
 * x,y: 字符串起点坐标。
 * chr: 字符串起始地址，仅显示空格到 ~ 之间的可见 ASCII 字符。
 * size1: 字号。
 * mode: 0 反色显示，1 正常显示。
 */
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size1, uint8_t mode)
{
    while ((*chr >= ' ') && (*chr <= '~')) // 遇到非可见 ASCII 字符时停止显示。
    {
        OLED_ShowChar(x, y, *chr, size1, mode);
        if (size1 == 8)
            x += 6;
        else
            x += size1 / 2;
        chr++;
    }
}

/* 计算 m 的 n 次方，供数字拆位显示使用。 */
uint32_t OLED_Pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--)
    {
        result *= m;
    }
    return result;
}

/* 显示无符号整数。
 * x,y: 起点坐标。
 * num: 要显示的数字。
 * len: 固定显示位数，不足时前面补 0。
 * size1: 字号。
 * mode: 0 反色显示，1 正常显示。
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode)
{
    uint8_t t, temp, m = 0;
    if (size1 == 8)
        m = 2;
    for (t = 0; t < len; t++)
    {
        temp = (num / OLED_Pow(10, len - t - 1)) % 10;
        if (temp == 0)
        {
            OLED_ShowChar(x + (size1 / 2 + m) * t, y, '0', size1, mode);
        }
        else
        {
            OLED_ShowChar(x + (size1 / 2 + m) * t, y, temp + '0', size1, mode);
        }
    }
}

/* 显示单个汉字点阵。
 * x,y: 汉字左上角坐标。
 * num: 汉字在字库数组中的序号。
 * size1: 汉字字号，可选 16、24、32、64。
 * mode: 0 反色显示，1 正常显示。
 */
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1, uint8_t mode)
{
    uint8_t m, temp;
    uint8_t x0 = x, y0 = y;
    uint16_t i, size3 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * size1; // 计算一个汉字点阵占用的字节数。
    for (i = 0; i < size3; i++)
    {
        if (size1 == 16)
        {
            temp = Hzk1[num][i];
        } // 使用 16x16 汉字点阵。
        else if (size1 == 24)
        {
            temp = Hzk2[num][i];
        } // 使用 24x24 汉字点阵。
        else if (size1 == 32)
        {
            temp = Hzk3[num][i];
        } // 使用 32x32 汉字点阵。
        else if (size1 == 64)
        {
            temp = Hzk4[num][i];
        } // 使用 64x64 汉字点阵。
        else
            return;
        for (m = 0; m < 8; m++)
        {
            if (temp & 0x01)
                OLED_DrawPoint(x, y, mode);
            else
                OLED_DrawPoint(x, y, !mode);
            temp >>= 1;
            y++;
        }
        x++;
        if ((x - x0) == size1)
        {
            x = x0;
            y0 = y0 + 8;
        }
        y = y0;
    }
}

/* 汉字滚动显示。
 * num: 参与滚动显示的汉字数量。
 * space: 每轮显示之间的空白间隔。
 * mode: 0 反色显示，1 正常显示。
 */
void OLED_ScrollDisplay(uint8_t num, uint8_t space, uint8_t mode)
{
    uint8_t i, n, t = 0, m = 0, r;
    while (1)
    {
        if (m == 0)
        {
            OLED_ShowChinese(128, 24, t, 16, mode); // 在屏幕右侧外写入一个汉字，随后通过搬移显存实现滚动。
            t++;
        }
        if (t == num)
        {
            for (r = 0; r < 16 * space; r++) // 控制两轮滚动之间的空白间隔。
            {
                for (i = 1; i < 144; i++)
                {
                    for (n = 0; n < 8; n++)
                    {
                        OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
                    }
                }
                OLED_Refresh();
            }
            t = 0;
        }
        m++;
        if (m == 16)
        {
            m = 0;
        }
        for (i = 1; i < 144; i++) // 整体显存左移一列，形成滚动效果。
        {
            for (n = 0; n < 8; n++)
            {
                OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
            }
        }
        OLED_Refresh();
    }
}

/* 显示位图图片。
 * x,y: 图片左上角坐标。
 * sizex,sizey: 图片宽度和高度，单位为像素。
 * BMP: 图片点阵数组。
 * mode: 0 反色显示，1 正常显示。
 */
void OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[], uint8_t mode)
{
    uint16_t j = 0;
    uint8_t i, n, temp, m;
    uint8_t x0 = x, y0 = y;
    sizey = sizey / 8 + ((sizey % 8) ? 1 : 0);
    for (n = 0; n < sizey; n++)
    {
        for (i = 0; i < sizex; i++)
        {
            temp = BMP[j];
            j++;
            for (m = 0; m < 8; m++)
            {
                if (temp & 0x01)
                    OLED_DrawPoint(x, y, mode);
                else
                    OLED_DrawPoint(x, y, !mode);
                temp >>= 1;
                y++;
            }
            x++;
            if ((x - x0) == sizex)
            {
                x = x0;
                y0 = y0 + 8;
            }
            y = y0;
        }
    }
}
/* OLED 初始化：复位屏幕并写入控制器所需的显示参数。 */
void OLED_Init(void)
{

    OLED_RES_Clr();
    HAL_Delay(200);
    OLED_RES_Set();

    OLED_WR_Byte(0xAE, OLED_CMD); // 关闭显示，避免初始化过程中屏幕闪烁。
    OLED_WR_Byte(0x00, OLED_CMD); // 设置列地址低 4 位。
    OLED_WR_Byte(0x10, OLED_CMD); // 设置列地址高 4 位。
    OLED_WR_Byte(0x40, OLED_CMD); // 设置显示起始行地址。
    OLED_WR_Byte(0x81, OLED_CMD); // 设置对比度控制寄存器。
    OLED_WR_Byte(0xCF, OLED_CMD); // 设置显示亮度。
    OLED_WR_Byte(0xA1, OLED_CMD); // 设置 SEG 列映射，0xA0 左右反置，0xA1 正常。
    OLED_WR_Byte(0xC8, OLED_CMD); // 设置 COM 行扫描方向，0xC0 上下反置，0xC8 正常。
    OLED_WR_Byte(0xA6, OLED_CMD); // 设置为正常显示模式。
    OLED_WR_Byte(0xA8, OLED_CMD); // 设置多路复用比例。
    OLED_WR_Byte(0x3f, OLED_CMD); // 1/64 占空比，适配 64 像素高度屏幕。
    OLED_WR_Byte(0xD3, OLED_CMD); // 设置显示偏移。
    OLED_WR_Byte(0x00, OLED_CMD); // 不使用显示偏移。
    OLED_WR_Byte(0xd5, OLED_CMD); // 设置显示时钟分频和振荡频率。
    OLED_WR_Byte(0x80, OLED_CMD); // 使用默认分频配置。
    OLED_WR_Byte(0xD9, OLED_CMD); // 设置预充电周期。
    OLED_WR_Byte(0xF1, OLED_CMD); // 设置预充电与放电时钟周期。
    OLED_WR_Byte(0xDA, OLED_CMD); // 设置 COM 引脚硬件配置。
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD); // 设置 VCOMH 电压倍率。
    OLED_WR_Byte(0x40, OLED_CMD); // 设置 VCOMH 取消选择电平。
    OLED_WR_Byte(0x20, OLED_CMD); // 设置内存地址模式。
    OLED_WR_Byte(0x02, OLED_CMD); //
    OLED_WR_Byte(0x8D, OLED_CMD); // 设置电荷泵开关。
    OLED_WR_Byte(0x14, OLED_CMD); // 开启电荷泵。
    OLED_WR_Byte(0xA4, OLED_CMD); // 关闭全屏强制点亮模式，改由显存控制像素。
    OLED_WR_Byte(0xA6, OLED_CMD); // 关闭反色显示。
    OLED_Clear();
    OLED_WR_Byte(0xAF, OLED_CMD);
}
