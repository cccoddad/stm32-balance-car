#ifndef __OLED_H
#define __OLED_H 

#include "stdlib.h"	
#include "spi.h"

/* OLED 端口控制宏：根据 CubeMX 生成的 GPIO 名称控制复位、数据/命令和片选引脚。 */ 



#define OLED_RES_Clr() HAL_GPIO_WritePin(OLED_RES_GPIO_Port,OLED_RES_Pin,GPIO_PIN_RESET) // OLED 复位引脚输出低电平。
#define OLED_RES_Set() HAL_GPIO_WritePin(OLED_RES_GPIO_Port,OLED_RES_Pin,GPIO_PIN_SET)   // OLED 复位引脚输出高电平。

#define OLED_DC_Clr()  HAL_GPIO_WritePin(OLED_DC_GPIO_Port,OLED_DC_Pin,GPIO_PIN_RESET) // DC=0 表示后续 SPI 字节是命令。
#define OLED_DC_Set()  HAL_GPIO_WritePin(OLED_DC_GPIO_Port,OLED_DC_Pin,GPIO_PIN_SET)   // DC=1 表示后续 SPI 字节是显示数据。

#define OLED_CS_Clr() HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port,SPI1_NSS_Pin,GPIO_PIN_RESET) // 片选拉低，开始一次 SPI 传输。
#define OLED_CS_Set() HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port,SPI1_NSS_Pin,GPIO_PIN_SET)   // 片选拉高，结束一次 SPI 传输。
 		     


#define OLED_CMD  0	// 写命令。
#define OLED_DATA 1	// 写显示数据。

/* 清除指定像素点。 */
void OLED_ClearPoint(uint8_t x,uint8_t y);
/* 设置正常显示或反色显示。 */
void OLED_ColorTurn(uint8_t i);
/* 设置屏幕显示方向。 */
void OLED_DisplayTurn(uint8_t i);
/* 通过 SPI 写入 1 个命令或数据字节。 */
void OLED_WR_Byte(uint8_t dat,uint8_t mode);
/* 打开 OLED 显示。 */
void OLED_DisPlay_On(void);
/* 关闭 OLED 显示。 */
void OLED_DisPlay_Off(void);
/* 将显存刷新到屏幕。 */
void OLED_Refresh(void);
/* 清空整屏显示。 */
void OLED_Clear(void);
/* 在显存中绘制单个像素点。 */
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
/* 在显存中绘制直线。 */
void OLED_DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t mode);
/* 在显存中绘制圆形。 */
void OLED_DrawCircle(uint8_t x,uint8_t y,uint8_t r);
/* 显示单个 ASCII 字符。 */
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size1,uint8_t mode);
/* 显示 6x8 字号字符。 */
void OLED_ShowChar6x8(uint8_t x,uint8_t y,uint8_t chr,uint8_t mode);
/* 显示 ASCII 字符串。 */
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t size1,uint8_t mode);
/* 显示无符号整数。 */
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1,uint8_t mode);
/* 显示单个汉字点阵。 */
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t num,uint8_t size1,uint8_t mode);
/* 滚动显示汉字。 */
void OLED_ScrollDisplay(uint8_t num,uint8_t space,uint8_t mode);
/* 显示位图图片。 */
void OLED_ShowPicture(uint8_t x,uint8_t y,uint8_t sizex,uint8_t sizey,uint8_t BMP[],uint8_t mode);
/* 初始化 OLED 控制器。 */
void OLED_Init(void);

#endif

