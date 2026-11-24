#ifndef __OLED_SOFTWARE_I2C_H
#define __OLED_SOFTWARE_I2C_H

#include <stdint.h>

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

/* Font size macros (height in pixels) */
#define OLED_8X16   16
#define OLED_6X8    8

//OLED控制用函数
void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t sizey);
uint32_t oled_pow(uint8_t m,uint8_t n);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t sizey);
void OLED_ShowFloatNum(uint8_t x,uint8_t y,double num,uint8_t intlen,uint8_t fraclen,uint8_t sizey);
void OLED_ShowString(uint8_t x,uint8_t y,const char *chr,uint8_t sizey);
void OLED_ShowStringInvert(uint8_t x,uint8_t y,uint8_t *chr);     /* 6x8 反色字符串 */
void OLED_FillPage(uint8_t y);                                     /* 页 y 全填充 0xFF (白底) */
void OLED_ClearPage(uint8_t y);                                    /* 页 y 全填充 0x00 (黑底) */
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t no,uint8_t sizey);
void OLED_DrawBMP(uint8_t x,uint8_t y,uint8_t sizex, uint8_t sizey,uint8_t BMP[]);
/* Call after HAL_Init(), SystemClock_Config(), and MX_GPIO_Init(). */
void OLED_Init(void);
void OLED_Update(void);

#endif /* #ifndef __OLED_SOFTWARE_I2C_H */
