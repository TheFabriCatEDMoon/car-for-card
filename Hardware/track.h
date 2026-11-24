#ifndef __TRACK_H
#define __TRACK_H

#include "main.h"

/* 灰度接口（按原理图：灰度座 3=CLK、4=DAT）
 *   CLK = PB0  推挽输出，空闲低
 *   DAT = PB1  浮空输入
 *
 * 注意：本板电机 PWM 在 PA8/PA11(TIM1)，PB0/PB1 专门给灰度，
 *       不要再把 PB0/PB1 配成 TIM3 复用。
 * 以后真要改线，只改下面 4 个宏。
 */
#define GRAY_CLK_PORT   GPIOB
#define GRAY_CLK_PIN    GPIO_PIN_0

#define GRAY_DAT_PORT   GPIOB
#define GRAY_DAT_PIN    GPIO_PIN_1

/* 初始化：配好 CLK/DAT 引脚、CLK 拉低、等传感器上电、丢掉第一帧 */
void    Track_Init(void);

/* 读一帧 8 路：bit0=CH1 … bit7=CH8，1=白、0=黑（厂家原始语义） */
uint8_t Track_Read_Serial(void);

typedef enum {
    TRACK_LINE,
    TRACK_LOST,
    TRACK_AMBIGUOUS
} TrackState;

/* 仅连续且宽度合理的黑线有效。有效误差以探头间距为单位，左负右正。 */
TrackState Track_Decode(uint8_t gray, float *error);

/* 统计一帧里有几路探头压黑（按 CAR_BLACK_BIT 的电平约定） */
uint8_t Track_BlackCount(uint8_t gray);

/* 兼容旧调用。控制代码通过 Track_Decode 区分居中和丢线。 */
float   Get_Line_Error(uint8_t gray);

// 滑动平均滤波函数
float Error_Moving_Average(float new_error);

#endif
