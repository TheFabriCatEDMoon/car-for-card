#ifndef BUZZER_H
#define BUZZER_H

#include <stdbool.h>
#include "main.h"

/*
 * 有源蜂鸣器：电平驱动，低电平响。
 * PC14 在 STM32F103 上是 OSC32_IN，本工程没开 LSE（只用 HSE + PLL），
 * 可以当普通 GPIO 用，所以不走 CubeMX，驱动自己配置。
 */
#define BUZZER_GPIO_Port GPIOC
#define BUZZER_Pin       GPIO_PIN_14

void Buzzer_Init(void);
void Buzzer_Set(bool on);
void Buzzer_Off(void);

#endif /* BUZZER_H */
