#include "buzzer.h"

void Buzzer_Init(void)
{
    GPIO_InitTypeDef init = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);   /* 低电平响，先关掉 */

    init.Pin = BUZZER_Pin;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_GPIO_Port, &init);
}

void Buzzer_Set(bool on)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Buzzer_Off(void)
{
    Buzzer_Set(false);
}
