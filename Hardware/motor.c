#include "motor.h"
#include "tim.h"
#include "config.h"

static uint16_t SetDirection(int32_t command, GPIO_TypeDef *in1_port, uint16_t in1,
                             GPIO_TypeDef *in2_port, uint16_t in2)
{
    HAL_GPIO_WritePin(in1_port, in1, command > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2, command < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (command < 0) {
        command = -command;
    }
    return (uint16_t)(command > 99 ? 99 : command);
}

void SetMotorSpeed(int16_t left_speed, int16_t right_speed)
{
    uint16_t left_pwm;
    uint16_t right_pwm;

    /* A/CH4 is left; B/CH1 is right. Motor signs compensate for wiring only. */
    left_pwm = SetDirection((int32_t)left_speed * CAR_LEFT_MOTOR_SIGN,
                            AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin);
    right_pwm = SetDirection((int32_t)right_speed * CAR_RIGHT_MOTOR_SIGN,
                             BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, left_pwm);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, right_pwm);
}
