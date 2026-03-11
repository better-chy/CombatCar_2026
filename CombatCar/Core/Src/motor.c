#include "motor.h"

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&MOTOR1_PWM_TIM, MOTOR1_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&MOTOR2_PWM_TIM, MOTOR2_PWM_CHANNEL);

    Motor_Stop(1);
    Motor_Stop(2);
}

void Motor_SetSpeed(uint8_t motor, uint8_t dir, uint16_t speed)
{
    if(speed > 999) speed = 999;

    if(motor == 1)
    {
        HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_PORT, MOTOR1_DIR_PIN, dir);
        __HAL_TIM_SET_COMPARE(&MOTOR1_PWM_TIM, MOTOR1_PWM_CHANNEL, speed);
    }
    else if(motor == 2)
    {
        HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_PORT, MOTOR2_DIR_PIN, dir);
        __HAL_TIM_SET_COMPARE(&MOTOR2_PWM_TIM, MOTOR2_PWM_CHANNEL, speed);
    }
}

void Motor_Stop(uint8_t motor)
{
    Motor_SetSpeed(motor, 0, 0);
}

void Motor_Forward(uint8_t motor, uint16_t speed)
{
    Motor_SetSpeed(motor, 0, speed);
}

void Motor_Backward(uint8_t motor, uint16_t speed)
{
    Motor_SetSpeed(motor, 1, speed);
}

void Motor_CarForward(uint16_t speed)
{
    Motor_Forward(1, speed);
    Motor_Forward(2, speed);
}

void Motor_CarBackward(uint16_t speed)
{
    Motor_Backward(1, speed);
    Motor_Backward(2, speed);
}

void Motor_CarLeft(uint16_t speed)
{
    Motor_Forward(1, speed/2);
    Motor_Forward(2, speed);
}

void Motor_CarRight(uint16_t speed)
{
    Motor_Forward(1, speed);
    Motor_Forward(2, speed/2);
}