#include "motor.h"

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&MOTOR1_PWM_TIM, MOTOR1_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&MOTOR2_PWM_TIM, MOTOR2_PWM_CHANNEL);

    Motor_StopAll();
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

void Motor_StopAll(void) // 停止所有电机
{
    Motor_SetSpeed(1, 0, 0);
    Motor_SetSpeed(2, 0, 0);
}

void Motor_SetSpeed_Signed(int speed)
{
    uint8_t dir;
    uint16_t pwm;

    if(speed >= 0)
    {
        dir = GPIO_PIN_SET;    // 正数 = 前进
        pwm = speed;
    }
    else
    {
        dir = GPIO_PIN_RESET;  // 负数 = 后退
        pwm = -speed;
    }

    // PWM 限制到 90%
    if(pwm > 899) pwm = 899;  

    Motor_SetSpeed(1, dir, pwm);
    Motor_SetSpeed(2, dir, pwm);
}

// 1. 前进
void GoForward(int speed)
{
    Motor_StopAll();
    HAL_Delay(50);  // 停 50ms
    Motor_SetSpeed_Signed(speed);
}

// 2. 后退
void GoBackward(int speed)
{
    Motor_StopAll();
    HAL_Delay(50);
    Motor_SetSpeed_Signed(-speed);
}

// 3. 左转
void TurnLeft(int speed1, int speed2)
{
    Motor_StopAll();
    HAL_Delay(50);

    // 电机1
    uint8_t dir1 = (speed1 >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    uint16_t pwm1 = (speed1 >= 0) ? speed1 : -speed1;
    if(pwm1 > 899) pwm1 = 899;

    // 电机2
    uint8_t dir2 = (speed2 >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    uint16_t pwm2 = (speed2 >= 0) ? speed2 : -speed2;
    if(pwm2 > 899) pwm2 = 899;

    Motor_SetSpeed(1, dir1, pwm1);
    Motor_SetSpeed(2, dir2, pwm2);
}

// 4. 右转
void TurnRight(int speed1, int speed2)
{
    Motor_StopAll();
    HAL_Delay(50);

    // 电机1
    uint8_t dir1 = (speed1 >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    uint16_t pwm1 = (speed1 >= 0) ? speed1 : -speed1;
    if(pwm1 > 899) pwm1 = 899;

    // 电机2
    uint8_t dir2 = (speed2 >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    uint16_t pwm2 = (speed2 >= 0) ? speed2 : -speed2;
    if(pwm2 > 899) pwm2 = 899;

    Motor_SetSpeed(1, dir1, pwm1);
    Motor_SetSpeed(2, dir2, pwm2);
}
