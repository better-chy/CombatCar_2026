#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"

// -------------- 引脚配置 --------------
#define MOTOR1_PWM_TIM        htim1
#define MOTOR1_PWM_CHANNEL    TIM_CHANNEL_1

#define MOTOR2_PWM_TIM        htim1
#define MOTOR2_PWM_CHANNEL    TIM_CHANNEL_2

// 方向引脚
#define MOTOR1_DIR_GPIO_PORT  GPIOE
#define MOTOR1_DIR_PIN        GPIO_PIN_10

#define MOTOR2_DIR_GPIO_PORT  GPIOE
#define MOTOR2_DIR_PIN        GPIO_PIN_12

// -------------- 函数声明 --------------
void Motor_Init(void);
void Motor_SetSpeed(uint8_t motor, uint8_t dir, uint16_t speed);
void Motor_StopAll(void);
void Motor_SetSpeed_Signed(int speed);

void GoForward(int speed);
void GoBackward(int speed);
void TurnLeft(int speed1, int speed2);
void TurnRight(int speed1, int speed2);

#endif
