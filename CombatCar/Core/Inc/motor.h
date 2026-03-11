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
void Motor_Stop(uint8_t motor);
void Motor_Forward(uint8_t motor, uint16_t speed);
void Motor_Backward(uint8_t motor, uint16_t speed);

void Motor_CarForward(uint16_t speed);
void Motor_CarBackward(uint16_t speed);
void Motor_CarLeft(uint16_t speed);
void Motor_CarRight(uint16_t speed);

#endif