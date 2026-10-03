#ifndef __MOTOR_H
#define __MOTOR_H

#include "CombatCar_TuneConfig.h"
#include "main.h"
#include "tim.h"

extern volatile uint8_t g_debug_motor_kill;

void Motor_Init(void);
void Motor_SetSpeeds(int left_speed, int right_speed);
void Motor_StopAll(void);

void Run_Forward(int speed);
void Run_Back(int speed);
void Turn_Left(int left_speed, int right_speed);
void Turn_Right(int left_speed, int right_speed);

#endif
