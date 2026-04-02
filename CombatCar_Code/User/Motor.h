#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"


void Motor_Init(void);
void Motor_SetSpeed(int id, int speed);
void Motor_StopAll(void);

void Run_Forward(int speed);
void Run_Back(int speed);
void Turn_Left(int left_speed, int right_speed);
void Turn_Right(int left_speed, int right_speed);

#endif
