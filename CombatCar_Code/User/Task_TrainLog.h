#ifndef __TASK_TRAIN_LOG_H
#define __TASK_TRAIN_LOG_H

#include "stm32h7xx_hal.h"

void Task_TrainLog_Run(void *argument);
void TaskTrainLog_HandleRxEventFromISR(UART_HandleTypeDef *huart, uint16_t size);
void TaskTrainLog_HandleError(UART_HandleTypeDef *huart);
void TaskTrainLog_NotifyEnemyHit(void);
void TaskTrainLog_NotifyEnemyFound(void);

#endif
