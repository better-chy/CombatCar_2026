#ifndef __LASER_RANGE_H
#define __LASER_RANGE_H

#include "main.h"

#define LASER_RANGE_SENSOR_COUNT 4U
#define LASER_RANGE_RX_BUF_LEN   23U
#define LASER_RANGE_DMA_BASE_ADDR 0x30000200U
#define LASER_RANGE_DMA_STRIDE    0x20U

void LaserRange_Init(void);
void LaserRange_ProcessPending(void);

void LaserRange_HandleRxEventFromISR(UART_HandleTypeDef *huart, uint16_t size);
void LaserRange_HandleError(UART_HandleTypeDef *huart);

uint16_t LaserRange_GetDistanceMm(uint8_t index);
void LaserRange_GetAllDistances(uint16_t *dest, uint16_t count);

#endif
