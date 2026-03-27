#ifndef __JY901S_H
#define __JY901S_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define JY901S_FRAME_LEN        11U
#define JY901S_RX_BUF_LEN       11U
#define JY901S_PENDING_SLOTS    2U
#define JY901S_DMA_ADDR         0x30000100U

typedef struct
{
    float acc[3];
    float gyro[3];
    float angle[3];
    uint8_t valid_mask;
    uint32_t update_tick;
} JY901S_Data;

void JY901S_Init(UART_HandleTypeDef *huart);
void JY901S_ProcessPending(void);
void JY901S_HandleRxEventFromISR(UART_HandleTypeDef *huart, uint16_t size);
void JY901S_HandleError(UART_HandleTypeDef *huart);

float JY901S_GetYawDeg(void);
float JY901S_GetPitchDeg(void);
float JY901S_GetRollDeg(void);
uint8_t JY901S_GetData(JY901S_Data *out);
uint8_t JY901S_IsOnline(uint32_t timeout_ms);

#endif
