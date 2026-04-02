#ifndef __VISION_H
#define __VISION_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define VISION_RX_BUF_LEN         8U
#define VISION_PENDING_SLOTS      2U
#define VISION_DMA_ADDR   0x30000120U

#define VISION_SOF1              0xA5U
#define VISION_SOF2              0x5AU
#define VISION_MSG_TARGET        0x01U
#define VISION_PAYLOAD_LEN        4U
#define VISION_TAG_ID_NONE       0xFFU

#define VISION_FLAG_DETECTED     0x01U
#define VISION_FLAG_ID_VALID     0x02U

typedef struct
{
    uint8_t seq;
    uint8_t flags;
    uint8_t tag_id;
    uint32_t update_tick;
} Vision_Tag;

void Vision_Init(UART_HandleTypeDef *huart);
void Vision_StartRx(void);
void Vision_ProcessPending(void);
void Vision_HandleRxEventFromISR(UART_HandleTypeDef *huart, uint16_t size);
void Vision_HandleError(UART_HandleTypeDef *huart);

uint8_t Vision_GetLatestTag(Vision_Tag *out);
uint8_t Vision_IsOnline(uint32_t timeout_ms);

#endif
