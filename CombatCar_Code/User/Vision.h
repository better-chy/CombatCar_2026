#ifndef __VISION_H
#define __VISION_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* 视觉串口 DMA 与待处理缓存 */
#define VISION_RX_BUF_LEN         8U          /* 视觉上位机单帧接收长度。 */
#define VISION_PENDING_SLOTS      2U          /* ISR 到任务之间的待处理帧槽位数。 */
#define VISION_DMA_ADDR           0x30000120U /* 视觉串口 DMA 缓冲地址。 */

/* 视觉串口协议 */
#define VISION_SOF1              0xA5U        /* 视觉数据包帧头第 1 字节。 */
#define VISION_SOF2              0x5AU        /* 视觉数据包帧头第 2 字节。 */
#define VISION_MSG_TARGET        0x01U        /* AprilTag 目标信息消息类型。 */
#define VISION_PAYLOAD_LEN        4U          /* 视觉目标消息 payload 长度。 */
#define VISION_TAG_ID_NONE       0xFFU        /* 没有有效 AprilTag 时使用的 ID。 */

/* 视觉目标状态标志 */
#define VISION_FLAG_DETECTED     0x01U        /* 当前帧检测到 AprilTag。 */
#define VISION_FLAG_ID_VALID     0x02U        /* 当前帧 AprilTag ID 有效。 */

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
