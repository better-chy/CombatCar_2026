#ifndef __JY901S_H
#define __JY901S_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* JY901S 串口协议与 DMA 缓存 */
#define JY901S_FRAME_LEN        11U         /* JY901S 单帧固定长度。 */
#define JY901S_RX_BUF_LEN       11U         /* 串口 DMA 单次接收长度。 */
#define JY901S_PENDING_SLOTS    2U          /* ISR 到任务之间的待处理帧槽位数。 */
#define JY901S_DMA_ADDR         0x30000100U /* JY901S 串口 DMA 缓冲地址。 */

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
