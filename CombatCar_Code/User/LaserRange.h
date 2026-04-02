#ifndef __LASER_RANGE_H
#define __LASER_RANGE_H

#include "main.h"

#define LASER_RANGE_SENSOR_COUNT 4U
#define LASER_RANGE_RX_BUF_LEN   23U
#define LASER_RANGE_DMA_BUF_LEN  128U
#define LASER_RANGE_DMA_BASE_ADDR 0x30000200U
#define LASER_RANGE_DMA_STRIDE    0x80U

#define LASER_RANGE_UART_RESCUE_MODE 1U
/* 四路救砖模式：
 * 0: 正常四路固定 115200 工作
 * 1: 启动时对 UART4/UART5/UART7/UART8 逐路扫描标准波特率，
 *    能识别到模块就尝试恢复到 115200，随后再进入正常 DMA 收数。 */

#define LASER_FRONT 0U
#define LASER_RIGHT 1U
#define LASER_BACK 2U
#define LASER_LEFT 3U

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t found;
    uint8_t matched_cmd;
    uint8_t last_hal_error;
    HAL_StatusTypeDef last_status;
    uint32_t matched_baud;
    uint16_t last_rx_len;
    uint8_t last_rx_raw[64];
} LaserRange_UartProbeResult;

extern LaserRange_UartProbeResult g_laser_uart_probe[LASER_RANGE_SENSOR_COUNT];

typedef struct
{
    uint32_t rx_done_count;
    uint32_t uart_error_count;
    uint32_t start_rx_fail_count;
    uint32_t last_error_code;
    uint16_t last_rx_size;
    HAL_StatusTypeDef last_start_rx_status;
} LaserRange_DebugStats;

extern LaserRange_DebugStats g_laser_debug[LASER_RANGE_SENSOR_COUNT];
extern volatile uint8_t g_laser_front_lt_50;

void LaserRange_Init(void);
void LaserRange_ProcessPending(void);

void LaserRange_HandleRxEventFromISR(UART_HandleTypeDef *huart, uint16_t size);
void LaserRange_HandleError(UART_HandleTypeDef *huart);

uint16_t LaserRange_GetDistanceMm(uint8_t index);
void LaserRange_GetAllDistances(uint16_t *dest, uint16_t count);

#endif
