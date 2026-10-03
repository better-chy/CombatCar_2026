#ifndef __LASER_RANGE_H
#define __LASER_RANGE_H

#include "main.h"

/* 小激光硬件数量与 DMA 缓存 */
#define LASER_RANGE_SENSOR_COUNT      4U          /* 前、右、后、左共 4 个小激光测距。 */
#define LASER_RANGE_RX_BUF_LEN        23U         /* 小激光单帧协议长度。 */
#define LASER_RANGE_DMA_BUF_LEN       128U        /* 每路小激光 UART DMA 缓冲长度。 */
#define LASER_RANGE_DMA_BASE_ADDR     0x30000200U /* 四路小激光 DMA 缓冲起始地址。 */
#define LASER_RANGE_DMA_STRIDE        0x80U       /* 每路小激光 DMA 缓冲地址间隔。 */

/* 小激光救援模式 */
#define LASER_RANGE_UART_RESCUE_MODE  0U          /* 置 1 后启动波特率探测与恢复流程。 */
/* 四路救砖模式：
 * 0: 正常四路固定 115200 工作
 * 1: 启动时对 UART4/UART5/UART7/UART8 逐路扫描标准波特率，
 *    能识别到模块就尝试恢复到 115200，随后再进入正常 DMA 收数。 */

/* 小激光方向索引 */
#define LASER_FRONT 0U /* 前侧小激光。 */
#define LASER_RIGHT 1U /* 右侧小激光。 */
#define LASER_BACK  2U /* 后侧小激光。 */
#define LASER_LEFT  3U /* 左侧小激光。 */

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
