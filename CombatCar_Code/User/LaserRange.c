#include "LaserRange.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

#define LASER_RANGE_UART_TIMEOUT_MS       50U
#define LASER_RANGE_STARTUP_SETTLE_MS     3000U
#define LASER_RANGE_RETRY_INTERVAL_MS     500U
#define LASER_RANGE_STREAM_STALE_MS       1000U
/* 超过 LASER_RANGE_STREAM_STALE_MS 没有新帧，就认为该路断流。
 * 断流后每隔 LASER_RANGE_RETRY_INTERVAL_MS 补发一次 0x01 开流命令。 */

#if LASER_RANGE_UART_RESCUE_MODE
#define LASER_RANGE_PROBE_RX_BUF_LEN      64U
#define LASER_RANGE_RESCUE_ACCUM_BUF_LEN  128U
#define LASER_RANGE_RESCUE_TARGET_BAUD    115200UL
#define LASER_RANGE_RESCUE_FIRST_BYTE_MS  40U
#define LASER_RANGE_RESCUE_NEXT_BYTE_MS   5U
#define LASER_RANGE_RESCUE_CMD_GAP_MS     20U
#define LASER_RANGE_RESCUE_TRIES_PER_BAUD 2U
#endif

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t *rx_dma_buf;
    uint16_t dma_last_pos;
    uint8_t frame_buf[LASER_RANGE_RX_BUF_LEN];
    uint8_t frame_idx;
    volatile uint32_t seq_counter;
    volatile uint16_t distance_mm;
    volatile uint8_t valid;
    volatile uint32_t last_rx_tick;
    uint32_t last_start_tick;
} LaserRange_Context;

LaserRange_Context s_laser_ctx[LASER_RANGE_SENSOR_COUNT];
LaserRange_UartProbeResult g_laser_uart_probe[LASER_RANGE_SENSOR_COUNT];
LaserRange_DebugStats g_laser_debug[LASER_RANGE_SENSOR_COUNT];
volatile uint8_t g_laser_front_lt_50 = 0U;

static const uint8_t s_start_measuring_cmd[9] =
{
    0xA5U, 0x03U, 0x20U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
};

#if LASER_RANGE_UART_RESCUE_MODE
static const uint8_t s_set_baud_115200_cmd[10] =
{
    0xA5U, 0x03U, 0x20U, 0x10U, 0x00U, 0x01U, 0xC2U, 0x00U, 0x00U, 0x00U
};

static const uint32_t s_rescue_baud_table[] =
{
    115200UL,
    9600UL,
    921600UL
};

static uint8_t s_rescue_rx_buf[LASER_RANGE_PROBE_RX_BUF_LEN];
static uint8_t s_rescue_accum_buf[LASER_RANGE_RESCUE_ACCUM_BUF_LEN];
#endif

static uint16_t LaserRange_Crc16(const uint8_t *data, uint16_t length);
static void LaserRange_FillCrc(uint8_t *data, uint16_t length);
static LaserRange_Context *LaserRange_FindContext(UART_HandleTypeDef *huart);
static int32_t LaserRange_FindIndex(UART_HandleTypeDef *huart);
static void LaserRange_AssignContexts(void);
static void LaserRange_ResetParser(LaserRange_Context *ctx);
static void LaserRange_StartRx(LaserRange_Context *ctx);
static void LaserRange_StartFixed115200All(void);
static void LaserRange_SendStart(LaserRange_Context *ctx);
static void LaserRange_DelayMs(uint32_t delay_ms);
static void LaserRange_ProcessDmaPosition(LaserRange_Context *ctx);
static void LaserRange_OnRxBytes(LaserRange_Context *ctx, const uint8_t *data, uint16_t len);
static void LaserRange_HandleByte(LaserRange_Context *ctx, uint8_t byte);
static uint8_t LaserRange_TryParseFrame(const uint8_t *data, uint16_t len, uint16_t *distance_mm);
#if LASER_RANGE_UART_RESCUE_MODE
static void LaserRange_RunRescueAll(void);
static void LaserRange_RunRescueOnContext(uint8_t index);
static HAL_StatusTypeDef LaserRange_SetUartBaud(UART_HandleTypeDef *huart, uint32_t baud);
static uint16_t LaserRange_RecvBurst(UART_HandleTypeDef *huart,
                                     uint8_t *buf,
                                     uint16_t max_len,
                                     uint32_t first_byte_timeout_ms,
                                     uint32_t next_byte_timeout_ms);
static int32_t LaserRange_FindValidStreamFrame(const uint8_t *data, uint16_t len, uint16_t *distance_mm);
#endif

void LaserRange_Init(void)
{
    memset(s_laser_ctx, 0, sizeof(s_laser_ctx));
    memset(g_laser_uart_probe, 0, sizeof(g_laser_uart_probe));
    memset(g_laser_debug, 0, sizeof(g_laser_debug));
    g_laser_front_lt_50 = 0U;

    LaserRange_AssignContexts();

#if LASER_RANGE_UART_RESCUE_MODE
    /* 逐路扫波特率，能识别的模块尽量统一恢复到 115200。 */
    LaserRange_RunRescueAll();
#endif

    /* 正常工作仍然走四路固定 115200 流式接收。 */
    LaserRange_StartFixed115200All();
}

void LaserRange_ProcessPending(void)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < LASER_RANGE_SENSOR_COUNT; ++i)
    {
        LaserRange_ProcessDmaPosition(&s_laser_ctx[i]);

        if ((s_laser_ctx[i].valid == 0U || (now - s_laser_ctx[i].last_rx_tick) > LASER_RANGE_STREAM_STALE_MS) &&
            (now - s_laser_ctx[i].last_start_tick) >= LASER_RANGE_RETRY_INTERVAL_MS)
        {
            if ((now - s_laser_ctx[i].last_rx_tick) > LASER_RANGE_STREAM_STALE_MS)
            {
                s_laser_ctx[i].valid = 0U;
            }
            LaserRange_SendStart(&s_laser_ctx[i]);
        }
    }

    g_laser_front_lt_50 = (uint8_t)((s_laser_ctx[LASER_FRONT].valid != 0U &&
                                     s_laser_ctx[LASER_FRONT].distance_mm < 50U) ? 1U : 0U);
}

void LaserRange_HandleRxEventFromISR(UART_HandleTypeDef *huart, uint16_t size)
{
    (void)huart;
    (void)size;
}

void LaserRange_HandleError(UART_HandleTypeDef *huart)
{
    LaserRange_Context *ctx = LaserRange_FindContext(huart);
    int32_t index = LaserRange_FindIndex(huart);

    if (ctx == NULL)
    {
        return;
    }

    if (index >= 0)
    {
        ++g_laser_debug[index].uart_error_count;
        g_laser_debug[index].last_error_code = HAL_UART_GetError(huart);
    }

    ctx->valid = 0U;
    (void)HAL_UART_DMAStop(huart);
    LaserRange_StartRx(ctx);
}

uint16_t LaserRange_GetDistanceMm(uint8_t index)
{
    if (index >= LASER_RANGE_SENSOR_COUNT)
    {
        return 0U;
    }

    return s_laser_ctx[index].distance_mm;
}

void LaserRange_GetAllDistances(uint16_t *dest, uint16_t count)
{
    if (dest == NULL)
    {
        return;
    }

    if (count > LASER_RANGE_SENSOR_COUNT)
    {
        count = LASER_RANGE_SENSOR_COUNT;
    }

    for (uint16_t i = 0; i < count; ++i)
    {
        dest[i] = s_laser_ctx[i].distance_mm;
    }
}

static uint16_t LaserRange_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    if (data == NULL)
    {
        return 0U;
    }

    for (uint16_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8U; ++j)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static void LaserRange_FillCrc(uint8_t *data, uint16_t length)
{
    uint16_t crc;

    if (data == NULL || length < 2U)
    {
        return;
    }

    crc = LaserRange_Crc16(data, (uint16_t)(length - 2U));
    data[length - 2U] = (uint8_t)(crc & 0xFFU);
    data[length - 1U] = (uint8_t)((crc >> 8U) & 0xFFU);
}

static LaserRange_Context *LaserRange_FindContext(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < LASER_RANGE_SENSOR_COUNT; ++i)
    {
        if (s_laser_ctx[i].huart == huart)
        {
            return &s_laser_ctx[i];
        }
    }

    return NULL;
}

static int32_t LaserRange_FindIndex(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < LASER_RANGE_SENSOR_COUNT; ++i)
    {
        if (s_laser_ctx[i].huart == huart)
        {
            return (int32_t)i;
        }
    }

    return -1;
}

static void LaserRange_AssignContexts(void)
{
    s_laser_ctx[0].huart = &huart4;
    s_laser_ctx[1].huart = &huart5;
    s_laser_ctx[2].huart = &huart7;
    s_laser_ctx[3].huart = &huart8;

    s_laser_ctx[0].rx_dma_buf = (uint8_t *)(LASER_RANGE_DMA_BASE_ADDR + 0U * LASER_RANGE_DMA_STRIDE);
    s_laser_ctx[1].rx_dma_buf = (uint8_t *)(LASER_RANGE_DMA_BASE_ADDR + 1U * LASER_RANGE_DMA_STRIDE);
    s_laser_ctx[2].rx_dma_buf = (uint8_t *)(LASER_RANGE_DMA_BASE_ADDR + 2U * LASER_RANGE_DMA_STRIDE);
    s_laser_ctx[3].rx_dma_buf = (uint8_t *)(LASER_RANGE_DMA_BASE_ADDR + 3U * LASER_RANGE_DMA_STRIDE);

    g_laser_uart_probe[0].huart = &huart4;
    g_laser_uart_probe[1].huart = &huart5;
    g_laser_uart_probe[2].huart = &huart7;
    g_laser_uart_probe[3].huart = &huart8;
}

static void LaserRange_ResetParser(LaserRange_Context *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    ctx->dma_last_pos = 0U;
    ctx->frame_idx = 0U;
    memset(ctx->frame_buf, 0, sizeof(ctx->frame_buf));
}

static void LaserRange_StartRx(LaserRange_Context *ctx)
{
    HAL_StatusTypeDef status;
    int32_t index;

    if (ctx == NULL || ctx->huart == NULL)
    {
        return;
    }

    index = LaserRange_FindIndex(ctx->huart);
    LaserRange_ResetParser(ctx);
    memset(ctx->rx_dma_buf, 0, LASER_RANGE_DMA_BUF_LEN);

    status = HAL_UART_Receive_DMA(ctx->huart, ctx->rx_dma_buf, LASER_RANGE_DMA_BUF_LEN);
    if (index >= 0)
    {
        g_laser_debug[index].last_start_rx_status = status;
        if (status != HAL_OK)
        {
            ++g_laser_debug[index].start_rx_fail_count;
        }
    }
    if (status == HAL_OK && ctx->huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(ctx->huart->hdmarx, DMA_IT_HT);
        __HAL_DMA_DISABLE_IT(ctx->huart->hdmarx, DMA_IT_TC);
        __HAL_UART_CLEAR_IDLEFLAG(ctx->huart);
        __HAL_UART_ENABLE_IT(ctx->huart, UART_IT_IDLE);
    }
}

static void LaserRange_StartFixed115200All(void)
{
    for (uint8_t i = 0; i < LASER_RANGE_SENSOR_COUNT; ++i)
    {
        LaserRange_StartRx(&s_laser_ctx[i]);
    }

    /* 厂家示例在开流前等待约 3 秒，这里先按同样节奏做基线验证。 */
    LaserRange_DelayMs(LASER_RANGE_STARTUP_SETTLE_MS);

    for (uint8_t i = 0; i < LASER_RANGE_SENSOR_COUNT; ++i)
    {
        LaserRange_SendStart(&s_laser_ctx[i]);
    }
}

static void LaserRange_SendStart(LaserRange_Context *ctx)
{
    uint8_t start_cmd[sizeof(s_start_measuring_cmd)];

    if (ctx == NULL || ctx->huart == NULL)
    {
        return;
    }

    memcpy(start_cmd, s_start_measuring_cmd, sizeof(start_cmd));
    LaserRange_FillCrc(start_cmd, (uint16_t)sizeof(start_cmd));
    (void)HAL_UART_Transmit(ctx->huart, start_cmd, sizeof(start_cmd), LASER_RANGE_UART_TIMEOUT_MS);
    ctx->last_start_tick = HAL_GetTick();
}

static void LaserRange_DelayMs(uint32_t delay_ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    else
    {
        HAL_Delay(delay_ms);
    }
}

static void LaserRange_ProcessDmaPosition(LaserRange_Context *ctx)
{
    uint16_t pos;

    if (ctx == NULL || ctx->huart == NULL || ctx->huart->hdmarx == NULL || ctx->rx_dma_buf == NULL)
    {
        return;
    }

    pos = (uint16_t)(LASER_RANGE_DMA_BUF_LEN - __HAL_DMA_GET_COUNTER(ctx->huart->hdmarx));
    if (pos > LASER_RANGE_DMA_BUF_LEN)
    {
        pos = (uint16_t)(pos % LASER_RANGE_DMA_BUF_LEN);
    }

    if (pos == ctx->dma_last_pos)
    {
        return;
    }

    if (pos > ctx->dma_last_pos)
    {
        LaserRange_OnRxBytes(ctx, &ctx->rx_dma_buf[ctx->dma_last_pos], (uint16_t)(pos - ctx->dma_last_pos));
    }
    else
    {
        LaserRange_OnRxBytes(ctx,
                             &ctx->rx_dma_buf[ctx->dma_last_pos],
                             (uint16_t)(LASER_RANGE_DMA_BUF_LEN - ctx->dma_last_pos));
        if (pos > 0U)
        {
            LaserRange_OnRxBytes(ctx, &ctx->rx_dma_buf[0], pos);
        }
    }

    ctx->dma_last_pos = pos;
}

static void LaserRange_OnRxBytes(LaserRange_Context *ctx, const uint8_t *data, uint16_t len)
{
    if (ctx == NULL || data == NULL || len == 0U)
    {
        return;
    }

    for (uint16_t i = 0; i < len; ++i)
    {
        LaserRange_HandleByte(ctx, data[i]);
    }
}

static void LaserRange_HandleByte(LaserRange_Context *ctx, uint8_t byte)
{
    uint16_t distance_mm;
    int32_t index;

    if (ctx == NULL)
    {
        return;
    }

    switch (ctx->frame_idx)
    {
    case 0U:
        if (byte == 0xA5U)
        {
            ctx->frame_buf[0] = byte;
            ctx->frame_idx = 1U;
        }
        return;

    case 1U:
        if (byte == 0x03U)
        {
            ctx->frame_buf[1] = byte;
            ctx->frame_idx = 2U;
        }
        else if (byte == 0xA5U)
        {
            ctx->frame_buf[0] = byte;
            ctx->frame_idx = 1U;
        }
        else
        {
            ctx->frame_idx = 0U;
        }
        return;

    case 2U:
        if (byte == 0x20U)
        {
            ctx->frame_buf[2] = byte;
            ctx->frame_idx = 3U;
        }
        else if (byte == 0xA5U)
        {
            ctx->frame_buf[0] = byte;
            ctx->frame_idx = 1U;
        }
        else
        {
            ctx->frame_idx = 0U;
        }
        return;

    case 3U:
        if (byte == 0x01U)
        {
            ctx->frame_buf[3] = byte;
            ctx->frame_idx = 4U;
        }
        else if (byte == 0xA5U)
        {
            ctx->frame_buf[0] = byte;
            ctx->frame_idx = 1U;
        }
        else
        {
            ctx->frame_idx = 0U;
        }
        return;

    default:
        ctx->frame_buf[ctx->frame_idx++] = byte;
        if (ctx->frame_idx < LASER_RANGE_RX_BUF_LEN)
        {
            return;
        }
        break;
    }

    if (LaserRange_TryParseFrame(ctx->frame_buf, LASER_RANGE_RX_BUF_LEN, &distance_mm) != 0U)
    {
        ctx->distance_mm = distance_mm;
        ctx->valid = 1U;
        ctx->last_rx_tick = HAL_GetTick();
        ++ctx->seq_counter;

        index = LaserRange_FindIndex(ctx->huart);
        if (index >= 0)
        {
            ++g_laser_debug[index].rx_done_count;
            g_laser_debug[index].last_rx_size = LASER_RANGE_RX_BUF_LEN;
        }
    }

    ctx->frame_idx = 0U;
}

static uint8_t LaserRange_TryParseFrame(const uint8_t *data, uint16_t len, uint16_t *distance_mm)
{
    uint16_t crc_calc;
    uint16_t crc_frame_lo_hi;
    uint16_t crc_frame_hi_lo;

    if (data == NULL || distance_mm == NULL || len != LASER_RANGE_RX_BUF_LEN)
    {
        return 0U;
    }

    if (data[0] != 0xA5U || data[1] != 0x03U || data[2] != 0x20U || data[3] != 0x01U)
    {
        return 0U;
    }

    if (data[4] != 0x00U || data[5] != 0x00U || data[6] != 0x0EU)
    {
        return 0U;
    }

    crc_calc = LaserRange_Crc16(data, LASER_RANGE_RX_BUF_LEN - 2U);
    crc_frame_lo_hi = (uint16_t)data[LASER_RANGE_RX_BUF_LEN - 2U]
                    | ((uint16_t)data[LASER_RANGE_RX_BUF_LEN - 1U] << 8U);
    crc_frame_hi_lo = ((uint16_t)data[LASER_RANGE_RX_BUF_LEN - 2U] << 8U)
                    | (uint16_t)data[LASER_RANGE_RX_BUF_LEN - 1U];

    if (crc_calc != crc_frame_lo_hi && crc_calc != crc_frame_hi_lo)
    {
        return 0U;
    }

    *distance_mm = (uint16_t)data[13U] | ((uint16_t)data[14U] << 8U);
    return 1U;
}

#if LASER_RANGE_UART_RESCUE_MODE
static void LaserRange_RunRescueAll(void)
{
    for (uint8_t i = 0; i < LASER_RANGE_SENSOR_COUNT; ++i)
    {
        LaserRange_RunRescueOnContext(i);
    }
}

static void LaserRange_RunRescueOnContext(uint8_t index)
{
    LaserRange_Context *ctx;
    LaserRange_UartProbeResult *probe;
    uint8_t start_cmd[sizeof(s_start_measuring_cmd)];
    uint8_t set_baud_cmd[sizeof(s_set_baud_115200_cmd)];
    uint16_t accum_len;
    uint16_t rx_len;
    uint16_t distance_mm;

    if (index >= LASER_RANGE_SENSOR_COUNT)
    {
        return;
    }

    ctx = &s_laser_ctx[index];
    probe = &g_laser_uart_probe[index];

    if (ctx->huart == NULL)
    {
        return;
    }

    memcpy(start_cmd, s_start_measuring_cmd, sizeof(start_cmd));
    memcpy(set_baud_cmd, s_set_baud_115200_cmd, sizeof(set_baud_cmd));
    LaserRange_FillCrc(start_cmd, (uint16_t)sizeof(start_cmd));
    LaserRange_FillCrc(set_baud_cmd, (uint16_t)sizeof(set_baud_cmd));

    memset(probe, 0, sizeof(*probe));
    probe->huart = ctx->huart;

    for (uint32_t i = 0; i < (sizeof(s_rescue_baud_table) / sizeof(s_rescue_baud_table[0])); ++i)
    {
        probe->matched_baud = s_rescue_baud_table[i];
        probe->last_status = LaserRange_SetUartBaud(ctx->huart, s_rescue_baud_table[i]);
        probe->last_hal_error = (uint8_t)HAL_UART_GetError(ctx->huart);
        if (probe->last_status != HAL_OK)
        {
            continue;
        }

        accum_len = 0U;
        memset(s_rescue_accum_buf, 0, sizeof(s_rescue_accum_buf));

        for (uint32_t attempt = 0; attempt < LASER_RANGE_RESCUE_TRIES_PER_BAUD; ++attempt)
        {
            LaserRange_DelayMs(2U);
            (void)HAL_UART_Transmit(ctx->huart, start_cmd, sizeof(start_cmd), LASER_RANGE_UART_TIMEOUT_MS);
            LaserRange_DelayMs(LASER_RANGE_RESCUE_CMD_GAP_MS);

            rx_len = LaserRange_RecvBurst(ctx->huart,
                                          s_rescue_rx_buf,
                                          (uint16_t)sizeof(s_rescue_rx_buf),
                                          LASER_RANGE_RESCUE_FIRST_BYTE_MS,
                                          LASER_RANGE_RESCUE_NEXT_BYTE_MS);

            probe->last_rx_len = rx_len;
            memset(probe->last_rx_raw, 0, sizeof(probe->last_rx_raw));
            if (rx_len > 0U)
            {
                uint16_t copy_len = rx_len;
                if (copy_len > (uint16_t)sizeof(probe->last_rx_raw))
                {
                    copy_len = (uint16_t)sizeof(probe->last_rx_raw);
                }
                memcpy(probe->last_rx_raw, s_rescue_rx_buf, copy_len);

                if ((uint16_t)(accum_len + rx_len) > (uint16_t)sizeof(s_rescue_accum_buf))
                {
                    accum_len = 0U;
                }

                memcpy(&s_rescue_accum_buf[accum_len], s_rescue_rx_buf, rx_len);
                accum_len = (uint16_t)(accum_len + rx_len);
            }

            if (LaserRange_FindValidStreamFrame(s_rescue_accum_buf, accum_len, &distance_mm) >= 0)
            {
                break;
            }
        }

        if (LaserRange_FindValidStreamFrame(s_rescue_accum_buf, accum_len, &distance_mm) < 0)
        {
            continue;
        }

        probe->found = 1U;
        probe->matched_cmd = 0x01U;
        ctx->distance_mm = distance_mm;
        ctx->valid = 1U;
        ctx->last_rx_tick = HAL_GetTick();
        ++ctx->seq_counter;

        if (probe->matched_baud != LASER_RANGE_RESCUE_TARGET_BAUD)
        {
            (void)HAL_UART_Transmit(ctx->huart, set_baud_cmd, sizeof(set_baud_cmd), LASER_RANGE_UART_TIMEOUT_MS);
            LaserRange_DelayMs(LASER_RANGE_RESCUE_CMD_GAP_MS);

            probe->last_status = LaserRange_SetUartBaud(ctx->huart, LASER_RANGE_RESCUE_TARGET_BAUD);
            probe->last_hal_error = (uint8_t)HAL_UART_GetError(ctx->huart);
            if (probe->last_status == HAL_OK)
            {
                probe->matched_baud = LASER_RANGE_RESCUE_TARGET_BAUD;
            }
        }
        else
        {
            (void)LaserRange_SetUartBaud(ctx->huart, LASER_RANGE_RESCUE_TARGET_BAUD);
        }

        return;
    }

    probe->last_status = LaserRange_SetUartBaud(ctx->huart, LASER_RANGE_RESCUE_TARGET_BAUD);
    probe->last_hal_error = (uint8_t)HAL_UART_GetError(ctx->huart);
    probe->matched_baud = LASER_RANGE_RESCUE_TARGET_BAUD;
}

static HAL_StatusTypeDef LaserRange_SetUartBaud(UART_HandleTypeDef *huart, uint32_t baud)
{
    HAL_StatusTypeDef status;

    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    (void)HAL_UART_DMAStop(huart);
    (void)HAL_UART_Abort(huart);
    (void)HAL_UART_DeInit(huart);

    huart->Init.BaudRate = baud;
    status = HAL_UART_Init(huart);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_UARTEx_SetTxFifoThreshold(huart, UART_TXFIFO_THRESHOLD_1_8);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_UARTEx_SetRxFifoThreshold(huart, UART_RXFIFO_THRESHOLD_1_8);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_UARTEx_DisableFifoMode(huart);
}

static uint16_t LaserRange_RecvBurst(UART_HandleTypeDef *huart,
                                     uint8_t *buf,
                                     uint16_t max_len,
                                     uint32_t first_byte_timeout_ms,
                                     uint32_t next_byte_timeout_ms)
{
    uint16_t len = 0U;

    if (huart == NULL || buf == NULL || max_len == 0U)
    {
        return 0U;
    }

    if (HAL_UART_Receive(huart, &buf[len], 1U, first_byte_timeout_ms) != HAL_OK)
    {
        return 0U;
    }

    ++len;
    while (len < max_len)
    {
        if (HAL_UART_Receive(huart, &buf[len], 1U, next_byte_timeout_ms) != HAL_OK)
        {
            break;
        }
        ++len;
    }

    return len;
}

static int32_t LaserRange_FindValidStreamFrame(const uint8_t *data, uint16_t len, uint16_t *distance_mm)
{
    if (data == NULL || distance_mm == NULL || len < LASER_RANGE_RX_BUF_LEN)
    {
        return -1;
    }

    for (uint16_t offset = 0U; (uint16_t)(offset + LASER_RANGE_RX_BUF_LEN) <= len; ++offset)
    {
        if (LaserRange_TryParseFrame(&data[offset], LASER_RANGE_RX_BUF_LEN, distance_mm) != 0U)
        {
            return (int32_t)offset;
        }
    }

    return -1;
}
#endif
