#include "JY901S.h"

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

typedef enum
{
    JY901S_BUF_FREE = 0U,
    JY901S_BUF_READY,
    JY901S_BUF_READING
} JY901S_BufferState;
//双缓冲状态

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t *rx_dma_buf;
    volatile uint8_t pending_buf[JY901S_PENDING_SLOTS][JY901S_RX_BUF_LEN];
    volatile uint16_t pending_len[JY901S_PENDING_SLOTS];
    volatile uint8_t pending_state[JY901S_PENDING_SLOTS];
    volatile uint32_t pending_seq[JY901S_PENDING_SLOTS];   //时间顺序编号
    volatile uint32_t seq_counter;
    uint8_t frame[JY901S_FRAME_LEN];
    uint8_t frame_idx;
    JY901S_Data latest;
    volatile uint8_t has_data;
} JY901S_Context;

JY901S_Context s_jy901;

static void JY901S_StartRx(JY901S_Context *ctx);
static int32_t JY901S_GetFreeSlot(const JY901S_Context *ctx);
static int32_t JY901S_GetOldestReadySlot(const JY901S_Context *ctx);
static int32_t JY901S_ClaimReadySlot(JY901S_Context *ctx, uint16_t *len_out);
static void JY901S_ReleaseReadingSlot(JY901S_Context *ctx, uint8_t slot);
static void JY901S_OnBytes(const uint8_t *data, uint16_t len);
static void JY901S_HandleByte(uint8_t byte);
static uint8_t JY901S_CheckFrame(const uint8_t *frame);
static void JY901S_ParseFrame(const uint8_t *frame);

void JY901S_Init(UART_HandleTypeDef *huart)
{
    memset(&s_jy901, 0, sizeof(s_jy901));    //状态清零
    s_jy901.huart = huart;
    s_jy901.rx_dma_buf = (uint8_t *)JY901S_DMA_ADDR;   //修改dma内存地址
    JY901S_StartRx(&s_jy901);
}

void JY901S_ProcessPending(void)
{
    int32_t slot;
    uint16_t slot_len;

    while (1)
    {
        slot = JY901S_ClaimReadySlot(&s_jy901, &slot_len);
        if (slot < 0)
        {
            break;
        }

        JY901S_OnBytes((const uint8_t *)s_jy901.pending_buf[slot], slot_len);
        JY901S_ReleaseReadingSlot(&s_jy901, (uint8_t)slot);
    }
}

void JY901S_HandleRxEventFromISR(UART_HandleTypeDef *huart, uint16_t size)
{
    int32_t slot;
    UBaseType_t uxSavedInterruptStatus;

    //确认是uart1的串口
    if (huart != s_jy901.huart || s_jy901.huart == NULL)
    {
        return;
    }

    //防止越界
    if (size > JY901S_RX_BUF_LEN)
    {
        size = JY901S_RX_BUF_LEN;
    }

    if (size == 0U)
    {
        return;
    }

    //双缓冲，先找空闲槽，如果没有，覆盖最旧的ready槽
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    slot = JY901S_GetFreeSlot(&s_jy901);
    if (slot < 0)
    {
        slot = JY901S_GetOldestReadySlot(&s_jy901);
    }

    //把dma数据搬到pendingbuf
    if (slot >= 0)
    {
        memcpy((void *)s_jy901.pending_buf[slot], s_jy901.rx_dma_buf, size);
        s_jy901.pending_len[slot] = size;
        s_jy901.pending_seq[slot] = ++s_jy901.seq_counter;
        s_jy901.pending_state[slot] = JY901S_BUF_READY;
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void JY901S_HandleError(UART_HandleTypeDef *huart)
{
    if (huart != s_jy901.huart || s_jy901.huart == NULL)
    {
        return;
    }

    (void)HAL_UART_DMAStop(huart);
    JY901S_StartRx(&s_jy901);
}

float JY901S_GetYawDeg(void)
{
    return s_jy901.latest.angle[2];
}

float JY901S_GetPitchDeg(void)
{
    return s_jy901.latest.angle[1];
}

float JY901S_GetRollDeg(void)
{
    return s_jy901.latest.angle[0];
}

uint8_t JY901S_GetData(JY901S_Data *out)
{
    if (out == NULL || s_jy901.has_data == 0U)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    *out = s_jy901.latest;
    taskEXIT_CRITICAL();
    return 1U;
}

uint8_t JY901S_IsOnline(uint32_t timeout_ms)
{
    if (s_jy901.has_data == 0U)
    {
        return 0U;
    }

    return (uint8_t)((xTaskGetTickCount() - s_jy901.latest.update_tick) <= pdMS_TO_TICKS(timeout_ms));
}

static void JY901S_StartRx(JY901S_Context *ctx)
{
    HAL_StatusTypeDef status;

    if (ctx == NULL || ctx->huart == NULL)
    {
        return;
    }

    status = HAL_UART_Receive_DMA(ctx->huart, ctx->rx_dma_buf, JY901S_RX_BUF_LEN);

    /* USART1_RX is configured as DMA_CIRCULAR, so DMA should be started once
       and only restarted from the explicit error recovery path. */
    if (status == HAL_OK && ctx->huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(ctx->huart->hdmarx, DMA_IT_HT);
    }
}

static int32_t JY901S_GetFreeSlot(const JY901S_Context *ctx)
{
    uint32_t i;

    for (i = 0U; i < JY901S_PENDING_SLOTS; ++i)
    {
        if (ctx->pending_state[i] == JY901S_BUF_FREE)
        {
            return (int32_t)i;
        }
    }

    return -1;
}

static int32_t JY901S_GetOldestReadySlot(const JY901S_Context *ctx)
{
    uint32_t i;
    int32_t oldest = -1;
    uint32_t oldest_seq = 0U;

    for (i = 0U; i < JY901S_PENDING_SLOTS; ++i)
    {
        if (ctx->pending_state[i] != JY901S_BUF_READY)
        {
            continue;
        }

        if (oldest < 0 || ctx->pending_seq[i] < oldest_seq)
        {
            oldest = (int32_t)i;
            oldest_seq = ctx->pending_seq[i];
        }
    }

    return oldest;
}

static int32_t JY901S_ClaimReadySlot(JY901S_Context *ctx, uint16_t *len_out)
{
    int32_t slot;

    taskENTER_CRITICAL();
    slot = JY901S_GetOldestReadySlot(ctx);
    if (slot >= 0)
    {
        *len_out = ctx->pending_len[slot];
        ctx->pending_state[slot] = JY901S_BUF_READING;
    }
    taskEXIT_CRITICAL();

    return slot;
}

static void JY901S_ReleaseReadingSlot(JY901S_Context *ctx, uint8_t slot)
{
    taskENTER_CRITICAL();
    ctx->pending_state[slot] = JY901S_BUF_FREE;
    ctx->pending_len[slot] = 0U;
    ctx->pending_seq[slot] = 0U;
    taskEXIT_CRITICAL();
}

static void JY901S_OnBytes(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    for (i = 0U; i < len; ++i)
    {
        JY901S_HandleByte(data[i]);
    }
}

static void JY901S_HandleByte(uint8_t byte)
{
    if (s_jy901.frame_idx == 0U)
    {
        if (byte == 0x55U)
        {
            s_jy901.frame[0] = byte;
            s_jy901.frame_idx = 1U;
        }
        return;
    }

    s_jy901.frame[s_jy901.frame_idx++] = byte;

    if (s_jy901.frame_idx < JY901S_FRAME_LEN)
    {
        return;
    }

    if (JY901S_CheckFrame(s_jy901.frame) != 0U)
    {
        JY901S_ParseFrame(s_jy901.frame);
    }

    s_jy901.frame_idx = 0U;
}

static uint8_t JY901S_CheckFrame(const uint8_t *frame)
{
    uint8_t i;
    uint8_t sum = 0U;

    if (frame[0] != 0x55U)
    {
        return 0U;
    }

    for (i = 0U; i < JY901S_FRAME_LEN - 1U; ++i)
    {
        sum = (uint8_t)(sum + frame[i]);
    }

    return (uint8_t)(sum == frame[JY901S_FRAME_LEN - 1U]);
}

static void JY901S_ParseFrame(const uint8_t *frame)
{
    int16_t raw0;
    int16_t raw1;
    int16_t raw2;

    raw0 = (int16_t)((uint16_t)frame[3] << 8U | frame[2]);
    raw1 = (int16_t)((uint16_t)frame[5] << 8U | frame[4]);
    raw2 = (int16_t)((uint16_t)frame[7] << 8U | frame[6]);

    switch (frame[1])
    {
    case 0x51U:
        s_jy901.latest.acc[0] = ((float)raw0 / 32768.0f) * 16.0f;
        s_jy901.latest.acc[1] = ((float)raw1 / 32768.0f) * 16.0f;
        s_jy901.latest.acc[2] = ((float)raw2 / 32768.0f) * 16.0f;
        s_jy901.latest.valid_mask |= 0x01U;
        break;

    case 0x52U:
        s_jy901.latest.gyro[0] = ((float)raw0 / 32768.0f) * 2000.0f;
        s_jy901.latest.gyro[1] = ((float)raw1 / 32768.0f) * 2000.0f;
        s_jy901.latest.gyro[2] = ((float)raw2 / 32768.0f) * 2000.0f;
        s_jy901.latest.valid_mask |= 0x02U;
        break;

    case 0x53U:
        s_jy901.latest.angle[0] = ((float)raw0 / 32768.0f) * 180.0f;
        s_jy901.latest.angle[1] = ((float)raw1 / 32768.0f) * 180.0f;
        s_jy901.latest.angle[2] = ((float)raw2 / 32768.0f) * 180.0f;
        s_jy901.latest.valid_mask |= 0x04U;
        break;

    default:
        return;
    }

    s_jy901.latest.update_tick = xTaskGetTickCount();
    s_jy901.has_data = 1U;
}
