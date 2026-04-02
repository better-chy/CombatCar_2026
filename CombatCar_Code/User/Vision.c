#include "Vision.h"

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

typedef enum
{
    VISION_BUF_FREE = 0U,
    VISION_BUF_READY,
    VISION_BUF_READING
} Vision_BufferState;

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t *rx_dma_buf;
    volatile uint8_t pending_buf[VISION_PENDING_SLOTS][VISION_RX_BUF_LEN];
    volatile uint16_t pending_len[VISION_PENDING_SLOTS];
    volatile uint8_t pending_state[VISION_PENDING_SLOTS];
    volatile uint32_t pending_seq[VISION_PENDING_SLOTS];
    volatile uint32_t seq_counter;

    uint8_t frame[VISION_PAYLOAD_LEN + 4U];
    uint16_t frame_idx;
    uint16_t expected_total;

    Vision_Tag latest;
    volatile uint8_t has_tag;
} Vision_Context;

static Vision_Context s_vision;

static int32_t Vision_GetFreeSlot(const Vision_Context *ctx);
static int32_t Vision_GetOldestReadySlot(const Vision_Context *ctx);
static int32_t Vision_ClaimReadySlot(Vision_Context *ctx, uint16_t *len_out);
static void Vision_ReleaseReadingSlot(Vision_Context *ctx, uint8_t slot);
static void Vision_OnBytes(const uint8_t *data, uint16_t len);
static void Vision_HandleByte(uint8_t byte);
static uint8_t Vision_Checksum8(const uint8_t *data, uint16_t len);
static void Vision_ParseFrame(const uint8_t *frame, uint16_t len);

void Vision_Init(UART_HandleTypeDef *huart)
{
    memset(&s_vision, 0, sizeof(s_vision));
    s_vision.huart = huart;
    s_vision.rx_dma_buf = (uint8_t *)VISION_DMA_ADDR;
    Vision_StartRx();
}

void Vision_StartRx(void)
{
    if (s_vision.huart == NULL || s_vision.rx_dma_buf == NULL)
    {
        return;
    }

    memset(s_vision.rx_dma_buf, 0, VISION_RX_BUF_LEN);
    if (HAL_UART_Receive_DMA(s_vision.huart, s_vision.rx_dma_buf, VISION_RX_BUF_LEN) == HAL_OK &&
        s_vision.huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(s_vision.huart->hdmarx, DMA_IT_HT);
    }
}

void Vision_ProcessPending(void)
{
    int32_t slot;
    uint16_t slot_len;

    while (1)
    {
        slot = Vision_ClaimReadySlot(&s_vision, &slot_len);
        if (slot < 0)
        {
            break;
        }

        Vision_OnBytes((const uint8_t *)s_vision.pending_buf[slot], slot_len);
        Vision_ReleaseReadingSlot(&s_vision, (uint8_t)slot);
    }
}

void Vision_HandleRxEventFromISR(UART_HandleTypeDef *huart, uint16_t size)
{
    int32_t slot;
    UBaseType_t saved_interrupt_status;

    if (huart != s_vision.huart || s_vision.huart == NULL)
    {
        return;
    }

    if (size > VISION_RX_BUF_LEN)
    {
        size = VISION_RX_BUF_LEN;
    }

    if (size == 0U)
    {
        Vision_StartRx();
        return;
    }

    saved_interrupt_status = taskENTER_CRITICAL_FROM_ISR();
    slot = Vision_GetFreeSlot(&s_vision);
    if (slot < 0)
    {
        slot = Vision_GetOldestReadySlot(&s_vision);
    }

    if (slot >= 0)
    {
        memcpy((void *)s_vision.pending_buf[slot], s_vision.rx_dma_buf, size);
        s_vision.pending_len[slot] = size;
        s_vision.pending_seq[slot] = ++s_vision.seq_counter;
        s_vision.pending_state[slot] = VISION_BUF_READY;
    }
    taskEXIT_CRITICAL_FROM_ISR(saved_interrupt_status);

    Vision_StartRx();
}

void Vision_HandleError(UART_HandleTypeDef *huart)
{
    if (huart != s_vision.huart || s_vision.huart == NULL)
    {
        return;
    }

    (void)HAL_UART_DMAStop(huart);
    Vision_StartRx();
}

uint8_t Vision_GetLatestTag(Vision_Tag *out)
{
    if (out == NULL || s_vision.has_tag == 0U)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    *out = s_vision.latest;
    taskEXIT_CRITICAL();
    return 1U;
}

uint8_t Vision_IsOnline(uint32_t timeout_ms)
{
    if (s_vision.has_tag == 0U)
    {
        return 0U;
    }

    return (uint8_t)((xTaskGetTickCount() - s_vision.latest.update_tick) <= pdMS_TO_TICKS(timeout_ms));
}

static int32_t Vision_GetFreeSlot(const Vision_Context *ctx)
{
    uint32_t i;

    for (i = 0U; i < VISION_PENDING_SLOTS; ++i)
    {
        if (ctx->pending_state[i] == VISION_BUF_FREE)
        {
            return (int32_t)i;
        }
    }

    return -1;
}

static int32_t Vision_GetOldestReadySlot(const Vision_Context *ctx)
{
    uint32_t i;
    int32_t oldest = -1;
    uint32_t oldest_seq = 0U;

    for (i = 0U; i < VISION_PENDING_SLOTS; ++i)
    {
        if (ctx->pending_state[i] != VISION_BUF_READY)
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

static int32_t Vision_ClaimReadySlot(Vision_Context *ctx, uint16_t *len_out)
{
    int32_t slot;

    taskENTER_CRITICAL();
    slot = Vision_GetOldestReadySlot(ctx);
    if (slot >= 0)
    {
        *len_out = ctx->pending_len[slot];
        ctx->pending_state[slot] = VISION_BUF_READING;
    }
    taskEXIT_CRITICAL();

    return slot;
}

static void Vision_ReleaseReadingSlot(Vision_Context *ctx, uint8_t slot)
{
    taskENTER_CRITICAL();
    ctx->pending_state[slot] = VISION_BUF_FREE;
    ctx->pending_len[slot] = 0U;
    ctx->pending_seq[slot] = 0U;
    taskEXIT_CRITICAL();
}

static void Vision_OnBytes(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    for (i = 0U; i < len; ++i)
    {
        Vision_HandleByte(data[i]);
    }
}

static void Vision_HandleByte(uint8_t byte)
{
    if (s_vision.frame_idx == 0U)
    {
        if (byte == VISION_SOF1)
        {
            s_vision.frame[0] = byte;
            s_vision.frame_idx = 1U;
        }
        return;
    }

    if (s_vision.frame_idx == 1U)
    {
        if (byte == VISION_SOF2)
        {
            s_vision.frame[1] = byte;
            s_vision.frame_idx = 2U;
            return;
        }

        s_vision.frame_idx = 0U;
        return;
    }

    if (s_vision.frame_idx == 2U)
    {
        s_vision.frame[2] = byte;
        if (byte != VISION_PAYLOAD_LEN)
        {
            s_vision.frame_idx = 0U;
            s_vision.expected_total = 0U;
            return;
        }

        s_vision.expected_total = (uint16_t)(byte + 4U);
        s_vision.frame_idx = 3U;
        return;
    }

    if (s_vision.frame_idx >= sizeof(s_vision.frame))
    {
        s_vision.frame_idx = 0U;
        s_vision.expected_total = 0U;
        return;
    }

    s_vision.frame[s_vision.frame_idx++] = byte;
    if (s_vision.expected_total == 0U)
    {
        s_vision.frame_idx = 0U;
        return;
    }

    if (s_vision.frame_idx >= s_vision.expected_total)
    {
        uint8_t checksum = Vision_Checksum8(&s_vision.frame[2], (uint16_t)(s_vision.expected_total - 3U));
        if (checksum == s_vision.frame[s_vision.expected_total - 1U])
        {
            Vision_ParseFrame(s_vision.frame, s_vision.expected_total);
        }
        s_vision.frame_idx = 0U;
        s_vision.expected_total = 0U;
    }
}

static uint8_t Vision_Checksum8(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint8_t sum = 0U;

    for (i = 0U; i < len; ++i)
    {
        sum = (uint8_t)(sum + data[i]);
    }

    return sum;
}

static void Vision_ParseFrame(const uint8_t *frame, uint16_t len)
{
    if (frame == NULL || len != (VISION_PAYLOAD_LEN + 4U))
    {
        return;
    }

    if (frame[3] != VISION_MSG_TARGET)
    {
        return;
    }

    taskENTER_CRITICAL();
    s_vision.latest.seq = frame[4];
    s_vision.latest.flags = frame[5];
    s_vision.latest.tag_id = frame[6];
    s_vision.latest.update_tick = xTaskGetTickCount();
    s_vision.has_tag = 1U;
    taskEXIT_CRITICAL();
}
