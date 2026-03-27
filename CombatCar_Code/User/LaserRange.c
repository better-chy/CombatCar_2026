#include "LaserRange.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

#define LASER_RANGE_PENDING_SLOTS 2U

typedef enum
{
    LASER_BUF_FREE = 0U,
    LASER_BUF_READY,
    LASER_BUF_READING
} LaserRange_BufferState;

// 单路激光测距的运行时上下文
typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t *rx_dma_buf;                                     // DMA 直接写入的接收缓冲区（非缓存区）
    volatile uint8_t pending_buf[LASER_RANGE_PENDING_SLOTS][LASER_RANGE_RX_BUF_LEN]; // 双缓冲待处理区
    volatile uint16_t pending_len[LASER_RANGE_PENDING_SLOTS];                         // 每个缓冲区对应的长度
    volatile uint8_t pending_state[LASER_RANGE_PENDING_SLOTS];                        // 缓冲区状态
    volatile uint32_t pending_seq[LASER_RANGE_PENDING_SLOTS];                         // 就绪顺序，值越小越旧
    volatile uint32_t seq_counter;                                                    // 新帧序号计数器
    volatile uint16_t distance_mm;                           // 最新距离值，单位 mm
    volatile uint8_t valid;                                  // 当前距离值是否有效
} LaserRange_Context;

static LaserRange_Context s_laser_ctx[LASER_RANGE_SENSOR_COUNT];     // 四路测距上下文

// 启动测量命令，最后两字节在发送前补 CRC16
static const uint8_t s_start_measuring_cmd[9] = {0xA5U, 0x03U, 0x20U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};

static uint16_t LaserRange_Crc16(const uint8_t *data, uint16_t length);
static void LaserRange_FillCrc(uint8_t *data, uint16_t length);
static LaserRange_Context *LaserRange_FindContext(UART_HandleTypeDef *huart);
static void LaserRange_StartRx(LaserRange_Context *ctx);
static uint8_t LaserRange_TryParseFrame(const uint8_t *data, uint16_t len, uint16_t *distance_mm);
static int32_t LaserRange_GetFreeSlot(const LaserRange_Context *ctx);
static int32_t LaserRange_GetOldestReadySlot(const LaserRange_Context *ctx);
static int32_t LaserRange_ClaimReadySlot(LaserRange_Context *ctx, uint16_t *len_out);
static void LaserRange_ReleaseReadingSlot(LaserRange_Context *ctx, uint8_t slot);

void LaserRange_Init(void)
{
    uint8_t start_cmd[sizeof(s_start_measuring_cmd)];

    memset(s_laser_ctx, 0, sizeof(s_laser_ctx));   // 清空四路测距状态

    // 绑定四个激光测距模块对应的串口
    s_laser_ctx[0].huart = &huart4;
    s_laser_ctx[1].huart = &huart5;
    s_laser_ctx[2].huart = &huart7;
    s_laser_ctx[3].huart = &huart8;
    s_laser_ctx[0].rx_dma_buf = (uint8_t *)(LASER_RANGE_DMA_BASE_ADDR + 0U * LASER_RANGE_DMA_STRIDE);
    s_laser_ctx[1].rx_dma_buf = (uint8_t *)(LASER_RANGE_DMA_BASE_ADDR + 1U * LASER_RANGE_DMA_STRIDE);
    s_laser_ctx[2].rx_dma_buf = (uint8_t *)(LASER_RANGE_DMA_BASE_ADDR + 2U * LASER_RANGE_DMA_STRIDE);
    s_laser_ctx[3].rx_dma_buf = (uint8_t *)(LASER_RANGE_DMA_BASE_ADDR + 3U * LASER_RANGE_DMA_STRIDE);

    memcpy(start_cmd, s_start_measuring_cmd, sizeof(start_cmd));   // 拷贝一份启动命令
    LaserRange_FillCrc(start_cmd, (uint16_t)sizeof(start_cmd));    // 给命令最后两字节补 CRC

    // 四路全部开启接收，并发送开始测量命令
    for (uint8_t i = 0; i < LASER_RANGE_SENSOR_COUNT; ++i)
    {
        LaserRange_StartRx(&s_laser_ctx[i]);
        (void)HAL_UART_Transmit(s_laser_ctx[i].huart, start_cmd, sizeof(start_cmd), 50U);
    }
}

// 在 SensorTask 中处理待解析的数据
void LaserRange_ProcessPending(void)
{
    int32_t slot;
    uint16_t slot_len;
    uint16_t distance_mm;

    for (uint8_t i = 0; i < LASER_RANGE_SENSOR_COUNT; ++i)
    {
        for (;;)
        {
            slot = LaserRange_ClaimReadySlot(&s_laser_ctx[i], &slot_len);
            if (slot < 0)
            {
                break;
            }

            if (LaserRange_TryParseFrame((const uint8_t *)s_laser_ctx[i].pending_buf[slot], slot_len, &distance_mm) != 0U)
            {
                s_laser_ctx[i].distance_mm = distance_mm;
                s_laser_ctx[i].valid = 1U;
            }

            LaserRange_ReleaseReadingSlot(&s_laser_ctx[i], (uint8_t)slot);
        }
    }
}

// UART 回调使用：把 DMA 收到的数据拷贝到待处理缓冲区
void LaserRange_HandleRxEventFromISR(UART_HandleTypeDef *huart, uint16_t size)
{
    LaserRange_Context *ctx = LaserRange_FindContext(huart);

    if (ctx == NULL)
    {
        return;
    }

    if (size > LASER_RANGE_RX_BUF_LEN)
    {
        size = LASER_RANGE_RX_BUF_LEN;
    }

    if (size > 0U)
    {
        int32_t slot = LaserRange_GetFreeSlot(ctx);
        if (slot < 0)
        {
            // 新数据优先：没有空闲槽时，覆盖最旧的 READY 槽，不覆盖任务正在读的槽
            slot = LaserRange_GetOldestReadySlot(ctx);
        }

        if (slot >= 0)
        {
            memcpy((void *)ctx->pending_buf[slot], ctx->rx_dma_buf, size);
            ctx->pending_len[slot] = size;
            ctx->pending_seq[slot] = ++ctx->seq_counter;
            ctx->pending_state[slot] = LASER_BUF_READY;
        }
    }

    // 继续启动下一次接收
    LaserRange_StartRx(ctx);
}

void LaserRange_HandleError(UART_HandleTypeDef *huart)
{
    LaserRange_Context *ctx = LaserRange_FindContext(huart);

    if (ctx == NULL)
    {
        return;
    }

    (void)HAL_UART_DMAStop(huart);
    LaserRange_StartRx(ctx);
}

// 读取单路距离
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

// 给发送命令补 CRC16
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

// 通过 UART 句柄找到对应的测距上下文
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

static void LaserRange_StartRx(LaserRange_Context *ctx)
{
    HAL_StatusTypeDef status;

    if (ctx == NULL || ctx->huart == NULL)
    {
        return;
    }

    status = HAL_UART_Receive_DMA(ctx->huart, ctx->rx_dma_buf, LASER_RANGE_RX_BUF_LEN);

    // 关闭 DMA 半传输中断，减少无意义中断次数
    if (status == HAL_OK && ctx->huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(ctx->huart->hdmarx, DMA_IT_HT);
    }
}

// 数据协议解析函数：保守模式，只接受完整 23 字节整包
static uint8_t LaserRange_TryParseFrame(const uint8_t *data, uint16_t len, uint16_t *distance_mm)
{
    uint16_t crc_calc;  // 根据收到的数据重新计算出的 CRC
    uint16_t crc_frame; // 数据帧最后两个字节携带的 CRC

    if (data == NULL || distance_mm == NULL || len != LASER_RANGE_RX_BUF_LEN)
    {
        return 0U;
    }

    crc_calc = LaserRange_Crc16(data, LASER_RANGE_RX_BUF_LEN - 2U);
    crc_frame = (uint16_t)data[LASER_RANGE_RX_BUF_LEN - 2U]
              | ((uint16_t)data[LASER_RANGE_RX_BUF_LEN - 1U] << 8U);

    if (crc_calc != crc_frame)
    {
        return 0U;
    }

    *distance_mm = (uint16_t)data[13U]
                 | ((uint16_t)data[14U] << 8U);
    return 1U;
}

static int32_t LaserRange_GetFreeSlot(const LaserRange_Context *ctx)
{
    for (uint8_t i = 0; i < LASER_RANGE_PENDING_SLOTS; ++i)
    {
        if (ctx->pending_state[i] == LASER_BUF_FREE)
        {
            return (int32_t)i;
        }
    }

    return -1;
}

static int32_t LaserRange_GetOldestReadySlot(const LaserRange_Context *ctx)
{
    int32_t slot = -1;
    uint32_t oldest_seq = 0U;

    for (uint8_t i = 0; i < LASER_RANGE_PENDING_SLOTS; ++i)
    {
        if (ctx->pending_state[i] != LASER_BUF_READY)
        {
            continue;
        }

        if (slot < 0 || ctx->pending_seq[i] < oldest_seq)
        {
            slot = (int32_t)i;
            oldest_seq = ctx->pending_seq[i];
        }
    }

    return slot;
}

static int32_t LaserRange_ClaimReadySlot(LaserRange_Context *ctx, uint16_t *len_out)
{
    int32_t slot;

    taskENTER_CRITICAL();
    slot = LaserRange_GetOldestReadySlot(ctx);
    if (slot >= 0)
    {
        uint16_t len = ctx->pending_len[slot];
        if (len > LASER_RANGE_RX_BUF_LEN)
        {
            len = LASER_RANGE_RX_BUF_LEN;
        }
        ctx->pending_state[slot] = LASER_BUF_READING;
        *len_out = len;
    }
    taskEXIT_CRITICAL();

    return slot;
}

static void LaserRange_ReleaseReadingSlot(LaserRange_Context *ctx, uint8_t slot)
{
    if (slot >= LASER_RANGE_PENDING_SLOTS)
    {
        return;
    }

    taskENTER_CRITICAL();
    ctx->pending_state[slot] = LASER_BUF_FREE;
    ctx->pending_len[slot] = 0U;
    ctx->pending_seq[slot] = 0U;
    taskEXIT_CRITICAL();
}
