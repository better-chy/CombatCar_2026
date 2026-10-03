#include "LiDAR.h"
#include <string.h>

/* LiDAR 私有协议常量 */
#define LIDAR_CMD_HEADER          0x54U  /* WHEELTEC 雷达命令帧头。 */
#define LIDAR_CMD_LEN                8U  /* 雷达控制命令长度。 */
#define LIDAR_CMD_DATA_LEN        0x04U  /* 雷达控制命令数据长度字段。 */
#define LIDAR_ANGLE_FULL_CDEG    36000U  /* 一整圈角度，单位 0.01 度。 */
#define LIDAR_CRC8_POLY          0x4DU   /* 雷达协议 CRC8 多项式。 */

/* LiDAR 安装方向 */
#define LIDAR_CAR_FRONT_OFFSET_DEG 90.0f /* 雷达原始 0 度相对车头的安装偏置。 */

typedef struct
{
    uint8_t buf[LIDAR_FRAME_LEN];
    uint16_t idx;
    uint16_t expected_total;
} LiDAR_Parser;

static const uint8_t s_crc8_table[256] =
{
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

static UART_HandleTypeDef *s_huart = NULL;
static LiDAR_Parser s_parser;
static LiDAR_Frame s_latest;
static volatile uint8_t s_has_frame = 0;
static volatile uint8_t s_scan_ready = 0;
static LiDAR_FrameCallback s_frame_cb = NULL;
static uint16_t s_scan_buf_a[LIDAR_SCAN_POINT_COUNT];
static uint16_t s_scan_buf_b[LIDAR_SCAN_POINT_COUNT];
static uint16_t *s_scan_write_buf = s_scan_buf_a;
static uint16_t *s_scan_read_buf = s_scan_buf_b;
static uint16_t s_last_end_angle_cdeg = 0;
static uint8_t s_scan_synced = 0U;

static uint8_t *s_dma_buf = (uint8_t *)LIDAR_DMA_ADDR;
static uint16_t s_dma_len = LIDAR_DMA_BUF_LEN;
static uint16_t s_dma_last_pos = 0;

static void LiDAR_ParseFrame(const uint8_t *buf, uint16_t len);
static void LiDAR_HandleByte(uint8_t b);
static void LiDAR_UpdateScan(const LiDAR_Frame *frame);
static void LiDAR_ClearScanBuffer(uint16_t *buf);
static void LiDAR_SwapScanBuffers(void);
static uint16_t LiDAR_MapAngleToIndex(float angle_deg);
static uint16_t LiDAR_GetDistanceFromBuffer(const uint16_t *buffer, float angle_deg);

void LiDAR_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_dma_buf = (uint8_t *)LIDAR_DMA_ADDR;     //指定dma内存地址，防止开启缓存后cpu的dma数据不一致
    s_dma_len = LIDAR_DMA_BUF_LEN;
    s_dma_last_pos = 0;
    s_frame_cb = NULL;
    s_has_frame = 0;
    s_scan_ready = 0;
    s_last_end_angle_cdeg = 0;
    s_scan_synced = 0U;
    s_scan_write_buf = s_scan_buf_a;
    s_scan_read_buf = s_scan_buf_b;
    memset(&s_latest, 0, sizeof(s_latest));
    LiDAR_ClearScanBuffer(s_scan_buf_a);
    LiDAR_ClearScanBuffer(s_scan_buf_b);
    LiDAR_ResetParser();
}

void LiDAR_SetFrameCallback(LiDAR_FrameCallback cb)
{
    s_frame_cb = cb;
}

void LiDAR_ResetParser(void)
{
    memset(&s_parser, 0, sizeof(s_parser));
}

// 将连续字节逐个喂给LiDAR_HandleByte()函数
void LiDAR_OnRxBytes(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U)
    {
        return;
    }
    for (uint16_t i = 0; i < len; ++i)
    {
        LiDAR_HandleByte(data[i]);
    }
}

void LiDAR_DecodeFrame(uint8_t *pData, uint16_t length)
{
    LiDAR_OnRxBytes(pData, length);
}

int LiDAR_GetLatestFrame(LiDAR_Frame *out)
{
    if (out == NULL)
    {
        return 0;
    }
    if (!s_has_frame)
    {
        return 0;
    }
    __disable_irq();
    *out = s_latest;
    s_has_frame = 0;
    __enable_irq();
    return 1;
}

void LiDAR_SetDmaBuffer(uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0U)
    {
        return;
    }
    s_dma_buf = buf;
    s_dma_len = len;
    s_dma_last_pos = 0;
}

void LiDAR_StartRxDMA(void)
{
    HAL_StatusTypeDef status;

    //如果前面的init初始化失败，这里就直接跳过，不要接收了
    if (s_huart == NULL || s_dma_buf == NULL || s_dma_len == 0U)
    {
        return;
    }
    memset(s_dma_buf, 0, s_dma_len);
    s_dma_last_pos = 0;

    /*这里就是开启一次接收，该函数为空闲中断，空闲中断是stm32自己判断什么时候接受完成，
    不用手动指定固定长度，最后的len参数是缓冲区容量，也就是最多可以写多少字节，
    空闲中断会在串口发生空闲（可能是接收到了一帧数据包，或者缓冲区满了，或者是一段时间没数据），发生一次中断*/
    status = HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_buf, s_dma_len);
    if (status != HAL_OK)
    {
        //空闲中断启动失败，退化到普通dma中断接收
        status = HAL_UART_Receive_DMA(s_huart, s_dma_buf, s_dma_len);
    }

    if (status != HAL_OK)
    {
        return;
    }

    //关闭半传输中断，也就是dma接收了一半的话，不要进中断
    if (s_huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT);
    }
}

//从上次处理到的位置s_dma_last_pos，处理到现在dma写到的位置
void LiDAR_ProcessDmaPosition(uint16_t pos)
{
    if (s_dma_buf == NULL || s_dma_len == 0U)
    {
        return;
    }

    if (pos > s_dma_len)
    {
        pos = (uint16_t)(pos % s_dma_len);
    }

    if (pos == s_dma_last_pos)
    {
        return;
    }

    //现在的位置大于上次处理到的位置，吊椅onrxbytes处理
    if (pos > s_dma_last_pos)
    {
        LiDAR_OnRxBytes(&s_dma_buf[s_dma_last_pos], (uint16_t)(pos - s_dma_last_pos));
    }
    else
    {
        //如果是小于，比如说我上次处理到是240，这次pos变成了2，说明环形缓冲区满了，现在是新的一圈，新数据分两段
        LiDAR_OnRxBytes(&s_dma_buf[s_dma_last_pos], (uint16_t)(s_dma_len - s_dma_last_pos));
        if (pos > 0U)
        {
            LiDAR_OnRxBytes(&s_dma_buf[0], pos);
        }
    }
    s_dma_last_pos = pos;
}

void LiDAR_PollDma(void)
{
    uint16_t pos;   //dma解析位置信息

    //防御性编程
    if (s_huart == NULL || s_huart->hdmarx == NULL || s_dma_buf == NULL || s_dma_len == 0U)
    {
        return;
    }

    //__HAL_DMA_GET_COUNTER(s_huart->hdmarx) 返回现在dma剩下多少字节没写完
    pos = (uint16_t)(s_dma_len - __HAL_DMA_GET_COUNTER(s_huart->hdmarx));  //减去剩下多少没写，说明现在写到了多少字节
    LiDAR_ProcessDmaPosition(pos);
}

uint8_t LiDAR_CalcCRC8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00U;
    if (data == NULL)
    {
        return crc;
    }
    for (uint16_t i = 0; i < len; ++i)
    {
        crc = s_crc8_table[(crc ^ data[i]) & 0xFFU];
    }
    return crc;
}

float LiDAR_RawToDistance(uint32_t raw_val)
{
    return (float)raw_val * 0.001f;
}

float LiDAR_PointAngleDeg(const LiDAR_Frame *frame, uint8_t index)
{
    if (frame == NULL || index >= LIDAR_POINT_PER_PACK)
    {
        return 0.0f;
    }
    uint16_t start = frame->start_angle_cdeg;
    uint16_t end = frame->end_angle_cdeg;
    uint16_t span;
    if (end >= start)
    {
        span = (uint16_t)(end - start);
    }
    else
    {
        span = (uint16_t)(end + LIDAR_ANGLE_FULL_CDEG - start);
    }
    float step = 0.0f;
    if (LIDAR_POINT_PER_PACK > 1U)
    {
        step = (float)span / (float)(LIDAR_POINT_PER_PACK - 1U);
    }
    float angle = (float)start + step * (float)index;
    if (angle >= (float)LIDAR_ANGLE_FULL_CDEG)
    {
        angle -= (float)LIDAR_ANGLE_FULL_CDEG;
    }
    return angle * 0.01f;
}

/*
 * ==================== LiDAR 消费接口说明 ====================
 *
 * 当前实现采用双缓冲：
 * 1. s_scan_write_buf 持续接收并更新当前正在扫描的一圈；
 * 2. 检测到角度回卷后，上一整圈会通过指针交换提交到 s_scan_read_buf；
 * 3. LiDAR_GetDistanceArray()/LiDAR_GetDistanceByAngle() 读取冻结快照；
 * 4. LiDAR_GetLiveDistanceArray()/LiDAR_GetLiveDistanceByAngle() 读取实时写缓冲。
 *
 * 这样既能给整圈分析提供一致快照，也能给临时避障保留低延迟实时值。
 */
uint8_t LiDAR_IsScanReady(void)
{
    return s_scan_ready;
}

void LiDAR_ClearScanReady(void)
{
    s_scan_ready = 0;
}

//===================================================获取角度数据======================
void LiDAR_GetDistanceArray(uint16_t *dest, uint16_t size)
{
    const uint16_t *read_buf;

    if (dest == NULL || size == 0U)
    {
        return;
    }

    if (size > LIDAR_SCAN_POINT_COUNT)
    {
        size = LIDAR_SCAN_POINT_COUNT;
    }

    read_buf = s_scan_read_buf;
    memcpy(dest, read_buf, (size_t)size * sizeof(uint16_t));
}

uint16_t LiDAR_GetDistanceByAngle(float angle_deg)
{
    return LiDAR_GetDistanceFromBuffer(s_scan_read_buf, angle_deg);
}

void LiDAR_GetLiveDistanceArray(uint16_t *dest, uint16_t size)
{
    const uint16_t *write_buf;

    if (dest == NULL || size == 0U)
    {
        return;
    }

    if (size > LIDAR_SCAN_POINT_COUNT)
    {
        size = LIDAR_SCAN_POINT_COUNT;
    }

    write_buf = s_scan_write_buf;
    memcpy(dest, write_buf, (size_t)size * sizeof(uint16_t));
}

uint16_t LiDAR_GetLiveDistanceByAngle(float angle_deg)
{
    return LiDAR_GetDistanceFromBuffer(s_scan_write_buf, angle_deg);
}
//=================================================================================

//逐个封装字节并阻塞发送
int LiDAR_SendCommand(uint8_t mode, const uint8_t data4[4])
{
    if (s_huart == NULL || data4 == NULL)
    {
        return 0;
    }
    uint8_t frame[LIDAR_CMD_LEN];
    frame[0] = LIDAR_CMD_HEADER;
    frame[1] = mode;
    frame[2] = LIDAR_CMD_DATA_LEN;
    frame[3] = data4[0];
    frame[4] = data4[1];
    frame[5] = data4[2];
    frame[6] = data4[3];
    frame[7] = LiDAR_CalcCRC8(frame, 7U);
    return (HAL_UART_Transmit(s_huart, frame, LIDAR_CMD_LEN, 50U) == HAL_OK) ? 1 : 0;
}

int LiDAR_CmdStartMotor(void)
{
    uint8_t d[4] = {0U, 0U, 0U, 0U};
    return LiDAR_SendCommand(0xA0U, d);
}

int LiDAR_CmdStopMotor(void)
{
    uint8_t d[4] = {0U, 0U, 0U, 0U};
    return LiDAR_SendCommand(0xA1U, d);
}

int LiDAR_CmdSetSpeed(uint16_t speed_dps)
{
    uint8_t d[4];
    d[0] = (uint8_t)(speed_dps & 0xFFU);
    d[1] = (uint8_t)((speed_dps >> 8U) & 0xFFU);
    d[2] = 0U;
    d[3] = 0U;
    return LiDAR_SendCommand(0xA2U, d);
}

int LiDAR_CmdQuerySpeed(void)
{
    uint8_t d[4] = {0U, 0U, 0U, 0U};
    return LiDAR_SendCommand(0xA3U, d);
}

//逐字节处理，从一堆字节解析一帧数据
static void LiDAR_HandleByte(uint8_t b)
{
    //没接收到帧头0x54，检查是不是帧头
    if (s_parser.idx == 0U)
    {
        //是帧头
        if (b == LIDAR_FRAME_HEADER)
        {
            s_parser.buf[0] = b;
            s_parser.idx = 1U;
        }
        return;
    }
    //已经发现了帧头，再次检查
    if (s_parser.idx == 1U)
    {
        //如果发现还是帧头，那么把index重置，上一个0x54可能是乱数据
        if (b == LIDAR_FRAME_HEADER)
        {
            s_parser.buf[0] = b;
            s_parser.idx = 1U;
            return;
        }
        //如果不是0x54，那么继续
        s_parser.buf[1] = b;
        //如果是0x2c（完整的数据帧头是0x54,0x2c），那么说明帧头校验成功
        if (b == LIDAR_FRAME_VER_LEN)
        {
            //更新预期总长度
            s_parser.expected_total = (uint16_t)b + 3U;
        }
        //如果是0x04，有可能是雷达的命令返回帧，更新总预期长度
        else if (b == LIDAR_CMD_DATA_LEN)
        {
            s_parser.expected_total = (uint16_t)b + 4U;
        }
        //如果都不是这两种，就说明之前的0x54是乱的，并不是完整的一帧
        else
        {
            s_parser.idx = 0U;
            s_parser.expected_total = 0U;
            return;
        }
        if (s_parser.expected_total > (uint16_t)sizeof(s_parser.buf) && b != LIDAR_CMD_DATA_LEN)
        {
            s_parser.idx = 0U;
            s_parser.expected_total = 0U;
            return;
        }
        s_parser.idx = 2U;
        return;
    }
    //前面的帧头正确，继续接收后面的字节
    s_parser.buf[s_parser.idx++] = b;
    //保护性代码，正常情况total是有值的
    if (s_parser.expected_total == 0U)
    {
        s_parser.idx = 0U;
        return;
    }

    //进行crc校验
    if (s_parser.idx >= s_parser.expected_total)
    {
        uint8_t crc = LiDAR_CalcCRC8(s_parser.buf, (uint16_t)(s_parser.expected_total - 1U));
        if (crc == s_parser.buf[s_parser.expected_total - 1U])
        {
            if (s_parser.buf[1] == LIDAR_FRAME_VER_LEN)
            {
                //将原始47字节拆为结构体
                LiDAR_ParseFrame(s_parser.buf, s_parser.expected_total);
            }
        }

        //不论校验是否成功，清空状态
        s_parser.idx = 0U;
        s_parser.expected_total = 0U;
    }
}

//将原始47字节拆为结构体
static void LiDAR_ParseFrame(const uint8_t *buf, uint16_t len)
{
    //再次检查， 防御性编程
    if (buf == NULL || len < LIDAR_FRAME_LEN)
    {
        return;
    }
    if (buf[0] != LIDAR_FRAME_HEADER || buf[1] != LIDAR_FRAME_VER_LEN)
    {
        return;
    }

    //从buf[2]开始，跳过帧头
    uint16_t idx = 2U;
    //激光雷达扫描速度
    s_latest.speed_dps = (uint16_t)(buf[idx] | ((uint16_t)buf[idx + 1U] << 8U));
    idx += 2U;
    //起始角
    s_latest.start_angle_cdeg = (uint16_t)(buf[idx] | ((uint16_t)buf[idx + 1U] << 8U));
    idx += 2U;
    //12个点，每个点3字节，是距离的低字节，高字节，光强
    for (uint8_t i = 0U; i < LIDAR_POINT_PER_PACK; ++i)
    {
        s_latest.points[i].distance = (uint16_t)(buf[idx] | ((uint16_t)buf[idx + 1U] << 8U));
        s_latest.points[i].intensity = buf[idx + 2U];
        idx += 3U;
    }
    //结束角
    s_latest.end_angle_cdeg = (uint16_t)(buf[idx] | ((uint16_t)buf[idx + 1U] << 8U));
    idx += 2U;
    //时间
    s_latest.timestamp_ms = (uint16_t)(buf[idx] | ((uint16_t)buf[idx + 1U] << 8U));
    //解析完一个包了
    LiDAR_UpdateScan(&s_latest);
    s_has_frame = 1U;
    if (s_frame_cb != NULL)
    {
        s_frame_cb(&s_latest);
    }
}

//将一个包12点，塞进整圈距离数组
static void LiDAR_UpdateScan(const LiDAR_Frame *frame)
{
    uint16_t start_angle_cdeg;

    if (frame == NULL)
    {
        return;
    }

    //上个函数的s_latest是结构体，传给了这个函数，现在起始角从结构体里读取
    start_angle_cdeg = frame->start_angle_cdeg;
    //如果起始角小于结束角，说明是从比如359度到1度，说明开始了新的一圈
    if (start_angle_cdeg < s_last_end_angle_cdeg)
    {
        //程序刚启动，有可能激光雷达从200启动，第一次的小于可能不是完整一圈，所以群里缓存
        if (s_scan_synced == 0U)
        {
            LiDAR_ClearScanBuffer(s_scan_write_buf);
            s_scan_synced = 1U;
        }
        //接收到一整圈，交换读写缓冲
        else
        {
            LiDAR_SwapScanBuffers();
            s_scan_ready = 1U;
        }
    }
    s_last_end_angle_cdeg = frame->end_angle_cdeg;

    //处理一个包的12个点
    for (uint8_t i = 0U; i < LIDAR_POINT_PER_PACK; ++i)
    {
        float angle_deg = LiDAR_PointAngleDeg(frame, i);
        uint16_t index = (uint16_t)(angle_deg / LIDAR_SCAN_ANGLE_STEP_DEG);
        if (index < LIDAR_SCAN_POINT_COUNT)
        {
            s_scan_write_buf[index] = frame->points[i].distance;
        }
    }
}

static void LiDAR_ClearScanBuffer(uint16_t *buf)
{
    if (buf == NULL)
    {
        return;
    }

    memset(buf, 0, (size_t)LIDAR_SCAN_POINT_COUNT * sizeof(uint16_t));
}

static void LiDAR_SwapScanBuffers(void)
{
    //把读缓冲区改为最新的写缓冲区，并清空写缓冲区
    uint16_t *old_read_buf = s_scan_read_buf;

    s_scan_read_buf = s_scan_write_buf;
    s_scan_write_buf = old_read_buf;
    LiDAR_ClearScanBuffer(s_scan_write_buf);
}

static uint16_t LiDAR_MapAngleToIndex(float angle_deg)
{
    uint16_t index;

    /* 将车体角映射到当前雷达安装方向对应的原始扫描索引。
       车体角约定：0=前，90=右，180=后，270=左。
       同时支持负角：-30 表示前左 30 度，-90 表示左侧。

       当前工程里的原始扫描索引约定：
       index 0   = 左
       index 125 = 后
       index 250 = 右
       index 375 = 前

       传参示例：
       angle 0    -> 前方
       angle 90   -> 右侧
       angle -90  -> 左侧
       angle 180  -> 后方 */
    angle_deg += LIDAR_CAR_FRONT_OFFSET_DEG;

    while (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    while (angle_deg >= 360.0f)
    {
        angle_deg -= 360.0f;
    }

    angle_deg = 360.0f - angle_deg;
    if (angle_deg >= 360.0f)
    {
        angle_deg -= 360.0f;
    }

    index = (uint16_t)(angle_deg / LIDAR_SCAN_ANGLE_STEP_DEG);
    if (index >= LIDAR_SCAN_POINT_COUNT)
    {
        index = LIDAR_SCAN_POINT_COUNT - 1U;
    }

    return index;
}

static uint16_t LiDAR_GetDistanceFromBuffer(const uint16_t *buffer, float angle_deg)
{
    if (buffer == NULL)
    {
        return 0U;
    }

    return buffer[LiDAR_MapAngleToIndex(angle_deg)];
}
