#ifndef __LIDAR_H
#define __LIDAR_H

#include "main.h"

/* LiDAR DMA 接收缓存 */
#define LIDAR_DMA_ADDR              0x30000000U /* LiDAR 串口 DMA 缓冲地址，应放在 DMA 可访问区域。 */
#define LIDAR_DMA_BUF_LEN                  256U /* LiDAR 串口 DMA 环形缓冲长度。 */

/* LiDAR 数据帧协议 */
#define LIDAR_FRAME_HEADER              0x54U   /* WHEELTEC 单线雷达数据帧头。 */
#define LIDAR_FRAME_VER_LEN             0x2CU   /* 雷达协议中的版本/长度字段。 */
#define LIDAR_POINT_PER_PACK              12U   /* 每包包含的测距点数量。 */
#define LIDAR_FRAME_LEN                  (LIDAR_FRAME_VER_LEN + 3U) /* 完整雷达数据帧长度。 */

/* LiDAR 整圈扫描缓存 */
#define LIDAR_SCAN_POINT_COUNT           500U   /* 整圈按 0.72 度离散后的点数。 */
#define LIDAR_SCAN_ANGLE_STEP_DEG        0.72f  /* 整圈扫描缓存的角度分辨率。 */

typedef struct
{
    uint16_t distance;
    uint8_t intensity;
} LiDAR_Point;

typedef struct
{
    uint16_t speed_dps;
    uint16_t start_angle_cdeg;
    LiDAR_Point points[LIDAR_POINT_PER_PACK];
    uint16_t end_angle_cdeg;
    uint16_t timestamp_ms;
} LiDAR_Frame;

typedef void (*LiDAR_FrameCallback)(const LiDAR_Frame *frame);

/* Public APIs */
void LiDAR_Init(UART_HandleTypeDef *huart);
void LiDAR_SetFrameCallback(LiDAR_FrameCallback cb);
void LiDAR_ResetParser(void);
void LiDAR_OnRxBytes(const uint8_t *data, uint16_t len);
void LiDAR_DecodeFrame(uint8_t *pData, uint16_t length);
int  LiDAR_GetLatestFrame(LiDAR_Frame *out);

void LiDAR_SetDmaBuffer(uint8_t *buf, uint16_t len);
void LiDAR_StartRxDMA(void);
void LiDAR_ProcessDmaPosition(uint16_t pos);
void LiDAR_PollDma(void);

uint8_t LiDAR_CalcCRC8(const uint8_t *data, uint16_t len);

float LiDAR_RawToDistance(uint32_t raw_val);
float LiDAR_PointAngleDeg(const LiDAR_Frame *frame, uint8_t index);

/*
 * ==================== LiDAR 消费接口说明 ====================
 *
 * 下面这组接口面向上层控制/决策逻辑消费雷达数据。
 * 当前实现采用“双缓冲”方式：
 *
 * 1. 写缓冲会随着串口数据到来持续被刷新；
 * 2. 当检测到角度回卷并完成同步后，会把上一整圈提交为“读快照”；
 * 3. LiDAR_IsScanReady() 置位表示“有一份新的整圈快照可读”；
 * 4. 因此：
 *    - LiDAR_GetDistanceArray()/LiDAR_GetDistanceByAngle() 读取冻结快照；
 *    - LiDAR_GetLiveDistanceArray()/LiDAR_GetLiveDistanceByAngle() 读取正在更新
 *      的实时写缓冲，适合只关心少量固定方向的低延迟场景。
 *
 * 当前车体角约定：
 * - 输入角度使用车体坐标系：0=前，90=右，180=后，270=左；
 * - 也支持传入负角：负号表示车头左侧方向，例如 -30=前左 30 度，-90=左；
 * - 内部会映射到当前雷达安装方向对应的原始扫描索引。
 *
 * 当前整圈距离数组的典型索引对应关系：
 * - index 0   -> 车体左侧
 * - index 125 -> 车体后侧
 * - index 250 -> 车体右侧
 * - index 375 -> 车体前侧
 *
 * 因此：
 * - LiDAR_GetDistanceArray()/LiDAR_GetLiveDistanceArray() 返回的是“距离数组”，
 *   每个数组元素对应 0.72 度一个槽位；
 * - LiDAR_GetDistanceByAngle()/LiDAR_GetLiveDistanceByAngle() 返回的是“指定车体角
 *   度方向上的距离值”，不是角度值本身。
 */
uint8_t LiDAR_IsScanReady(void);
void LiDAR_ClearScanReady(void);
void LiDAR_GetDistanceArray(uint16_t *dest, uint16_t size);
/* 按车体角查询冻结快照距离：0=前，90=右，180=后，270=左。 */
uint16_t LiDAR_GetDistanceByAngle(float angle_deg);
void LiDAR_GetLiveDistanceArray(uint16_t *dest, uint16_t size);
/* 按车体角查询实时距离：0=前，90=右，180=后，270=左。 */
uint16_t LiDAR_GetLiveDistanceByAngle(float angle_deg);

int LiDAR_SendCommand(uint8_t mode, const uint8_t data4[4]);
int LiDAR_CmdStartMotor(void);
int LiDAR_CmdStopMotor(void);
int LiDAR_CmdSetSpeed(uint16_t speed_dps);
int LiDAR_CmdQuerySpeed(void);

#endif
