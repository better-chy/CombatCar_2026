#ifndef __LIDAR_H
#define __LIDAR_H

#include "main.h"

/* DMA buffer address (placed in non-cacheable RAM if using MPU config) */
#define LIDAR_DMA_ADDR       0x30000000U
#define LIDAR_DMA_BUF_LEN    256U

#define LIDAR_FRAME_HEADER   0x54U
#define LIDAR_FRAME_VER_LEN  0x2CU
#define LIDAR_POINT_PER_PACK 12U
#define LIDAR_FRAME_LEN      (LIDAR_FRAME_VER_LEN + 3U)
#define LIDAR_SCAN_POINT_COUNT 500U
#define LIDAR_SCAN_ANGLE_STEP_DEG 0.72f

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
uint8_t LiDAR_IsScanReady(void);
void LiDAR_ClearScanReady(void);
void LiDAR_GetDistanceArray(uint16_t *dest, uint16_t size);
/* Query by car-body angle: 0=front, 90=right, 180=rear, 270=left. */
uint16_t LiDAR_GetDistanceByAngle(float angle_deg);

int LiDAR_SendCommand(uint8_t mode, const uint8_t data4[4]);
int LiDAR_CmdStartMotor(void);
int LiDAR_CmdStopMotor(void);
int LiDAR_CmdSetSpeed(uint16_t speed_dps);
int LiDAR_CmdQuerySpeed(void);

#endif
