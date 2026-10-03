#ifndef __TASK_STAGE_H
#define __TASK_STAGE_H

#include "main.h"
#include "LaserRange.h"

/*
以右上角为基准
__________________
                |
车头方向          |
                |
                |
                |

*/
typedef enum
{
    STAGE_ON_STAGE = 0,                    /* on_stage */
    STAGE_OFF_PARALLEL = 1,                /* off_parallel */
    STAGE_OFF_CORNER_FRONT_LEFT = 2,       /* off_corner_front_left */
    STAGE_OFF_CORNER_FRONT_RIGHT = 3,      /* off_corner_front_right */
    STAGE_OFF_CORNER_REAR_LEFT = 4,        /* off_corner_rear_left */
    STAGE_OFF_CORNER_REAR_RIGHT = 5,       /* off_corner_rear_right */
    STAGE_OFF_CORNER_UNALIGNED = 6,        /* off_corner_unaligned */
    STAGE_OFF_REENTER_READY = 9            /* off_parallel_reenter_ready */
} StageType;

typedef struct
{
    uint16_t lidar_front_mm;
    uint16_t lidar_right_mm;
    uint16_t lidar_left_mm;

    uint16_t laser_front_mm;
    uint16_t laser_right_mm;
    uint16_t laser_back_mm;
    uint16_t laser_left_mm;

    uint8_t lidar_front_valid_count;
    uint8_t lidar_right_valid_count;
    uint8_t lidar_left_valid_count;
    uint8_t model_label;
    uint8_t model_feature_valid;
    uint8_t skip_reason;
    uint8_t imu_online;
    uint16_t zero_data_mask;
    int16_t roll_deg_x10;
    uint32_t loop_count;
    StageType judged_type;
    StageType candidate_type;
    StageType applied_type;
} StageDebugInfo;

/* Stage 模型 LiDAR 扇区角度，车体坐标系：0 度为车头，正角在右侧，负角在左侧。 */
#define TASKSTAGE_FRONT_SECTOR_MIN_DEG (-35) /* 前方扇区左边界。 */
#define TASKSTAGE_FRONT_SECTOR_MAX_DEG   35  /* 前方扇区右边界。 */
#define TASKSTAGE_RIGHT_SECTOR_MIN_DEG   55  /* 右侧扇区起点。 */
#define TASKSTAGE_RIGHT_SECTOR_MAX_DEG  120  /* 右侧扇区终点。 */
#define TASKSTAGE_LEFT_SECTOR_MIN_DEG  (-120) /* 左侧扇区起点。 */
#define TASKSTAGE_LEFT_SECTOR_MAX_DEG   (-55) /* 左侧扇区终点。 */

/* Stage 模型 LiDAR 扇区点数，角度步进为 1 度采样。 */
#define TASKSTAGE_FRONT_SECTOR_POINT_COUNT 71U /* 前方 -35..35 共 71 点。 */
#define TASKSTAGE_RIGHT_SECTOR_POINT_COUNT 66U /* 右侧 55..120 共 66 点。 */
#define TASKSTAGE_LEFT_SECTOR_POINT_COUNT  66U /* 左侧 -120..-55 共 66 点。 */
#define TASKSTAGE_MAX_SECTOR_POINT_COUNT   TASKSTAGE_FRONT_SECTOR_POINT_COUNT /* 三个扇区中的最大点数。 */

typedef struct
{
    uint32_t timestamp_ms;
    uint16_t lidar_front_mm[TASKSTAGE_FRONT_SECTOR_POINT_COUNT];
    uint16_t lidar_right_mm[TASKSTAGE_RIGHT_SECTOR_POINT_COUNT];
    uint16_t lidar_left_mm[TASKSTAGE_LEFT_SECTOR_POINT_COUNT];
    uint16_t laser_mm[LASER_RANGE_SENSOR_COUNT];
} TaskStageTrainFrame;

void Task_Stage_Run(void *argument);
void TaskStage_CollectTrainFrame(TaskStageTrainFrame *frame_out);
void TaskStage_NotifyReenterComplete(void);

extern volatile StageType g_stage_type;
extern volatile StageDebugInfo g_stage_debug_info;

#endif
