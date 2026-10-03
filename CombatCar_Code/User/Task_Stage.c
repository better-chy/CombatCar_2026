#include "Task_Stage.h"

#include <math.h>
#include <string.h>

#include "cmsis_os.h"
#include "JY901S.h"
#include "LaserRange.h"
#include "LiDAR.h"
#include "StageModel.h"
#include "Task_Main.h"
#include "Track_Enemy.h"

/* 任务周期与台上/台下切换 */
#define TASKSTAGE_TASK_PERIOD_MS                 5U  /* Stage 线程每轮判断周期。 */
#define TASKSTAGE_SWITCH_WINDOW_MS            1000U  /* 台上/台下二值切换使用 1s 滑动窗口。 */
#define TASKSTAGE_SWITCH_RATIO_PERCENT          80U  /* 1s 窗口内目标状态占比达到 80% 才确认切换。 */
#define TASKSTAGE_SWITCH_WINDOW_SAMPLES         (TASKSTAGE_SWITCH_WINDOW_MS / TASKSTAGE_TASK_PERIOD_MS) /* 滑动窗口采样数。 */
#define TASKSTAGE_SWITCH_CONFIRM_SAMPLES        ((TASKSTAGE_SWITCH_WINDOW_SAMPLES * TASKSTAGE_SWITCH_RATIO_PERCENT + 99U) / 100U) /* 切换确认所需采样数。 */

/* LiDAR 有效性与安装参数 */
#define TASKSTAGE_LIDAR_VALID_MIN_MM            20U  /* LiDAR 有效最小距离。 */
#define TASKSTAGE_LIDAR_VALID_MAX_MM          5500U  /* LiDAR 有效最大距离，覆盖 3.8m 场地对角线远墙回波。 */
#define TASKSTAGE_LIDAR_OFFSET_DEG            90.0f  /* 与 LiDAR.c 中安装偏置保持一致。 */
#define TASKSTAGE_FRONT_LIDAR_MIN_VALID_COUNT   55U  /* 前侧 71 点中至少需要的有效点数。 */
#define TASKSTAGE_SIDE_LIDAR_MIN_VALID_COUNT    50U  /* 左右侧 66 点中至少需要的有效点数。 */

/* LiDAR 快照来源 */
#define TASKSTAGE_LIDAR_SOURCE_READ             0U   /* 使用完整读缓冲。 */
#define TASKSTAGE_LIDAR_SOURCE_READ_LIVE_MERGE  1U   /* 读缓冲与实时缓冲合并。 */
#define TASKSTAGE_LIDAR_SOURCE_LIVE             2U   /* 只使用实时缓冲。 */
#define TASKSTAGE_LIDAR_SOURCE_MODE             TASKSTAGE_LIDAR_SOURCE_READ_LIVE_MERGE /* 当前 Stage 采样模式。 */

/* 姿态保护与重上台保护 */
#define TASKSTAGE_ENABLE_IMU_GUARD              0U   /* 未接 IMU 调试时关闭在线和横滚保护。 */
#define TASKSTAGE_LOCALIZER_ONLINE_TIMEOUT_MS 300U   /* IMU 在线判定超时。 */
#define TASKSTAGE_OFFSTAGE_ROLL_LIMIT_DEG      8.0f  /* 只有横滚角在该阈值内，才允许判台下。 */
#define TASKSTAGE_POST_REENTER_FREEZE_MS        0U   /* 重上台结束后不再强制保护为台上。 */

/* Debug 跳过原因 */
#define TASKSTAGE_DEBUG_SKIP_NONE               0U   /* 本轮正常运行模型。 */
#define TASKSTAGE_DEBUG_SKIP_INIT               1U   /* INIT 阶段强制认为在台上。 */
#define TASKSTAGE_DEBUG_SKIP_REENTER_FREEZE     2U   /* 重上台后保护期内强制认为在台上。 */
#define TASKSTAGE_DEBUG_SKIP_IMU_OFFLINE        3U   /* IMU 离线，跳过台下判断。 */
#define TASKSTAGE_DEBUG_SKIP_ROLL_LIMIT         4U   /* 横滚角过大，跳过台下判断。 */
#define TASKSTAGE_DEBUG_SKIP_ZERO_DATA          5U   /* 输入数据含 0，跳过模型判断。 */

/* 0 数据掩码 */
#define TASKSTAGE_ZERO_MASK_FRONT_LIDAR    0x0001U  /* 前方 LiDAR 特征含 0。 */
#define TASKSTAGE_ZERO_MASK_RIGHT_LIDAR    0x0002U  /* 右侧 LiDAR 特征含 0。 */
#define TASKSTAGE_ZERO_MASK_LEFT_LIDAR     0x0004U  /* 左侧 LiDAR 特征含 0。 */
#define TASKSTAGE_ZERO_MASK_LASER_FRONT    0x0010U  /* 前小激光测距为 0。 */
#define TASKSTAGE_ZERO_MASK_LASER_RIGHT    0x0020U  /* 右小激光测距为 0。 */
#define TASKSTAGE_ZERO_MASK_LASER_BACK     0x0040U  /* 后小激光测距为 0。 */
#define TASKSTAGE_ZERO_MASK_LASER_LEFT     0x0080U  /* 左小激光测距为 0。 */

/* 模型特征布局 */
#define TASKSTAGE_FRONT_CENTER_INDEX           35U  /* 前方宽扇区中心点下标。 */
#define TASKSTAGE_RIGHT_CENTER_INDEX           35U  /* 右侧宽扇区中心点下标。 */
#define TASKSTAGE_LEFT_CENTER_INDEX            30U  /* 左侧宽扇区中心点下标。 */
#define TASKSTAGE_SECTOR_SEGMENT_COUNT          5U  /* 宽特征中每个扇区分段数量。 */
#define TASKSTAGE_WIDE_MODEL_FEATURE_COUNT    109U  /* 宽特征模型输入总数。 */

#if STAGE_MODEL_FEATURE_COUNT != TASKSTAGE_WIDE_MODEL_FEATURE_COUNT
#error "Wide stage model must be trained with 109 features."
#endif

typedef struct
{
    uint16_t center;
    uint8_t valid_count;
    uint16_t min;
    uint16_t median;
    uint16_t p80;
    uint16_t p90;
    uint16_t max;
    uint8_t zero_count;
    uint16_t left_median;
    uint16_t right_median;
    uint8_t close_count_800;
} TaskStageModelSectorFeatures;

typedef struct
{
    uint8_t start_index;
    uint8_t count;
} TaskStageSegmentRange;

typedef struct
{
    uint16_t median;
    uint16_t p80;
    uint16_t min;
    uint16_t max;
} TaskStageModelSegmentFeatures;

static StageType TaskStage_JudgeType(void);
static void TaskStage_ApplyStateMachine(StageType new_type);
static void TaskStage_DebugClearPerCycle(void);
static uint8_t TaskStage_IsReenterFreezeActive(uint32_t now_tick);
static uint8_t TaskStage_IsOffStageType(StageType type);
static void TaskStage_ResetSwitchWindow(void);
static void TaskStage_UpdateSwitchWindow(StageType judged_type);
static void TaskStage_UpdateAppliedType(StageType judged_type);
static uint16_t TaskStage_MapBodyAngleToIndex(float angle_deg);
static uint8_t TaskStage_IsValidLidarDistance(uint16_t distance_mm);
#if TASKSTAGE_LIDAR_SOURCE_MODE == TASKSTAGE_LIDAR_SOURCE_READ_LIVE_MERGE
static void TaskStage_MergeLiveLidarSnapshot(void);
#endif
static uint8_t TaskStage_CountValidLidarValues(const uint16_t *values, uint8_t count);
static uint8_t TaskStage_SectorHasEnoughValid(const uint16_t *values,
                                              uint8_t count,
                                              uint8_t min_valid_count);
static uint16_t TaskStage_GetZeroDataMask(const TaskStageTrainFrame *frame);
static void TaskStage_BuildModelFeatures(const TaskStageTrainFrame *frame,
                                         int16_t features[STAGE_MODEL_FEATURE_COUNT]);
static uint8_t TaskStage_AppendSectorSegmentFeatures(int16_t features[STAGE_MODEL_FEATURE_COUNT],
                                                     uint8_t feature_index,
                                                     const uint16_t *values,
                                                     const TaskStageSegmentRange *segments);
static StageType TaskStage_MapModelLabelToStageType(uint8_t label);
static void TaskStage_UpdateDebugFromRawFrame(const TaskStageTrainFrame *frame);
static void TaskStage_UpdateDebugFromModelFrame(const TaskStageTrainFrame *frame,
                                                const int16_t features[STAGE_MODEL_FEATURE_COUNT],
                                                StageType result_type,
                                                uint8_t label);

volatile StageType g_stage_type = STAGE_ON_STAGE;
volatile StageDebugInfo g_stage_debug_info = {0};

static uint16_t s_stage_train_scan_distances[LIDAR_SCAN_POINT_COUNT];
#if TASKSTAGE_LIDAR_SOURCE_MODE == TASKSTAGE_LIDAR_SOURCE_READ_LIVE_MERGE
static uint16_t s_stage_live_scan_distances[LIDAR_SCAN_POINT_COUNT];
#endif
static StageType s_stage_candidate_type = STAGE_ON_STAGE;
static uint8_t s_stage_switch_window[TASKSTAGE_SWITCH_WINDOW_SAMPLES];
static uint16_t s_stage_switch_window_index = 0U;
static uint16_t s_stage_switch_window_count = 0U;
static uint16_t s_stage_switch_window_off_count = 0U;
static StageType s_stage_latest_off_type = STAGE_OFF_PARALLEL;
static StageType s_stage_last_valid_judged_type = STAGE_ON_STAGE;
static uint32_t s_stage_debug_loop_count = 0U;
static volatile uint32_t s_stage_reenter_freeze_until_tick = 0U;

static const TaskStageSegmentRange s_front_segment_ranges[TASKSTAGE_SECTOR_SEGMENT_COUNT] = {
    {0U, 14U},
    {14U, 14U},
    {28U, 15U},
    {43U, 14U},
    {57U, 14U},
};

static const TaskStageSegmentRange s_right_segment_ranges[TASKSTAGE_SECTOR_SEGMENT_COUNT] = {
    {0U, 13U},
    {13U, 13U},
    {26U, 14U},
    {40U, 13U},
    {53U, 13U},
};

static const TaskStageSegmentRange s_left_segment_ranges[TASKSTAGE_SECTOR_SEGMENT_COUNT] = {
    {0U, 13U},
    {13U, 13U},
    {26U, 14U},
    {40U, 13U},
    {53U, 13U},
};

static void TaskStage_DebugClearPerCycle(void)
{
    ++s_stage_debug_loop_count;
    memset((void *)&g_stage_debug_info, 0, sizeof(g_stage_debug_info));
    g_stage_debug_info.loop_count = s_stage_debug_loop_count;
    g_stage_debug_info.candidate_type = s_stage_candidate_type;
    g_stage_debug_info.applied_type = g_stage_type;
    g_stage_debug_info.judged_type = STAGE_ON_STAGE;
}

void TaskStage_NotifyReenterComplete(void)
{
    s_stage_reenter_freeze_until_tick =
        osKernelGetTickCount() + TASKSTAGE_POST_REENTER_FREEZE_MS;
}

static uint8_t TaskStage_IsReenterFreezeActive(uint32_t now_tick)
{
    uint32_t freeze_until_tick = s_stage_reenter_freeze_until_tick;

    if (freeze_until_tick == 0U)
    {
        return 0U;
    }

    if ((int32_t)(freeze_until_tick - now_tick) > 0)
    {
        return 1U;
    }

    s_stage_reenter_freeze_until_tick = 0U;
    return 0U;
}

static uint8_t TaskStage_IsOffStageType(StageType type)
{
    return (type == STAGE_ON_STAGE) ? 0U : 1U;
}

static void TaskStage_ResetSwitchWindow(void)
{
    memset(s_stage_switch_window, 0, sizeof(s_stage_switch_window));
    s_stage_switch_window_index = 0U;
    s_stage_switch_window_count = 0U;
    s_stage_switch_window_off_count = 0U;
    s_stage_latest_off_type = STAGE_OFF_PARALLEL;
    s_stage_last_valid_judged_type = STAGE_ON_STAGE;
}

static void TaskStage_UpdateSwitchWindow(StageType judged_type)
{
    uint8_t is_off_stage = TaskStage_IsOffStageType(judged_type);

    if (TaskStage_IsOffStageType(judged_type) != 0U)
    {
        s_stage_latest_off_type = judged_type;
    }

    if (s_stage_switch_window_count < TASKSTAGE_SWITCH_WINDOW_SAMPLES)
    {
        s_stage_switch_window[s_stage_switch_window_index] = is_off_stage;
        ++s_stage_switch_window_count;
        s_stage_switch_window_off_count += is_off_stage;
    }
    else
    {
        s_stage_switch_window_off_count -= s_stage_switch_window[s_stage_switch_window_index];
        s_stage_switch_window[s_stage_switch_window_index] = is_off_stage;
        s_stage_switch_window_off_count += is_off_stage;
    }

    ++s_stage_switch_window_index;
    if (s_stage_switch_window_index >= TASKSTAGE_SWITCH_WINDOW_SAMPLES)
    {
        s_stage_switch_window_index = 0U;
    }
}

static void TaskStage_UpdateAppliedType(StageType judged_type)
{
    uint16_t on_count;

    s_stage_candidate_type = judged_type;

    if (g_stage_type == STAGE_ON_STAGE)
    {
        if (s_stage_switch_window_count >= TASKSTAGE_SWITCH_WINDOW_SAMPLES &&
            s_stage_switch_window_off_count >= TASKSTAGE_SWITCH_CONFIRM_SAMPLES)
        {
            g_stage_type = s_stage_latest_off_type;
        }
        return;
    }

    on_count = (uint16_t)(s_stage_switch_window_count - s_stage_switch_window_off_count);
    if (s_stage_switch_window_count >= TASKSTAGE_SWITCH_WINDOW_SAMPLES &&
        on_count >= TASKSTAGE_SWITCH_CONFIRM_SAMPLES)
    {
        g_stage_type = STAGE_ON_STAGE;
        return;
    }

    if (judged_type != STAGE_ON_STAGE)
    {
        g_stage_type = judged_type;
    }
}

static void TaskStage_FillTrainSector(uint16_t *dest,
                                      const uint16_t *scan_distances,
                                      int16_t min_angle_deg,
                                      int16_t max_angle_deg)
{
    int16_t angle_deg;
    uint16_t out_index = 0U;

    if (dest == NULL || scan_distances == NULL)
    {
        return;
    }

    for (angle_deg = min_angle_deg; angle_deg <= max_angle_deg; ++angle_deg)
    {
        float sample_angle_deg = TrackEnemy_NormalizeAngleDeg((float)angle_deg);
        uint16_t sample_index = TaskStage_MapBodyAngleToIndex(sample_angle_deg);
        dest[out_index++] = scan_distances[sample_index];
    }
}

void TaskStage_CollectTrainFrame(TaskStageTrainFrame *frame_out)
{
    if (frame_out == NULL)
    {
        return;
    }

    memset(frame_out, 0, sizeof(*frame_out));
    frame_out->timestamp_ms = HAL_GetTick();

#if TASKSTAGE_LIDAR_SOURCE_MODE == TASKSTAGE_LIDAR_SOURCE_LIVE
    LiDAR_GetLiveDistanceArray(s_stage_train_scan_distances, LIDAR_SCAN_POINT_COUNT);
#else
    LiDAR_GetDistanceArray(s_stage_train_scan_distances, LIDAR_SCAN_POINT_COUNT);
#if TASKSTAGE_LIDAR_SOURCE_MODE == TASKSTAGE_LIDAR_SOURCE_READ_LIVE_MERGE
    TaskStage_MergeLiveLidarSnapshot();
#endif
#endif
    TrackEnemy_PatchZeroHoles(s_stage_train_scan_distances, LIDAR_SCAN_POINT_COUNT);

    TaskStage_FillTrainSector(frame_out->lidar_front_mm,
                              s_stage_train_scan_distances,
                              TASKSTAGE_FRONT_SECTOR_MIN_DEG,
                              TASKSTAGE_FRONT_SECTOR_MAX_DEG);
    TaskStage_FillTrainSector(frame_out->lidar_right_mm,
                              s_stage_train_scan_distances,
                              TASKSTAGE_RIGHT_SECTOR_MIN_DEG,
                              TASKSTAGE_RIGHT_SECTOR_MAX_DEG);
    TaskStage_FillTrainSector(frame_out->lidar_left_mm,
                              s_stage_train_scan_distances,
                              TASKSTAGE_LEFT_SECTOR_MIN_DEG,
                              TASKSTAGE_LEFT_SECTOR_MAX_DEG);

    LaserRange_GetAllDistances(frame_out->laser_mm, LASER_RANGE_SENSOR_COUNT);
}

static uint16_t TaskStage_MapBodyAngleToIndex(float angle_deg)
{
    uint16_t index;

    angle_deg += TASKSTAGE_LIDAR_OFFSET_DEG;

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

static uint8_t TaskStage_IsValidLidarDistance(uint16_t distance_mm)
{
    return (uint8_t)(distance_mm >= TASKSTAGE_LIDAR_VALID_MIN_MM &&
                     distance_mm <= TASKSTAGE_LIDAR_VALID_MAX_MM);
}

#if TASKSTAGE_LIDAR_SOURCE_MODE == TASKSTAGE_LIDAR_SOURCE_READ_LIVE_MERGE
static void TaskStage_MergeLiveLidarSnapshot(void)
{
    uint16_t i;

    LiDAR_GetLiveDistanceArray(s_stage_live_scan_distances, LIDAR_SCAN_POINT_COUNT);
    for (i = 0U; i < LIDAR_SCAN_POINT_COUNT; ++i)
    {
        if (TaskStage_IsValidLidarDistance(s_stage_live_scan_distances[i]) != 0U)
        {
            s_stage_train_scan_distances[i] = s_stage_live_scan_distances[i];
        }
    }
}
#endif

static uint8_t TaskStage_CountValidLidarValues(const uint16_t *values, uint8_t count)
{
    uint8_t i;
    uint8_t valid_count = 0U;

    if (values == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < count; ++i)
    {
        if (TaskStage_IsValidLidarDistance(values[i]) != 0U)
        {
            ++valid_count;
        }
    }

    return valid_count;
}

static uint8_t TaskStage_SectorHasEnoughValid(const uint16_t *values,
                                              uint8_t count,
                                              uint8_t min_valid_count)
{
    return (uint8_t)(TaskStage_CountValidLidarValues(values, count) >= min_valid_count);
}

static uint16_t TaskStage_GetZeroDataMask(const TaskStageTrainFrame *frame)
{
    uint16_t mask = 0U;

    if (frame == NULL)
    {
        return 0xFFFFU;
    }

    if (TaskStage_SectorHasEnoughValid(frame->lidar_front_mm,
                                       TASKSTAGE_FRONT_SECTOR_POINT_COUNT,
                                       TASKSTAGE_FRONT_LIDAR_MIN_VALID_COUNT) == 0U)
    {
        mask |= TASKSTAGE_ZERO_MASK_FRONT_LIDAR;
    }
    if (TaskStage_SectorHasEnoughValid(frame->lidar_right_mm,
                                       TASKSTAGE_RIGHT_SECTOR_POINT_COUNT,
                                       TASKSTAGE_SIDE_LIDAR_MIN_VALID_COUNT) == 0U)
    {
        mask |= TASKSTAGE_ZERO_MASK_RIGHT_LIDAR;
    }
    if (TaskStage_SectorHasEnoughValid(frame->lidar_left_mm,
                                       TASKSTAGE_LEFT_SECTOR_POINT_COUNT,
                                       TASKSTAGE_SIDE_LIDAR_MIN_VALID_COUNT) == 0U)
    {
        mask |= TASKSTAGE_ZERO_MASK_LEFT_LIDAR;
    }
    if (frame->laser_mm[LASER_FRONT] == 0U)
    {
        mask |= TASKSTAGE_ZERO_MASK_LASER_FRONT;
    }
    if (frame->laser_mm[LASER_RIGHT] == 0U)
    {
        mask |= TASKSTAGE_ZERO_MASK_LASER_RIGHT;
    }
    if (frame->laser_mm[LASER_BACK] == 0U)
    {
        mask |= TASKSTAGE_ZERO_MASK_LASER_BACK;
    }
    if (frame->laser_mm[LASER_LEFT] == 0U)
    {
        mask |= TASKSTAGE_ZERO_MASK_LASER_LEFT;
    }

    return mask;
}

static void TaskStage_SortU16(uint16_t *values, uint8_t count)
{
    uint8_t i;

    for (i = 1U; i < count; ++i)
    {
        uint16_t key = values[i];
        int16_t j = (int16_t)i - 1;

        while (j >= 0 && values[j] > key)
        {
            values[j + 1] = values[j];
            --j;
        }
        values[j + 1] = key;
    }
}

static uint8_t TaskStage_CopyValidLidarValues(const uint16_t *src,
                                              uint8_t count,
                                              uint16_t *dest)
{
    uint8_t i;
    uint8_t out_count = 0U;

    for (i = 0U; i < count; ++i)
    {
        if (TaskStage_IsValidLidarDistance(src[i]) != 0U)
        {
            dest[out_count++] = src[i];
        }
    }

    return out_count;
}

static uint16_t TaskStage_MedianSortedU16(const uint16_t *values, uint8_t count)
{
    if (count == 0U)
    {
        return 0U;
    }

    if ((count & 0x01U) != 0U)
    {
        return values[count / 2U];
    }

    return (uint16_t)(((uint32_t)values[(count / 2U) - 1U] + (uint32_t)values[count / 2U]) / 2U);
}

static uint16_t TaskStage_PercentileSortedU16(const uint16_t *values, uint8_t count, uint8_t pct)
{
    uint16_t scaled_index;
    uint16_t index;
    uint16_t remainder;

    if (count == 0U)
    {
        return 0U;
    }

    scaled_index = (uint16_t)((uint16_t)(count - 1U) * (uint16_t)pct);
    index = (uint16_t)(scaled_index / 100U);
    remainder = (uint16_t)(scaled_index % 100U);
    if (remainder > 50U || (remainder == 50U && (index & 0x01U) != 0U))
    {
        ++index;
    }
    if (index >= count)
    {
        index = (uint16_t)(count - 1U);
    }

    return values[index];
}

static uint16_t TaskStage_ValidMedian(const uint16_t *values, uint8_t count)
{
    uint16_t valid_values[TASKSTAGE_MAX_SECTOR_POINT_COUNT];
    uint8_t valid_count = TaskStage_CopyValidLidarValues(values, count, valid_values);

    TaskStage_SortU16(valid_values, valid_count);
    return TaskStage_MedianSortedU16(valid_values, valid_count);
}

static void TaskStage_GetModelSectorFeatures(const uint16_t *values,
                                             uint8_t count,
                                             uint8_t center_index,
                                             TaskStageModelSectorFeatures *features_out)
{
    uint16_t valid_values[TASKSTAGE_MAX_SECTOR_POINT_COUNT];
    uint8_t i;

    if (values == NULL || features_out == NULL || count == 0U || center_index >= count)
    {
        return;
    }

    memset(features_out, 0, sizeof(*features_out));
    features_out->center = values[center_index];

    for (i = 0U; i < count; ++i)
    {
        if (values[i] == 0U)
        {
            ++features_out->zero_count;
        }
        if (TaskStage_IsValidLidarDistance(values[i]) != 0U && values[i] <= 800U)
        {
            ++features_out->close_count_800;
        }
    }

    features_out->valid_count = TaskStage_CopyValidLidarValues(values,
                                                               count,
                                                               valid_values);
    if (features_out->valid_count > 0U)
    {
        TaskStage_SortU16(valid_values, features_out->valid_count);
        features_out->min = valid_values[0U];
        features_out->median = TaskStage_MedianSortedU16(valid_values, features_out->valid_count);
        features_out->p80 = TaskStage_PercentileSortedU16(valid_values, features_out->valid_count, 80U);
        features_out->p90 = TaskStage_PercentileSortedU16(valid_values, features_out->valid_count, 90U);
        features_out->max = valid_values[features_out->valid_count - 1U];
    }

    features_out->left_median = TaskStage_ValidMedian(values, center_index);
    features_out->right_median = TaskStage_ValidMedian(&values[center_index + 1U],
                                                       (uint8_t)(count - center_index - 1U));
}

static int16_t TaskStage_ToModelFeatureI16(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }

    return (int16_t)value;
}

static void TaskStage_SetModelFeature(int16_t features[STAGE_MODEL_FEATURE_COUNT],
                                      uint8_t index,
                                      int32_t value)
{
    if (features == NULL || index >= STAGE_MODEL_FEATURE_COUNT)
    {
        return;
    }

    features[index] = TaskStage_ToModelFeatureI16(value);
}

static void TaskStage_GetModelSegmentFeatures(const uint16_t *values,
                                              uint8_t count,
                                              TaskStageModelSegmentFeatures *features_out)
{
    uint16_t valid_values[TASKSTAGE_MAX_SECTOR_POINT_COUNT];
    uint8_t valid_count;

    if (values == NULL || features_out == NULL || count == 0U)
    {
        return;
    }

    memset(features_out, 0, sizeof(*features_out));

    valid_count = TaskStage_CopyValidLidarValues(values, count, valid_values);
    if (valid_count > 0U)
    {
        TaskStage_SortU16(valid_values, valid_count);
        features_out->median = TaskStage_MedianSortedU16(valid_values, valid_count);
        features_out->p80 = TaskStage_PercentileSortedU16(valid_values, valid_count, 80U);
        features_out->min = valid_values[0U];
        features_out->max = valid_values[valid_count - 1U];
    }
}

static uint8_t TaskStage_AppendSectorSegmentFeatures(int16_t features[STAGE_MODEL_FEATURE_COUNT],
                                                     uint8_t feature_index,
                                                     const uint16_t *values,
                                                     const TaskStageSegmentRange *segments)
{
    uint8_t i;

    if (features == NULL || values == NULL || segments == NULL)
    {
        return feature_index;
    }

    for (i = 0U; i < TASKSTAGE_SECTOR_SEGMENT_COUNT; ++i)
    {
        TaskStageModelSegmentFeatures segment = {0};
        TaskStage_GetModelSegmentFeatures(&values[segments[i].start_index],
                                          segments[i].count,
                                          &segment);
        TaskStage_SetModelFeature(features, feature_index++, segment.median);
        TaskStage_SetModelFeature(features, feature_index++, segment.p80);
        TaskStage_SetModelFeature(features, feature_index++, segment.min);
        TaskStage_SetModelFeature(features, feature_index++, segment.max);
    }

    return feature_index;
}

static void TaskStage_BuildModelFeatures(const TaskStageTrainFrame *frame,
                                         int16_t features[STAGE_MODEL_FEATURE_COUNT])
{
    TaskStageModelSectorFeatures front = {0};
    TaskStageModelSectorFeatures right = {0};
    TaskStageModelSectorFeatures left = {0};
    uint8_t feature_index;

    if (frame == NULL || features == NULL)
    {
        return;
    }

    TaskStage_GetModelSectorFeatures(frame->lidar_front_mm,
                                     TASKSTAGE_FRONT_SECTOR_POINT_COUNT,
                                     TASKSTAGE_FRONT_CENTER_INDEX,
                                     &front);
    TaskStage_GetModelSectorFeatures(frame->lidar_right_mm,
                                     TASKSTAGE_RIGHT_SECTOR_POINT_COUNT,
                                     TASKSTAGE_RIGHT_CENTER_INDEX,
                                     &right);
    TaskStage_GetModelSectorFeatures(frame->lidar_left_mm,
                                     TASKSTAGE_LEFT_SECTOR_POINT_COUNT,
                                     TASKSTAGE_LEFT_CENTER_INDEX,
                                     &left);

    features[0] = TaskStage_ToModelFeatureI16(front.center);
    features[1] = TaskStage_ToModelFeatureI16(front.valid_count);
    features[2] = TaskStage_ToModelFeatureI16(front.min);
    features[3] = TaskStage_ToModelFeatureI16(front.median);
    features[4] = TaskStage_ToModelFeatureI16(front.p80);
    features[5] = TaskStage_ToModelFeatureI16(front.p90);
    features[6] = TaskStage_ToModelFeatureI16(front.max);
    features[7] = TaskStage_ToModelFeatureI16(front.zero_count);
    features[8] = TaskStage_ToModelFeatureI16(front.left_median);
    features[9] = TaskStage_ToModelFeatureI16(front.right_median);

    features[10] = TaskStage_ToModelFeatureI16(right.center);
    features[11] = TaskStage_ToModelFeatureI16(right.valid_count);
    features[12] = TaskStage_ToModelFeatureI16(right.min);
    features[13] = TaskStage_ToModelFeatureI16(right.median);
    features[14] = TaskStage_ToModelFeatureI16(right.p80);
    features[15] = TaskStage_ToModelFeatureI16(right.p90);
    features[16] = TaskStage_ToModelFeatureI16(right.max);
    features[17] = TaskStage_ToModelFeatureI16(right.zero_count);
    features[18] = TaskStage_ToModelFeatureI16(right.left_median);
    features[19] = TaskStage_ToModelFeatureI16(right.right_median);

    features[20] = TaskStage_ToModelFeatureI16(left.center);
    features[21] = TaskStage_ToModelFeatureI16(left.valid_count);
    features[22] = TaskStage_ToModelFeatureI16(left.min);
    features[23] = TaskStage_ToModelFeatureI16(left.median);
    features[24] = TaskStage_ToModelFeatureI16(left.p80);
    features[25] = TaskStage_ToModelFeatureI16(left.p90);
    features[26] = TaskStage_ToModelFeatureI16(left.max);
    features[27] = TaskStage_ToModelFeatureI16(left.zero_count);
    features[28] = TaskStage_ToModelFeatureI16(left.left_median);
    features[29] = TaskStage_ToModelFeatureI16(left.right_median);

    features[30] = TaskStage_ToModelFeatureI16(frame->laser_mm[LASER_FRONT]);
    features[31] = TaskStage_ToModelFeatureI16(frame->laser_mm[LASER_RIGHT]);
    features[32] = TaskStage_ToModelFeatureI16(frame->laser_mm[LASER_BACK]);
    features[33] = TaskStage_ToModelFeatureI16(frame->laser_mm[LASER_LEFT]);

    features[34] = TaskStage_ToModelFeatureI16((int32_t)front.p80 - (int32_t)frame->laser_mm[LASER_FRONT]);
    features[35] = TaskStage_ToModelFeatureI16((int32_t)right.p80 - (int32_t)frame->laser_mm[LASER_RIGHT]);
    features[36] = TaskStage_ToModelFeatureI16((int32_t)left.p80 - (int32_t)frame->laser_mm[LASER_LEFT]);
    features[37] = TaskStage_ToModelFeatureI16((int32_t)front.median - (int32_t)frame->laser_mm[LASER_FRONT]);
    features[38] = TaskStage_ToModelFeatureI16((int32_t)right.median - (int32_t)frame->laser_mm[LASER_RIGHT]);
    features[39] = TaskStage_ToModelFeatureI16((int32_t)left.median - (int32_t)frame->laser_mm[LASER_LEFT]);
    features[40] = TaskStage_ToModelFeatureI16((int32_t)front.center - (int32_t)frame->laser_mm[LASER_FRONT]);
    features[41] = TaskStage_ToModelFeatureI16((int32_t)right.center - (int32_t)frame->laser_mm[LASER_RIGHT]);
    features[42] = TaskStage_ToModelFeatureI16((int32_t)left.center - (int32_t)frame->laser_mm[LASER_LEFT]);
    features[43] = TaskStage_ToModelFeatureI16((int32_t)front.min - (int32_t)frame->laser_mm[LASER_FRONT]);
    features[44] = TaskStage_ToModelFeatureI16((int32_t)right.min - (int32_t)frame->laser_mm[LASER_RIGHT]);
    features[45] = TaskStage_ToModelFeatureI16((int32_t)left.min - (int32_t)frame->laser_mm[LASER_LEFT]);
    features[46] = TaskStage_ToModelFeatureI16(front.close_count_800);
    features[47] = TaskStage_ToModelFeatureI16(right.close_count_800);
    features[48] = TaskStage_ToModelFeatureI16(left.close_count_800);

    feature_index = 49U;
    feature_index = TaskStage_AppendSectorSegmentFeatures(features,
                                                          feature_index,
                                                          frame->lidar_front_mm,
                                                          s_front_segment_ranges);
    feature_index = TaskStage_AppendSectorSegmentFeatures(features,
                                                          feature_index,
                                                          frame->lidar_right_mm,
                                                          s_right_segment_ranges);
    feature_index = TaskStage_AppendSectorSegmentFeatures(features,
                                                          feature_index,
                                                          frame->lidar_left_mm,
                                                          s_left_segment_ranges);
    (void)feature_index;
}

static StageType TaskStage_MapModelLabelToStageType(uint8_t label)
{
    switch (label)
    {
    case 0U:
        return STAGE_ON_STAGE;
    case 1U:
        return STAGE_OFF_PARALLEL;
    case 2U:
        return STAGE_OFF_CORNER_FRONT_LEFT;
    case 3U:
        return STAGE_OFF_CORNER_FRONT_RIGHT;
    case 4U:
        return STAGE_OFF_CORNER_REAR_LEFT;
    case 5U:
        return STAGE_OFF_CORNER_REAR_RIGHT;
    case 6U:
        return STAGE_OFF_CORNER_UNALIGNED;
    case 7U:
    case 8U:
        return STAGE_OFF_CORNER_UNALIGNED;
    case 9U:
        return STAGE_OFF_REENTER_READY;
    default:
        return STAGE_OFF_PARALLEL;
    }
}

static void TaskStage_UpdateDebugFromRawFrame(const TaskStageTrainFrame *frame)
{
    if (frame == NULL)
    {
        return;
    }

    g_stage_debug_info.lidar_front_mm = frame->lidar_front_mm[TASKSTAGE_FRONT_CENTER_INDEX];
    g_stage_debug_info.lidar_right_mm = frame->lidar_right_mm[TASKSTAGE_RIGHT_CENTER_INDEX];
    g_stage_debug_info.lidar_left_mm = frame->lidar_left_mm[TASKSTAGE_LEFT_CENTER_INDEX];
    g_stage_debug_info.laser_front_mm = frame->laser_mm[LASER_FRONT];
    g_stage_debug_info.laser_right_mm = frame->laser_mm[LASER_RIGHT];
    g_stage_debug_info.laser_back_mm = frame->laser_mm[LASER_BACK];
    g_stage_debug_info.laser_left_mm = frame->laser_mm[LASER_LEFT];
    g_stage_debug_info.lidar_front_valid_count =
        TaskStage_CountValidLidarValues(frame->lidar_front_mm, TASKSTAGE_FRONT_SECTOR_POINT_COUNT);
    g_stage_debug_info.lidar_right_valid_count =
        TaskStage_CountValidLidarValues(frame->lidar_right_mm, TASKSTAGE_RIGHT_SECTOR_POINT_COUNT);
    g_stage_debug_info.lidar_left_valid_count =
        TaskStage_CountValidLidarValues(frame->lidar_left_mm, TASKSTAGE_LEFT_SECTOR_POINT_COUNT);
}

static void TaskStage_UpdateDebugFromModelFrame(const TaskStageTrainFrame *frame,
                                                const int16_t features[STAGE_MODEL_FEATURE_COUNT],
                                                StageType result_type,
                                                uint8_t label)
{
    if (frame == NULL || features == NULL)
    {
        return;
    }

    g_stage_debug_info.lidar_front_mm = (uint16_t)features[0];
    g_stage_debug_info.lidar_right_mm = (uint16_t)features[10];
    g_stage_debug_info.lidar_left_mm = (uint16_t)features[20];
    g_stage_debug_info.laser_front_mm = frame->laser_mm[LASER_FRONT];
    g_stage_debug_info.laser_right_mm = frame->laser_mm[LASER_RIGHT];
    g_stage_debug_info.laser_back_mm = frame->laser_mm[LASER_BACK];
    g_stage_debug_info.laser_left_mm = frame->laser_mm[LASER_LEFT];
    g_stage_debug_info.model_label = label;
    g_stage_debug_info.model_feature_valid = 1U;
    g_stage_debug_info.judged_type = result_type;
}

static StageType TaskStage_JudgeType(void)
{
    TaskStageTrainFrame frame;
    int16_t features[STAGE_MODEL_FEATURE_COUNT] = {0};
    uint8_t label;
#if TASKSTAGE_ENABLE_IMU_GUARD != 0U
    uint8_t imu_online;
    float roll_deg;
#endif
    uint16_t zero_data_mask;
    StageType result_type;

    TaskStage_DebugClearPerCycle();

#if TASKSTAGE_ENABLE_IMU_GUARD != 0U
    imu_online = JY901S_IsOnline(TASKSTAGE_LOCALIZER_ONLINE_TIMEOUT_MS);
    g_stage_debug_info.imu_online = imu_online;
    if (imu_online == 0U)
    {
        g_stage_debug_info.skip_reason = TASKSTAGE_DEBUG_SKIP_IMU_OFFLINE;
        g_stage_debug_info.judged_type = s_stage_last_valid_judged_type;
        return s_stage_last_valid_judged_type;
    }

    roll_deg = JY901S_GetRollDeg();
    g_stage_debug_info.roll_deg_x10 = (int16_t)(roll_deg * 10.0f);
    if (fabsf(roll_deg) > TASKSTAGE_OFFSTAGE_ROLL_LIMIT_DEG)
    {
        g_stage_debug_info.skip_reason = TASKSTAGE_DEBUG_SKIP_ROLL_LIMIT;
        g_stage_debug_info.judged_type = s_stage_last_valid_judged_type;
        return s_stage_last_valid_judged_type;
    }
#else
    g_stage_debug_info.imu_online = 1U;
    g_stage_debug_info.roll_deg_x10 = 0;
#endif

    TaskStage_CollectTrainFrame(&frame);
    TaskStage_UpdateDebugFromRawFrame(&frame);
    zero_data_mask = TaskStage_GetZeroDataMask(&frame);
    g_stage_debug_info.zero_data_mask = zero_data_mask;
    if (zero_data_mask != 0U)
    {
        g_stage_debug_info.skip_reason = TASKSTAGE_DEBUG_SKIP_ZERO_DATA;
        g_stage_debug_info.judged_type = s_stage_last_valid_judged_type;
        return s_stage_last_valid_judged_type;
    }

    TaskStage_BuildModelFeatures(&frame, features);
    label = StageModel_Predict(features);
    result_type = TaskStage_MapModelLabelToStageType(label);
    s_stage_last_valid_judged_type = result_type;

    TaskStage_UpdateDebugFromModelFrame(&frame, features, result_type, label);

    return result_type;
}

static void TaskStage_ApplyStateMachine(StageType new_type)
{
    if (state_manager.current_state == STATE_INIT ||
        new_type == STAGE_ON_STAGE)
    {
        return;
    }

    TaskMain_SwitchState(STATE_OFF_STAGE_SEARCH);
}

void Task_Stage_Run(void *argument)
{
    uint32_t now_tick;
    StageType new_type;

    (void)argument;

    for (;;)
    {
        now_tick = osKernelGetTickCount();

        if (state_manager.current_state == STATE_INIT)
        {
            s_stage_candidate_type = STAGE_ON_STAGE;
            g_stage_type = STAGE_ON_STAGE;
            TaskStage_ResetSwitchWindow();
            TaskStage_DebugClearPerCycle();
            g_stage_debug_info.skip_reason = TASKSTAGE_DEBUG_SKIP_INIT;
            osDelay(TASKSTAGE_TASK_PERIOD_MS);
            continue;
        }

        if (TaskStage_IsReenterFreezeActive(now_tick) != 0U)
        {
            s_stage_candidate_type = STAGE_ON_STAGE;
            g_stage_type = STAGE_ON_STAGE;
            TaskStage_ResetSwitchWindow();
            TaskStage_DebugClearPerCycle();
            g_stage_debug_info.skip_reason = TASKSTAGE_DEBUG_SKIP_REENTER_FREEZE;
            osDelay(TASKSTAGE_TASK_PERIOD_MS);
            continue;
        }

        new_type = TaskStage_JudgeType();
        TaskStage_UpdateSwitchWindow(new_type);
        TaskStage_UpdateAppliedType(new_type);

        g_stage_debug_info.candidate_type = s_stage_candidate_type;
        g_stage_debug_info.applied_type = g_stage_type;

        if (state_manager.current_state != STATE_OFF_STAGE_SEARCH &&
            state_manager.current_state != STATE_REENTER_STAGE)
        {
            TaskStage_ApplyStateMachine(g_stage_type);
        }

        osDelay(TASKSTAGE_TASK_PERIOD_MS);
    }
}
