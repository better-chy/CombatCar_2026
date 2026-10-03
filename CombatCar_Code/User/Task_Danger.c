#include "Task_Danger.h"

#include "cmsis_os.h"
#include "CombatCar_TuneConfig.h"
#include "DangerModel.h"
#include "main.h"
#include "Motor.h"
#include "Task_Main.h"
#include "Task_Stage.h"
#include "Vision.h"
#include "JY901S.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

extern uint8_t TaskMain_IsEdgeProtectionDisabledByTilt(void);

/* 功能开关 */
#if COMBATCAR_FAST_PARAM_TUNE != 0U
#define TASKDANGER_USE_SLOWDOWN_MODEL          0U  /* 快速调参时跳过随机森林减速模型。 */
#else
#define TASKDANGER_USE_SLOWDOWN_MODEL          1U  /* 编译随机森林减速模型，是否运行由运行时策略决定。 */
#endif
#define TASKDANGER_USE_FAR_EDGE_SENSORS        1U  /* 启用远处二级光电；PATROL/TRACK 硬保护，ATTACK 软限速。 */

/* 任务周期与视觉策略 */
#define TASKDANGER_MODEL_UPDATE_PERIOD_MS     25U  /* 危险减速模型的最小更新周期，用于排查随机森林是否拖慢索敌。 */
#define TASKDANGER_VISION_TIMEOUT_MS         200U  /* 判断 AprilTag 仍在线的视觉超时。 */
#define TASKDANGER_APRILTAG_RECENT_MS        200U  /* 200ms 内出现过 AprilTag，都按 Tag 场景限速。 */

/* 减速限幅 */
#define TASKDANGER_FAST_CONFIRM_MS           500U  /* 从慢速恢复到快速前，模型必须连续判定安全的确认时间。 */
#define TASKDANGER_SLOWDOWN_TAG_LIMIT_PWM     200  /* 减速已激活且 200ms 内出现过 AprilTag 时，限速压到 200。 */
#define TASKDANGER_SLOWDOWN_NO_TAG_LIMIT_PWM  200  /* 减速已激活但没看到 AprilTag 时，也限速压到 200。 */
#define TASKDANGER_FAR_EDGE_SOFT_LIMIT_PWM    220  /* ATTACK 中二级光电触发时只做软限速，不直接脱困。 */
#define TASKDANGER_ATTACK_APRILTAG_LIMIT_PWM  200  /* ATTACK 中 200ms 内出现过 AprilTag 时，直接限制前进分量。 */

/* LiDAR 有效性 */
#define TASKDANGER_LIDAR_VALID_MIN_MM         20U  /* LiDAR 统计特征允许的最小有效距离。 */
#define TASKDANGER_LIDAR_VALID_MAX_MM       5500U  /* LiDAR 统计特征允许的最大有效距离。 */

/* 模型特征布局 */
#define TASKDANGER_FRONT_CENTER_INDEX         35U  /* 与 StageModel 保持一致，前方扇区中心点在 -35..+35 的第 35 点。 */
#define TASKDANGER_RIGHT_CENTER_INDEX         35U  /* 与 StageModel 保持一致，右侧扇区中心点在 55..120 的第 35 点。 */
#define TASKDANGER_LEFT_CENTER_INDEX          30U  /* 与 StageModel 保持一致，左侧扇区中心点在 -120..-55 的第 30 点。 */
#define TASKDANGER_SECTOR_SEGMENT_COUNT        5U  /* 与 StageModel 保持一致，每个扇区切成 5 段统计局部形状。 */
#define TASKDANGER_CLOSE_COUNT_DISTANCE_MM   800U  /* 近距离点数量阈值，用来识别局部被敌车或 Tag 挡住的情况。 */

#if DANGER_MODEL_FEATURE_COUNT != 109U
#error "Danger model must be trained with the same 109 wide features as StageModel."
#endif

typedef struct
{
    uint16_t center; /* 扇区中心射线距离。 */
    uint8_t valid_count; /* 扇区有效 LiDAR 点数。 */
    uint16_t min; /* 扇区最近有效距离。 */
    uint16_t median; /* 扇区中位数距离。 */
    uint16_t p80; /* 扇区 80 百分位距离。 */
    uint16_t p90; /* 扇区 90 百分位距离。 */
    uint16_t max; /* 扇区最远有效距离。 */
    uint8_t zero_count; /* 扇区内 0 点数量。 */
    uint16_t left_median; /* 中心点左半区中位数。 */
    uint16_t right_median; /* 中心点右半区中位数。 */
    uint8_t close_count_800; /* 800mm 内近点数量。 */
} TaskDangerModelSectorFeatures;

typedef struct
{
    uint8_t start_index; /* 分段起始点。 */
    uint8_t count; /* 分段点数。 */
} TaskDangerSegmentRange;

typedef struct
{
    uint16_t median; /* 分段中位数。 */
    uint16_t p80; /* 分段 80 百分位。 */
    uint16_t min; /* 分段最近距离。 */
    uint16_t max; /* 分段最远距离。 */
} TaskDangerModelSegmentFeatures;

/* 左侧近处硬件边缘传感器当前电平，0 表示车轮附近已经探不到台面。 */
static int left_danger = 1;
#if TASKDANGER_USE_FAR_EDGE_SENSORS != 0U
/* 左侧远处硬件边缘传感器当前电平，1 表示更前方已经探不到台面。 */
static int left_danger_2 = 0;
#endif
/* 右侧近处硬件边缘传感器当前电平，0 表示车轮附近已经探不到台面。 */
static int right_danger = 1;
#if TASKDANGER_USE_FAR_EDGE_SENSORS != 0U
/* 右侧远处硬件边缘传感器当前电平，1 表示更前方已经探不到台面。 */
static int right_danger_2 = 0;
#endif

/* 当前硬件边缘触发类型，供主状态机 EDGE_ESCAPE 决定脱困动作。 */
EdgeDangerType edge_danger_type = EDGE_NONE;
/* 危险减速是否处于激活状态，电机输出会读取该变量做限速。 */
volatile uint8_t g_danger_slowdown_active = 0U;
/* TRACK/ATTACK 中二级光电触发的软限速状态，供 debug 观察。 */
volatile uint8_t g_danger_far_edge_soft_slowdown_active = 0U;
/* 危险随机森林最近一次输出的标签，0 为安全，1 为减速。 */
volatile uint8_t g_danger_model_label = DANGER_MODEL_LABEL_0;
/* DangerTask 当前剩余栈空间，单位为 FreeRTOS stack word，供 debug 观察。 */
volatile uint32_t g_danger_stack_free_words = 0U;
/* 临时屏蔽光电边缘保护的截止 tick，0 表示不屏蔽。 */
static volatile uint32_t s_edge_protection_suppressed_until_tick = 0U;
/* 最近一次看到 AprilTag 的 tick，用于给视觉短丢帧留 200ms 保活。 */
static uint32_t s_apriltag_last_detect_tick = 0U;

#if TASKDANGER_USE_SLOWDOWN_MODEL != 0U
/* 危险模型上一次实际运行的系统 tick。 */
static uint32_t s_danger_model_last_update_tick = 0U;
/* 慢速恢复快速的候选起始 tick，0 表示尚未进入恢复确认。 */
static uint32_t s_danger_fast_candidate_tick = 0U;
/* 危险模型复用的传感器训练帧缓存，避免放在线程栈上。 */
static TaskStageTrainFrame s_danger_model_frame;
/* 危险模型输入特征缓存。 */
static int16_t s_danger_model_features[DANGER_MODEL_FEATURE_COUNT];
/* 扇区统计时复用的排序缓存。 */
static uint16_t s_danger_sort_buffer[TASKSTAGE_MAX_SECTOR_POINT_COUNT];
#endif

#if TASKDANGER_USE_SLOWDOWN_MODEL != 0U
static uint8_t TaskDanger_IsValidLidarDistance(uint16_t distance_mm);
static uint8_t TaskDanger_FillSortedValid(const uint16_t *values,
                                          uint8_t count,
                                          uint16_t *sorted);
static uint16_t TaskDanger_GetMedian(const uint16_t *sorted_values, uint8_t count);
static uint16_t TaskDanger_GetPercentile(const uint16_t *sorted_values,
                                         uint8_t count,
                                         uint8_t percentile);
static uint16_t TaskDanger_GetValidMedian(const uint16_t *values, uint8_t count);
static int16_t TaskDanger_ToModelFeatureI16(int32_t value);
static void TaskDanger_SetModelFeature(int16_t features[DANGER_MODEL_FEATURE_COUNT],
                                       uint8_t index,
                                       int32_t value);
static void TaskDanger_GetModelSectorFeatures(const uint16_t *values,
                                              uint8_t count,
                                              uint8_t center_index,
                                              TaskDangerModelSectorFeatures *features_out);
static void TaskDanger_GetModelSegmentFeatures(const uint16_t *values,
                                               uint8_t count,
                                               TaskDangerModelSegmentFeatures *features_out);
static uint8_t TaskDanger_AppendSectorSegmentFeatures(int16_t features[DANGER_MODEL_FEATURE_COUNT],
                                                      uint8_t feature_index,
                                                      const uint16_t *values,
                                                      const TaskDangerSegmentRange *segments);
static void TaskDanger_BuildModelFeatures(const TaskStageTrainFrame *frame,
                                          int16_t features[DANGER_MODEL_FEATURE_COUNT]);
#endif
static void TaskDanger_UpdateSlowdownModel(uint32_t now_tick);
static uint8_t TaskDanger_IsAprilTagDetected(void);
static uint8_t TaskDanger_ShouldUseHardwareEdge(void);
static uint8_t TaskDanger_IsLeftEdgeDetected(void);
static uint8_t TaskDanger_IsRightEdgeDetected(void);

#if TASKDANGER_USE_SLOWDOWN_MODEL != 0U
static const TaskDangerSegmentRange s_danger_front_segment_ranges[TASKDANGER_SECTOR_SEGMENT_COUNT] = {
    {0U, 14U},
    {14U, 14U},
    {28U, 15U},
    {43U, 14U},
    {57U, 14U},
};

static const TaskDangerSegmentRange s_danger_right_segment_ranges[TASKDANGER_SECTOR_SEGMENT_COUNT] = {
    {0U, 13U},
    {13U, 13U},
    {26U, 14U},
    {40U, 13U},
    {53U, 13U},
};

static const TaskDangerSegmentRange s_danger_left_segment_ranges[TASKDANGER_SECTOR_SEGMENT_COUNT] = {
    {0U, 13U},
    {13U, 13U},
    {26U, 14U},
    {40U, 13U},
    {53U, 13U},
};
#endif

/* 返回当前危险减速是否激活，供其他任务或调试窗口读取。 */
uint8_t TaskDanger_IsSlowdownActive(void)
{
#if TASKDANGER_USE_SLOWDOWN_MODEL != 0U
    return g_danger_slowdown_active;
#else
    return 0U;
#endif
}

/* 视觉上看到 AprilTag 时，认为当前更可能在攻击/处理能量块。 */
static uint8_t TaskDanger_IsAprilTagDetected(void)
{
    Vision_Tag tag;
    uint32_t now_tick = osKernelGetTickCount();

    if (Vision_GetLatestTag(&tag) != 0U &&
        (tag.flags & VISION_FLAG_DETECTED) != 0U &&
        (uint32_t)(now_tick - tag.update_tick) <= TASKDANGER_APRILTAG_RECENT_MS)
    {
        s_apriltag_last_detect_tick = tag.update_tick;
    }

    if (s_apriltag_last_detect_tick != 0U &&
        (uint32_t)(now_tick - s_apriltag_last_detect_tick) <= TASKDANGER_APRILTAG_RECENT_MS)
    {
        return 1U;
    }

    return 0U;
}

/* 随机森林减速在台上常规运动状态启用；Tag/普通危险都限到 200。 */
uint8_t TaskDanger_ShouldUseSlowdownProtection(void)
{
#if TASKDANGER_USE_SLOWDOWN_MODEL == 0U
    return 0U;
#else
    return (state_manager.current_state == STATE_PATROL ||
            state_manager.current_state == STATE_TRACK ||
            state_manager.current_state == STATE_ATTACK ||
            state_manager.current_state == STATE_AVOID_FRIENDLY_BLOCK) ? 1U : 0U;
#endif
}

/* 二级光电：
 * 1. PATROL/TRACK 中启用远处光电硬保护，提前防掉台；
 * 2. ATTACK 无 AprilTag 时远处光电只做软限速，不直接触发 EDGE_ESCAPE；
 * 3. ATTACK 有 AprilTag 时远处光电恢复硬保护，触发后直接避险后退；
 * 4. AVOID_FRIENDLY_BLOCK 一定是在处理己方 Tag，固定启用远处光电。 */
uint8_t TaskDanger_ShouldUseFarEdgeProtection(void)
{
    if (TaskDanger_IsEdgeProtectionSuppressed() != 0U ||
        TaskMain_IsEdgeProtectionDisabledByTilt() != 0U)
    {
        return 0U;
    }

    if (state_manager.current_state == STATE_PATROL ||
        state_manager.current_state == STATE_TRACK)
    {
        return 1U;
    }

    if (state_manager.current_state == STATE_AVOID_FRIENDLY_BLOCK)
    {
        return 1U;
    }

    if (state_manager.current_state == STATE_ATTACK &&
        TaskDanger_IsAprilTagDetected() != 0U)
    {
        return 1U;
    }

    return 0U;
}

void TaskDanger_SuppressEdgeProtection(uint32_t duration_ms)
{
    if (duration_ms == 0U)
    {
        s_edge_protection_suppressed_until_tick = 0U;
        return;
    }

    s_edge_protection_suppressed_until_tick = osKernelGetTickCount() + duration_ms;
}

uint8_t TaskDanger_IsEdgeProtectionSuppressed(void)
{
    uint32_t until_tick = s_edge_protection_suppressed_until_tick;
    uint32_t now_tick;

    if (until_tick == 0U)
    {
        return 0U;
    }

    now_tick = osKernelGetTickCount();
    if ((uint32_t)(now_tick - until_tick) < 0x80000000U)
    {
        s_edge_protection_suppressed_until_tick = 0U;
        return 0U;
    }

    return 1U;
}

/* ATTACK 无 AprilTag 时二级光电只作为软限速信号；有 Tag 时改由硬保护处理。 */
static uint8_t TaskDanger_IsFarEdgeSoftSlowdownActive(void)
{
#if TASKDANGER_USE_FAR_EDGE_SENSORS == 0U
    return 0U;
#else
    if (TaskDanger_IsEdgeProtectionSuppressed() != 0U ||
        TaskMain_IsEdgeProtectionDisabledByTilt() != 0U)
    {
        return 0U;
    }

    if (state_manager.current_state != STATE_ATTACK)
    {
        return 0U;
    }

    if (TaskDanger_IsAprilTagDetected() != 0U)
    {
        return 0U;
    }

    return (left_danger_2 != 0 || right_danger_2 != 0) ? 1U : 0U;
#endif
}

/* 对单个 PWM 做危险限速；当前工程主要使用双轮版本，该函数保留给单轮调用。 */
int TaskDanger_ApplySpeedLimit(int pwm)
{
#if TASKDANGER_USE_SLOWDOWN_MODEL == 0U
    return pwm;
#else
    int abs_pwm; /* 当前单轮 PWM 的绝对值。 */
    int limit_pwm; /* 当前场景允许的最大 PWM。 */
    uint8_t limit_active = 0U; /* 当前是否需要限速。 */
    uint8_t apriltag_detected = TaskDanger_IsAprilTagDetected(); /* 200ms 内是否出现过 Tag。 */

    if (g_danger_slowdown_active != 0U)
    {
        limit_pwm = apriltag_detected != 0U ? TASKDANGER_SLOWDOWN_TAG_LIMIT_PWM : TASKDANGER_SLOWDOWN_NO_TAG_LIMIT_PWM;
        limit_active = 1U;
    }

    if (state_manager.current_state == STATE_ATTACK &&
        apriltag_detected != 0U &&
        (limit_active == 0U || TASKDANGER_ATTACK_APRILTAG_LIMIT_PWM < limit_pwm))
    {
        limit_pwm = TASKDANGER_ATTACK_APRILTAG_LIMIT_PWM;
        limit_active = 1U;
    }

    if (limit_active == 0U)
    {
        return pwm;
    }

    abs_pwm = (pwm < 0) ? -pwm : pwm;
    if (abs_pwm <= limit_pwm)
    {
        return pwm;
    }

    if (pwm > 0)
    {
        return limit_pwm;
    }

    if (pwm < 0)
    {
        return -limit_pwm;
    }

    return pwm;
#endif
}

/* 对左右轮 PWM 做危险限速，只限制高速前进分量，尽量保留转向分量。 */
void TaskDanger_ApplySpeedLimitPair(int *left_pwm, int *right_pwm)
{
    int forward_pwm; /* 左右轮平均值，代表整车前进分量。 */
    int turn_pwm; /* 左右轮差值的一半，代表转向分量。 */
    int limit_pwm = 0; /* 当前场景允许的最大前进分量。 */
    uint8_t limit_active = 0U; /* 当前是否需要限速。 */
    uint8_t apriltag_detected; /* 1.2s 内是否出现过 Tag。 */

    if (left_pwm == NULL || right_pwm == NULL)
    {
        return;
    }

    apriltag_detected = TaskDanger_IsAprilTagDetected();
#if TASKDANGER_USE_SLOWDOWN_MODEL != 0U
    if (g_danger_slowdown_active != 0U)
    {
        limit_pwm = apriltag_detected != 0U ? TASKDANGER_SLOWDOWN_TAG_LIMIT_PWM : TASKDANGER_SLOWDOWN_NO_TAG_LIMIT_PWM;
        limit_active = 1U;
    }
#endif

    if (state_manager.current_state == STATE_ATTACK &&
        apriltag_detected != 0U &&
        (limit_active == 0U || TASKDANGER_ATTACK_APRILTAG_LIMIT_PWM < limit_pwm))
    {
        limit_pwm = TASKDANGER_ATTACK_APRILTAG_LIMIT_PWM;
        limit_active = 1U;
    }

    g_danger_far_edge_soft_slowdown_active = TaskDanger_IsFarEdgeSoftSlowdownActive();
    if (g_danger_far_edge_soft_slowdown_active != 0U &&
        (limit_active == 0U || TASKDANGER_FAR_EDGE_SOFT_LIMIT_PWM < limit_pwm))
    {
        limit_pwm = TASKDANGER_FAR_EDGE_SOFT_LIMIT_PWM;
        limit_active = 1U;
    }

    if (limit_active == 0U)
    {
        return;
    }

    if (*left_pwm <= 0 && *right_pwm <= 0)
    {
        return;
    }

    forward_pwm = (*left_pwm + *right_pwm) / 2;
    if (forward_pwm <= limit_pwm)
    {
        return;
    }

    turn_pwm = (*right_pwm - *left_pwm) / 2;
    forward_pwm = limit_pwm;

    *left_pwm = forward_pwm - turn_pwm;
    *right_pwm = forward_pwm + turn_pwm;

    if (*left_pwm < 0)
    {
        *left_pwm = 0;
    }

    if (*right_pwm < 0)
    {
        *right_pwm = 0;
    }
}

#if TASKDANGER_USE_SLOWDOWN_MODEL != 0U
/* 判断 LiDAR 距离是否能参与危险模型的扇区统计。 */
static uint8_t TaskDanger_IsValidLidarDistance(uint16_t distance_mm)
{
    return (distance_mm >= TASKDANGER_LIDAR_VALID_MIN_MM &&
            distance_mm <= TASKDANGER_LIDAR_VALID_MAX_MM) ? 1U : 0U;
}

/* 从一组 LiDAR 点中筛出有效距离，并按从小到大插入排序。 */
static uint8_t TaskDanger_FillSortedValid(const uint16_t *values,
                                          uint8_t count,
                                          uint16_t *sorted)
{
    uint8_t valid_count = 0U; /* 已写入 sorted 的有效点数量。 */
    uint8_t i; /* 当前遍历的原始点索引。 */

    if (values == NULL || sorted == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < count; ++i)
    {
        uint16_t value = values[i]; /* 当前检查的 LiDAR 距离。 */

        if (TaskDanger_IsValidLidarDistance(value) != 0U)
        {
            uint8_t insert = valid_count; /* 当前有效值应插入到 sorted 的位置。 */

            while (insert > 0U && sorted[insert - 1U] > value)
            {
                sorted[insert] = sorted[insert - 1U];
                --insert;
            }

            sorted[insert] = value;
            ++valid_count;
        }
    }

    return valid_count;
}

/* 获取已排序数组的中位数；count 为 0 时返回 0。 */
static uint16_t TaskDanger_GetMedian(const uint16_t *sorted_values, uint8_t count)
{
    if (sorted_values == NULL || count == 0U)
    {
        return 0U;
    }

    return sorted_values[count / 2U];
}

/* 获取已排序数组的指定百分位值，例如 p80/p90。 */
static uint16_t TaskDanger_GetPercentile(const uint16_t *sorted_values,
                                         uint8_t count,
                                         uint8_t percentile)
{
    uint16_t index; /* 百分位对应的数组下标。 */

    if (sorted_values == NULL || count == 0U)
    {
        return 0U;
    }

    index = (uint16_t)((((uint16_t)count - 1U) * (uint16_t)percentile + 50U) / 100U);
    if (index >= count)
    {
        index = (uint16_t)count - 1U;
    }

    return sorted_values[index];
}

/* 计算一段原始 LiDAR 点的有效距离中位数。 */
static uint16_t TaskDanger_GetValidMedian(const uint16_t *values, uint8_t count)
{
    uint8_t valid_count; /* 当前窗口有效点数。 */

    if (values == NULL || count == 0U)
    {
        return 0U;
    }

    valid_count = TaskDanger_FillSortedValid(values, count, s_danger_sort_buffer);
    return TaskDanger_GetMedian(s_danger_sort_buffer, valid_count);
}

/* 将 int32 特征限幅到模型使用的 int16 范围。 */
static int16_t TaskDanger_ToModelFeatureI16(int32_t value)
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

/* 安全写入模型特征数组，避免新增特征时数组越界。 */
static void TaskDanger_SetModelFeature(int16_t features[DANGER_MODEL_FEATURE_COUNT],
                                       uint8_t index,
                                       int32_t value)
{
    if (features == NULL || index >= DANGER_MODEL_FEATURE_COUNT)
    {
        return;
    }

    features[index] = TaskDanger_ToModelFeatureI16(value);
}

/* 计算与 StageModel 完全一致的单个 LiDAR 扇区统计。 */
static void TaskDanger_GetModelSectorFeatures(const uint16_t *values,
                                              uint8_t count,
                                              uint8_t center_index,
                                              TaskDangerModelSectorFeatures *features_out)
{
    uint8_t i; /* 当前遍历的扇区点索引。 */

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
        if (TaskDanger_IsValidLidarDistance(values[i]) != 0U && values[i] <= TASKDANGER_CLOSE_COUNT_DISTANCE_MM)
        {
            ++features_out->close_count_800;
        }
    }

    features_out->valid_count = TaskDanger_FillSortedValid(values, count, s_danger_sort_buffer);
    if (features_out->valid_count > 0U)
    {
        features_out->min = s_danger_sort_buffer[0U];
        features_out->median = TaskDanger_GetMedian(s_danger_sort_buffer, features_out->valid_count);
        features_out->p80 = TaskDanger_GetPercentile(s_danger_sort_buffer, features_out->valid_count, 80U);
        features_out->p90 = TaskDanger_GetPercentile(s_danger_sort_buffer, features_out->valid_count, 90U);
        features_out->max = s_danger_sort_buffer[features_out->valid_count - 1U];
    }

    features_out->left_median = TaskDanger_GetValidMedian(values, center_index);
    features_out->right_median = TaskDanger_GetValidMedian(&values[center_index + 1U],
                                                           (uint8_t)(count - center_index - 1U));
}

/* 计算一个局部小扇区的 median/p80/min/max。 */
static void TaskDanger_GetModelSegmentFeatures(const uint16_t *values,
                                               uint8_t count,
                                               TaskDangerModelSegmentFeatures *features_out)
{
    uint8_t valid_count; /* 当前小扇区有效点数。 */

    if (values == NULL || features_out == NULL || count == 0U)
    {
        return;
    }

    memset(features_out, 0, sizeof(*features_out));
    valid_count = TaskDanger_FillSortedValid(values, count, s_danger_sort_buffer);
    if (valid_count > 0U)
    {
        features_out->median = TaskDanger_GetMedian(s_danger_sort_buffer, valid_count);
        features_out->p80 = TaskDanger_GetPercentile(s_danger_sort_buffer, valid_count, 80U);
        features_out->min = s_danger_sort_buffer[0U];
        features_out->max = s_danger_sort_buffer[valid_count - 1U];
    }
}

/* 追加 5 段局部扇区统计，返回下一个可写特征下标。 */
static uint8_t TaskDanger_AppendSectorSegmentFeatures(int16_t features[DANGER_MODEL_FEATURE_COUNT],
                                                      uint8_t feature_index,
                                                      const uint16_t *values,
                                                      const TaskDangerSegmentRange *segments)
{
    uint8_t i; /* 当前分段索引。 */

    if (features == NULL || values == NULL || segments == NULL)
    {
        return feature_index;
    }

    for (i = 0U; i < TASKDANGER_SECTOR_SEGMENT_COUNT; ++i)
    {
        TaskDangerModelSegmentFeatures segment = {0};
        TaskDanger_GetModelSegmentFeatures(&values[segments[i].start_index],
                                           segments[i].count,
                                           &segment);
        TaskDanger_SetModelFeature(features, feature_index++, segment.median);
        TaskDanger_SetModelFeature(features, feature_index++, segment.p80);
        TaskDanger_SetModelFeature(features, feature_index++, segment.min);
        TaskDanger_SetModelFeature(features, feature_index++, segment.max);
    }

    return feature_index;
}

/* 将一帧原始传感器数据转换成 DangerModel 需要的 109 个 int16 特征。 */
static void TaskDanger_BuildModelFeatures(const TaskStageTrainFrame *frame,
                                          int16_t features[DANGER_MODEL_FEATURE_COUNT])
{
    TaskDangerModelSectorFeatures front = {0}; /* 前方扇区统计。 */
    TaskDangerModelSectorFeatures right = {0}; /* 右侧扇区统计。 */
    TaskDangerModelSectorFeatures left = {0}; /* 左侧扇区统计。 */
    uint8_t feature_index; /* 当前追加分段特征的位置。 */

    if (frame == NULL || features == NULL)
    {
        return;
    }

    TaskDanger_GetModelSectorFeatures(frame->lidar_front_mm,
                                      TASKSTAGE_FRONT_SECTOR_POINT_COUNT,
                                      TASKDANGER_FRONT_CENTER_INDEX,
                                      &front);
    TaskDanger_GetModelSectorFeatures(frame->lidar_right_mm,
                                      TASKSTAGE_RIGHT_SECTOR_POINT_COUNT,
                                      TASKDANGER_RIGHT_CENTER_INDEX,
                                      &right);
    TaskDanger_GetModelSectorFeatures(frame->lidar_left_mm,
                                      TASKSTAGE_LEFT_SECTOR_POINT_COUNT,
                                      TASKDANGER_LEFT_CENTER_INDEX,
                                      &left);

    features[0] = TaskDanger_ToModelFeatureI16(front.center);
    features[1] = TaskDanger_ToModelFeatureI16(front.valid_count);
    features[2] = TaskDanger_ToModelFeatureI16(front.min);
    features[3] = TaskDanger_ToModelFeatureI16(front.median);
    features[4] = TaskDanger_ToModelFeatureI16(front.p80);
    features[5] = TaskDanger_ToModelFeatureI16(front.p90);
    features[6] = TaskDanger_ToModelFeatureI16(front.max);
    features[7] = TaskDanger_ToModelFeatureI16(front.zero_count);
    features[8] = TaskDanger_ToModelFeatureI16(front.left_median);
    features[9] = TaskDanger_ToModelFeatureI16(front.right_median);

    features[10] = TaskDanger_ToModelFeatureI16(right.center);
    features[11] = TaskDanger_ToModelFeatureI16(right.valid_count);
    features[12] = TaskDanger_ToModelFeatureI16(right.min);
    features[13] = TaskDanger_ToModelFeatureI16(right.median);
    features[14] = TaskDanger_ToModelFeatureI16(right.p80);
    features[15] = TaskDanger_ToModelFeatureI16(right.p90);
    features[16] = TaskDanger_ToModelFeatureI16(right.max);
    features[17] = TaskDanger_ToModelFeatureI16(right.zero_count);
    features[18] = TaskDanger_ToModelFeatureI16(right.left_median);
    features[19] = TaskDanger_ToModelFeatureI16(right.right_median);

    features[20] = TaskDanger_ToModelFeatureI16(left.center);
    features[21] = TaskDanger_ToModelFeatureI16(left.valid_count);
    features[22] = TaskDanger_ToModelFeatureI16(left.min);
    features[23] = TaskDanger_ToModelFeatureI16(left.median);
    features[24] = TaskDanger_ToModelFeatureI16(left.p80);
    features[25] = TaskDanger_ToModelFeatureI16(left.p90);
    features[26] = TaskDanger_ToModelFeatureI16(left.max);
    features[27] = TaskDanger_ToModelFeatureI16(left.zero_count);
    features[28] = TaskDanger_ToModelFeatureI16(left.left_median);
    features[29] = TaskDanger_ToModelFeatureI16(left.right_median);

    features[30] = TaskDanger_ToModelFeatureI16(frame->laser_mm[LASER_FRONT]);
    features[31] = TaskDanger_ToModelFeatureI16(frame->laser_mm[LASER_RIGHT]);
    features[32] = TaskDanger_ToModelFeatureI16(frame->laser_mm[LASER_BACK]);
    features[33] = TaskDanger_ToModelFeatureI16(frame->laser_mm[LASER_LEFT]);

    features[34] = TaskDanger_ToModelFeatureI16((int32_t)front.p80 - (int32_t)frame->laser_mm[LASER_FRONT]);
    features[35] = TaskDanger_ToModelFeatureI16((int32_t)right.p80 - (int32_t)frame->laser_mm[LASER_RIGHT]);
    features[36] = TaskDanger_ToModelFeatureI16((int32_t)left.p80 - (int32_t)frame->laser_mm[LASER_LEFT]);
    features[37] = TaskDanger_ToModelFeatureI16((int32_t)front.median - (int32_t)frame->laser_mm[LASER_FRONT]);
    features[38] = TaskDanger_ToModelFeatureI16((int32_t)right.median - (int32_t)frame->laser_mm[LASER_RIGHT]);
    features[39] = TaskDanger_ToModelFeatureI16((int32_t)left.median - (int32_t)frame->laser_mm[LASER_LEFT]);
    features[40] = TaskDanger_ToModelFeatureI16((int32_t)front.center - (int32_t)frame->laser_mm[LASER_FRONT]);
    features[41] = TaskDanger_ToModelFeatureI16((int32_t)right.center - (int32_t)frame->laser_mm[LASER_RIGHT]);
    features[42] = TaskDanger_ToModelFeatureI16((int32_t)left.center - (int32_t)frame->laser_mm[LASER_LEFT]);
    features[43] = TaskDanger_ToModelFeatureI16((int32_t)front.min - (int32_t)frame->laser_mm[LASER_FRONT]);
    features[44] = TaskDanger_ToModelFeatureI16((int32_t)right.min - (int32_t)frame->laser_mm[LASER_RIGHT]);
    features[45] = TaskDanger_ToModelFeatureI16((int32_t)left.min - (int32_t)frame->laser_mm[LASER_LEFT]);
    features[46] = TaskDanger_ToModelFeatureI16(front.close_count_800);
    features[47] = TaskDanger_ToModelFeatureI16(right.close_count_800);
    features[48] = TaskDanger_ToModelFeatureI16(left.close_count_800);

    feature_index = 49U;
    feature_index = TaskDanger_AppendSectorSegmentFeatures(features,
                                                           feature_index,
                                                           frame->lidar_front_mm,
                                                           s_danger_front_segment_ranges);
    feature_index = TaskDanger_AppendSectorSegmentFeatures(features,
                                                           feature_index,
                                                           frame->lidar_right_mm,
                                                           s_danger_right_segment_ranges);
    feature_index = TaskDanger_AppendSectorSegmentFeatures(features,
                                                           feature_index,
                                                           frame->lidar_left_mm,
                                                           s_danger_left_segment_ranges);
    (void)feature_index;
}
#endif

/* 周期性运行危险随机森林，并维护慢速/快速恢复的防抖状态。 */
static void TaskDanger_UpdateSlowdownModel(uint32_t now_tick)
{
#if TASKDANGER_USE_SLOWDOWN_MODEL == 0U
    (void)now_tick;
    g_danger_model_label = DANGER_MODEL_LABEL_0;
    g_danger_slowdown_active = 0U;
#else
    uint8_t label; /* DangerModel 当前预测标签。 */
    uint8_t slowdown_candidate; /* 本轮模型是否认为需要减速。 */

    if ((uint32_t)(now_tick - s_danger_model_last_update_tick) < TASKDANGER_MODEL_UPDATE_PERIOD_MS)
    {
        return;
    }
    s_danger_model_last_update_tick = now_tick;

    if (state_manager.current_state == STATE_INIT ||
        state_manager.current_state == STATE_EDGE_ESCAPE ||
        g_stage_type != STAGE_ON_STAGE ||
        TaskDanger_ShouldUseSlowdownProtection() == 0U)
    {
        g_danger_model_label = DANGER_MODEL_LABEL_0;
        g_danger_slowdown_active = 0U;
        s_danger_fast_candidate_tick = 0U;
        return;
    }

    TaskStage_CollectTrainFrame(&s_danger_model_frame);

    if (s_danger_model_frame.laser_mm[0] == 0U ||
        s_danger_model_frame.laser_mm[1] == 0U ||
        s_danger_model_frame.laser_mm[2] == 0U ||
        s_danger_model_frame.laser_mm[3] == 0U)
    {
        g_danger_model_label = DANGER_MODEL_LABEL_1;
        s_danger_fast_candidate_tick = 0U;
        g_danger_slowdown_active = 1U;
        return;
    }

    TaskDanger_BuildModelFeatures(&s_danger_model_frame, s_danger_model_features);
    label = DangerModel_Predict(s_danger_model_features);
    g_danger_model_label = label;
    slowdown_candidate = (label == DANGER_MODEL_LABEL_1) ? 1U : 0U;

    if (slowdown_candidate != 0U)
    {
        s_danger_fast_candidate_tick = 0U;
        g_danger_slowdown_active = 1U;
        return;
    }

    if (g_danger_slowdown_active == 0U)
    {
        s_danger_fast_candidate_tick = 0U;
        return;
    }

    if (s_danger_fast_candidate_tick == 0U)
    {
        s_danger_fast_candidate_tick = now_tick;
        return;
    }

    if ((uint32_t)(now_tick - s_danger_fast_candidate_tick) >= TASKDANGER_FAST_CONFIRM_MS)
    {
        g_danger_slowdown_active = 0U;
        s_danger_fast_candidate_tick = 0U;
    }
#endif
}

/* 只有台上常规运动状态启用硬件边缘保护，避免台下搜索/重上台时被远处传感器误打断。 */
static uint8_t TaskDanger_ShouldUseHardwareEdge(void)
{
    if (TaskDanger_IsEdgeProtectionSuppressed() != 0U ||
        TaskMain_IsEdgeProtectionDisabledByTilt() != 0U)
    {
        return 0U;
    }

    return (state_manager.current_state == STATE_PATROL ||
            state_manager.current_state == STATE_TRACK ||
            state_manager.current_state == STATE_ATTACK ||
            state_manager.current_state == STATE_AVOID_FRIENDLY_BLOCK) ? 1U : 0U;
}

/* 左侧边缘判断：默认只使用原来的近处传感器。 */
static uint8_t TaskDanger_IsLeftEdgeDetected(void)
{
#if TASKDANGER_USE_FAR_EDGE_SENSORS != 0U
    if (TaskDanger_ShouldUseFarEdgeProtection() != 0U)
    {
        return (left_danger == 0 || left_danger_2 != 0) ? 1U : 0U;
    }
#endif
    return (left_danger == 0) ? 1U : 0U;
}

/* 右侧边缘判断：默认只使用原来的近处传感器。 */
static uint8_t TaskDanger_IsRightEdgeDetected(void)
{
#if TASKDANGER_USE_FAR_EDGE_SENSORS != 0U
    if (TaskDanger_ShouldUseFarEdgeProtection() != 0U)
    {
        return (right_danger == 0 || right_danger_2 != 0) ? 1U : 0U;
    }
#endif
    return (right_danger == 0) ? 1U : 0U;
}

/* 危险检测线程入口：处理硬件边缘传感器和危险减速模型。 */
void Task_Danger_Run(void *argument)
{
    (void)argument;

    for (;;)
    {
        left_danger = HAL_GPIO_ReadPin(LEFT_LIGHT_GPIO_Port, LEFT_LIGHT_Pin);
        right_danger = HAL_GPIO_ReadPin(RIGHT_LIGHT_GPIO_Port, RIGHT_LIGHT_Pin);
#if TASKDANGER_USE_FAR_EDGE_SENSORS != 0U
        left_danger_2 = HAL_GPIO_ReadPin(LEFT_LIGHT_2_GPIO_Port, LEFT_LIGHT_2_Pin);
        right_danger_2 = HAL_GPIO_ReadPin(RIGHT_LIGHT_2_GPIO_Port, RIGHT_LIGHT_2_Pin);
#endif
        g_danger_stack_free_words = (uint32_t)uxTaskGetStackHighWaterMark(NULL);

        if (state_manager.current_state == STATE_INIT)
        {
            g_danger_slowdown_active = 0U;
#if TASKDANGER_USE_SLOWDOWN_MODEL != 0U
            s_danger_fast_candidate_tick = 0U;
#endif
            osDelay(1);
            continue;
        }
        if (TaskDanger_ShouldUseHardwareEdge() == 0U)
        {
            TaskDanger_UpdateSlowdownModel(osKernelGetTickCount());
            osDelay(1);
            continue;
        }

        if (TaskDanger_IsLeftEdgeDetected() != 0U && TaskDanger_IsRightEdgeDetected() == 0U)
        {
            g_danger_slowdown_active = 0U;
#if TASKDANGER_USE_SLOWDOWN_MODEL != 0U
            s_danger_fast_candidate_tick = 0U;
#endif
            edge_danger_type = EDGE_LEFT;
            if (left_danger == 0 || right_danger == 0)
            {
                Motor_SetSpeeds(-430, -430);
            }
            else
            {
                Motor_SetSpeeds(-280, -280);
            }
            TaskMain_SwitchState(STATE_EDGE_ESCAPE);
        }
        else if (TaskDanger_IsRightEdgeDetected() != 0U && TaskDanger_IsLeftEdgeDetected() == 0U)
        {
            g_danger_slowdown_active = 0U;
#if TASKDANGER_USE_SLOWDOWN_MODEL != 0U
            s_danger_fast_candidate_tick = 0U;
#endif
            edge_danger_type = EDGE_RIGHT;
            if (left_danger == 0 || right_danger == 0)
            {
                Motor_SetSpeeds(-430, -430);
            }
            else
            {
                Motor_SetSpeeds(-280, -280);
            }
            TaskMain_SwitchState(STATE_EDGE_ESCAPE);
        }
        else if (TaskDanger_IsRightEdgeDetected() != 0U && TaskDanger_IsLeftEdgeDetected() != 0U)
        {
            g_danger_slowdown_active = 0U;
#if TASKDANGER_USE_SLOWDOWN_MODEL != 0U
            s_danger_fast_candidate_tick = 0U;
#endif
            edge_danger_type = EDGE_FRONT;
            if (left_danger == 0 || right_danger == 0)
            {
                Motor_SetSpeeds(-430, -430);
            }
            else
            {
                Motor_SetSpeeds(-280, -280);
            }
            TaskMain_SwitchState(STATE_EDGE_ESCAPE);
        }

        TaskDanger_UpdateSlowdownModel(osKernelGetTickCount());

        osDelay(1);
    }
}
