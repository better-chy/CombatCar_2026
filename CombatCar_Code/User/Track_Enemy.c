#include "Track_Enemy.h"

#include <math.h>
#include "LiDAR.h"
#include "Task_TrainLog.h"

#define TRACK_ENEMY_CONFIRM_MAX_ANGLE_JUMP_DEG     45.0f /* 连续命中确认时，允许的最大目标角度跳变。 */
#define TRACK_ENEMY_CONFIRM_MAX_DISTANCE_JUMP_MM    800U /* 连续命中确认时，允许的最大目标距离跳变。 */
#define TRACK_ENEMY_MASK_EDGE_LEFT_SIDE            0x01U /* 候选段起点侧靠近屏蔽区。 */
#define TRACK_ENEMY_MASK_EDGE_RIGHT_SIDE           0x02U /* 候选段终点侧靠近屏蔽区。 */

typedef struct
{
    uint16_t start_index; /* 目标段起始索引。 */
    uint16_t end_index; /* 目标段结束索引。 */
    uint16_t center_index; /* 目标段中心索引。 */
    uint16_t min_distance_mm; /* 目标段内最近距离。 */
    uint16_t point_count; /* 目标段内连续点数。 */
} TrackEnemySegment;

static uint8_t s_trig_table_ready = 0U; /* 三角函数查表是否已经初始化。 */
static float s_trig_sin_table[LIDAR_SCAN_POINT_COUNT]; /* 每个原始扫描索引对应的正弦值。 */
static float s_trig_cos_table[LIDAR_SCAN_POINT_COUNT]; /* 每个原始扫描索引对应的余弦值。 */
static uint8_t s_raw_zero_mask[LIDAR_SCAN_POINT_COUNT]; /* 最近一圈原始数据中的 0 点标记，避免修补点参与背景跳变。 */
volatile TrackEnemyDebugInfo g_track_enemy_debug = {0}; /* 索敌调试信息，供 debug 直接观察。 */

static uint8_t TrackEnemy_GetSegmentMaskEdgeSide(uint16_t start_index, uint16_t end_index, uint16_t size);
static uint8_t TrackEnemy_IsSegmentNearMaskEdge(uint16_t start_index, uint16_t end_index, uint16_t size);
static uint16_t TrackEnemy_GetSegmentMinPoints(uint16_t start_index, uint16_t end_index, uint16_t size);
static uint8_t TrackEnemy_IsBackgroundRunSupported(const uint16_t *scan_distances,
                                                   uint16_t size,
                                                   uint16_t start_index,
                                                   int8_t direction,
                                                   uint16_t min_background_mm);
static uint8_t TrackEnemy_GetBackgroundDistanceMm(const uint16_t *scan_distances,
                                                  uint16_t size,
                                                  uint16_t segment_edge_index,
                                                  int8_t direction,
                                                  uint16_t foreground_distance_mm,
                                                  uint16_t *background_distance_mm);
static uint8_t TrackEnemy_HasAnyDistance(const uint16_t *scan_distances, uint16_t size);
static uint16_t TrackEnemy_CountZeroDistances(const uint16_t *scan_distances, uint16_t size);
static uint16_t TrackEnemy_CountMaxUnmaskedZeroRun(const uint16_t *scan_distances, uint16_t size);
static void TrackEnemy_RecordRawZeroMask(const uint16_t *scan_distances, uint16_t size);
static uint8_t TrackEnemy_IsSegmentEnemyLike(const uint16_t *scan_distances,
                                             uint16_t start_index,
                                             uint16_t end_index,
                                             uint16_t min_distance_mm,
                                             uint16_t point_count,
                                             uint16_t size);
static void TrackEnemy_DebugClearCandidate(void);
static void TrackEnemy_DebugRecordCandidate(uint16_t start_index,
                                            uint16_t end_index,
                                            uint16_t min_distance_mm,
                                            uint16_t point_count,
                                            uint16_t min_points_required,
                                            uint16_t width_mm,
                                            uint16_t left_background_mm,
                                            uint16_t right_background_mm,
                                            uint16_t left_jump_mm,
                                            uint16_t right_jump_mm,
                                            uint8_t reject_reason);
static void TrackEnemy_DebugAppendSegment(uint16_t start_index,
                                          uint16_t end_index,
                                          uint16_t center_index,
                                          int16_t center_angle_deg,
                                          uint16_t min_distance_mm,
                                          uint16_t point_count,
                                          uint16_t min_points_required,
                                          uint16_t width_mm,
                                          uint16_t left_background_mm,
                                          uint16_t right_background_mm,
                                          uint16_t left_jump_mm,
                                          uint16_t right_jump_mm,
                                          uint8_t reject_reason);
static void TrackEnemy_DebugRecordEnemyInfo(const EnemyInfo *enemy_info);
static uint8_t TrackEnemy_IsSameTargetForConfirm(const EnemyInfo *previous_enemy_info,
                                                 const EnemyInfo *current_enemy_info);
static void TrackEnemyTracker_ClearPending(TrackEnemyTracker *tracker);

//===========================角度辅助函数==================================

static void TrackEnemy_DebugClearCandidate(void)
{
    g_track_enemy_debug.candidate_start_index = 0U;
    g_track_enemy_debug.candidate_end_index = 0U;
    g_track_enemy_debug.candidate_center_index = 0U;
    g_track_enemy_debug.candidate_angle_deg = 0;
    g_track_enemy_debug.candidate_min_distance_mm = 0U;
    g_track_enemy_debug.candidate_point_count = 0U;
    g_track_enemy_debug.candidate_min_points_required = 0U;
    g_track_enemy_debug.candidate_width_mm = 0U;
    g_track_enemy_debug.candidate_left_background_mm = 0U;
    g_track_enemy_debug.candidate_right_background_mm = 0U;
    g_track_enemy_debug.candidate_left_jump_mm = 0U;
    g_track_enemy_debug.candidate_right_jump_mm = 0U;
}

static void TrackEnemy_DebugAppendSegment(uint16_t start_index,
                                          uint16_t end_index,
                                          uint16_t center_index,
                                          int16_t center_angle_deg,
                                          uint16_t min_distance_mm,
                                          uint16_t point_count,
                                          uint16_t min_points_required,
                                          uint16_t width_mm,
                                          uint16_t left_background_mm,
                                          uint16_t right_background_mm,
                                          uint16_t left_jump_mm,
                                          uint16_t right_jump_mm,
                                          uint8_t reject_reason)
{
    TrackEnemyDebugSegment *segment; /* 当前写入的 debug 线段。 */

    if (g_track_enemy_debug.segment_count >= TRACK_ENEMY_DEBUG_MAX_SEGMENTS)
    {
        g_track_enemy_debug.segment_overflow = 1U;
        return;
    }

    segment = (TrackEnemyDebugSegment *)&g_track_enemy_debug.segments[g_track_enemy_debug.segment_count];
    segment->accepted = (reject_reason == TRACK_ENEMY_DEBUG_REJECT_NONE) ? 1U : 0U;
    segment->reject_reason = reject_reason;
    segment->start_index = start_index;
    segment->end_index = end_index;
    segment->center_index = center_index;
    segment->center_angle_deg = center_angle_deg;
    segment->min_distance_mm = min_distance_mm;
    segment->point_count = point_count;
    segment->min_points_required = min_points_required;
    segment->width_mm = width_mm;
    segment->left_background_mm = left_background_mm;
    segment->right_background_mm = right_background_mm;
    segment->left_jump_mm = left_jump_mm;
    segment->right_jump_mm = right_jump_mm;
    ++g_track_enemy_debug.segment_count;
}

static void TrackEnemy_DebugRecordCandidate(uint16_t start_index,
                                            uint16_t end_index,
                                            uint16_t min_distance_mm,
                                            uint16_t point_count,
                                            uint16_t min_points_required,
                                            uint16_t width_mm,
                                            uint16_t left_background_mm,
                                            uint16_t right_background_mm,
                                            uint16_t left_jump_mm,
                                            uint16_t right_jump_mm,
                                            uint8_t reject_reason)
{
    uint16_t center_index; /* 最近候选段中心索引。 */

    center_index = (uint16_t)((start_index + ((point_count > 0U ? point_count : 1U) - 1U) / 2U) % LIDAR_SCAN_POINT_COUNT);

    TrackEnemy_DebugAppendSegment(start_index,
                                  end_index,
                                  center_index,
                                  (int16_t)TrackEnemy_IndexToAngleDeg(center_index),
                                  min_distance_mm,
                                  point_count,
                                  min_points_required,
                                  width_mm,
                                  left_background_mm,
                                  right_background_mm,
                                  left_jump_mm,
                                  right_jump_mm,
                                  reject_reason);

    g_track_enemy_debug.candidate_start_index = start_index;
    g_track_enemy_debug.candidate_end_index = end_index;
    g_track_enemy_debug.candidate_center_index = center_index;
    g_track_enemy_debug.candidate_angle_deg = (int16_t)TrackEnemy_IndexToAngleDeg(center_index);
    g_track_enemy_debug.candidate_min_distance_mm = min_distance_mm;
    g_track_enemy_debug.candidate_point_count = point_count;
    g_track_enemy_debug.candidate_min_points_required = min_points_required;
    g_track_enemy_debug.candidate_width_mm = width_mm;
    g_track_enemy_debug.candidate_left_background_mm = left_background_mm;
    g_track_enemy_debug.candidate_right_background_mm = right_background_mm;
    g_track_enemy_debug.candidate_left_jump_mm = left_jump_mm;
    g_track_enemy_debug.candidate_right_jump_mm = right_jump_mm;
    g_track_enemy_debug.last_reject_reason = reject_reason;
}

static void TrackEnemy_DebugRecordEnemyInfo(const EnemyInfo *enemy_info)
{
    if (enemy_info == NULL)
    {
        return;
    }

    g_track_enemy_debug.enemy_info.is_found = enemy_info->is_found;
    g_track_enemy_debug.enemy_info.angle_deg = enemy_info->angle_deg;
    g_track_enemy_debug.enemy_info.distance_mm = enemy_info->distance_mm;
    g_track_enemy_debug.enemy_info.center_index = enemy_info->center_index;
    g_track_enemy_debug.enemy_info.point_count = enemy_info->point_count;
}

/* 判断当前锁定结果是否像是上一帧目标的延续。
 * 如果角度或距离突然跳得太大，就把它当成新目标，避免误判线段快速升级成“连续 3 帧”。 */
static uint8_t TrackEnemy_IsSameTargetForConfirm(const EnemyInfo *previous_enemy_info,
                                                 const EnemyInfo *current_enemy_info)
{
    float angle_jump_deg;
    uint16_t distance_jump_mm;

    if (previous_enemy_info == NULL || current_enemy_info == NULL)
    {
        return 0U;
    }

    if (previous_enemy_info->is_found == 0U || current_enemy_info->is_found == 0U)
    {
        return 0U;
    }

    angle_jump_deg = fabsf(TrackEnemy_NormalizeAngleDeg((float)current_enemy_info->angle_deg -
                                                        (float)previous_enemy_info->angle_deg));
    if (current_enemy_info->distance_mm > previous_enemy_info->distance_mm)
    {
        distance_jump_mm = (uint16_t)(current_enemy_info->distance_mm - previous_enemy_info->distance_mm);
    }
    else
    {
        distance_jump_mm = (uint16_t)(previous_enemy_info->distance_mm - current_enemy_info->distance_mm);
    }

    if (angle_jump_deg > TRACK_ENEMY_CONFIRM_MAX_ANGLE_JUMP_DEG)
    {
        return 0U;
    }

    if (distance_jump_mm > TRACK_ENEMY_CONFIRM_MAX_DISTANCE_JUMP_MM)
    {
        return 0U;
    }

    return 1U;
}

static void TrackEnemyTracker_ClearPending(TrackEnemyTracker *tracker)
{
    if (tracker == NULL)
    {
        return;
    }

    TrackEnemy_ClearEnemyInfo(&tracker->pending_enemy_info);
    tracker->pending_start_tick = 0U;
    tracker->pending_last_seen_tick = 0U;
    tracker->pending_found = 0U;
}

/* 初始化原始扫描索引对应的 sin/cos 查表。 */
void TrackEnemy_InitTrigTable(void)
{
    uint16_t index; /* 当前初始化的原始扫描索引。 */

    if (s_trig_table_ready != 0U)
    {
        return;
    }

    for (index = 0U; index < LIDAR_SCAN_POINT_COUNT; ++index)
    {
        float angle_rad = TrackEnemy_IndexToAngleDeg(index) * (TRACK_ENEMY_PI / 180.0f); /* 当前索引对应的弧度角。 */
        s_trig_sin_table[index] = sinf(angle_rad);
        s_trig_cos_table[index] = cosf(angle_rad);
    }

    s_trig_table_ready = 1U;
}

/* 将任意角度归一化到 -180 到 180 度。 */
float TrackEnemy_NormalizeAngleDeg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

/* 将原始扫描索引转换为车体角。
 * 参数说明：
 * index 为原始扫描索引，取值范围为 0 到 499。
 * 返回值：
 * 返回该索引对应的车体角，单位为度。
 */
float TrackEnemy_IndexToAngleDeg(uint16_t index)
{
    if (index >= LIDAR_SCAN_POINT_COUNT)
    {
        return 180.0f;
    }

    return TrackEnemy_NormalizeAngleDeg((375.0f - (float)index) * LIDAR_SCAN_ANGLE_STEP_DEG);
}

//===========================================================================

//===========================遮挡区判断==================================

/* 判断给定车体角是否落在已知固定遮挡区内。
 * 返回值：
 * 1 表示落在遮挡区内；
 * 0 表示不在遮挡区内。
 */
uint8_t TrackEnemy_IsMaskedAngle(float angle_deg)
{
    float normalized_angle = TrackEnemy_NormalizeAngleDeg(angle_deg); /* 归一化后的车体角度。 */

    if ((normalized_angle >= TRACK_ENEMY_REAR_MASK_MIN_DEG && normalized_angle <= TRACK_ENEMY_REAR_MASK_MAX_DEG) ||
        (normalized_angle >= TRACK_ENEMY_REAR_MASK_NEG_MIN_DEG && normalized_angle <= TRACK_ENEMY_REAR_MASK_NEG_MAX_DEG))
    {
        return 1U;
    }

    if ((normalized_angle >= TRACK_ENEMY_FRONT_RIGHT_MASK_MIN_DEG && normalized_angle <= TRACK_ENEMY_FRONT_RIGHT_MASK_MAX_DEG) ||
        (normalized_angle >= TRACK_ENEMY_FRONT_LEFT_MASK_MIN_DEG && normalized_angle <= TRACK_ENEMY_FRONT_LEFT_MASK_MAX_DEG))
    {
        return 1U;
    }

    return 0U;
}

/* 判断原始扫描索引对应的车体角是否落在遮挡区内。 */
uint8_t TrackEnemy_IsMaskedIndex(uint16_t index)
{
    float angle_deg; /* 当前索引对应的车体角。 */

    if (index >= LIDAR_SCAN_POINT_COUNT)
    {
        return 1U;
    }

    angle_deg = TrackEnemy_IndexToAngleDeg(index);
    return TrackEnemy_IsMaskedAngle(angle_deg);
}

//===========================================================================

//===========================距离有效性判断==================================

/* 判断距离值是否落在当前索敌允许的有效窗口内。
 * 参数说明：
 * distance_mm 为待判断的距离值，单位为毫米。
 * 返回值：
 * 1 表示该距离可以参与索敌；
 * 0 表示该距离无效。
 */
uint8_t TrackEnemy_IsValidDistance(uint16_t distance_mm)
{
    if (distance_mm < TRACK_ENEMY_MIN_DISTANCE_MM)
    {
        return 0U;
    }

    if (distance_mm > TRACK_ENEMY_MAX_DISTANCE_MM)
    {
        return 0U;
    }

    return 1U;
}

//===========================================================================

//===========================目标宽度估计==================================

/* 使用目标段首尾两个点的极坐标，估算该目标段的实际宽度。
 * 参数说明：
 * scan_distances 为一圈距离数组；
 * start_index 为目标段起始索引；
 * end_index 为目标段结束索引。
 * 返回值：
 * 返回该目标段首尾两点之间的直线距离，单位为毫米；
 * 参数无效时返回 0。
 */
float TrackEnemy_ComputeSegmentWidthMm(const uint16_t *scan_distances, uint16_t start_index, uint16_t end_index)
{
    float start_x; /* 起点 X 坐标。 */
    float start_y; /* 起点 Y 坐标。 */
    float end_x; /* 终点 X 坐标。 */
    float end_y; /* 终点 Y 坐标。 */
    float dx; /* 首尾两点的 X 方向差值。 */
    float dy; /* 首尾两点的 Y 方向差值。 */
    uint16_t start_distance_mm; /* 起点距离值。 */
    uint16_t end_distance_mm; /* 终点距离值。 */

    if (scan_distances == NULL ||
        start_index >= LIDAR_SCAN_POINT_COUNT ||
        end_index >= LIDAR_SCAN_POINT_COUNT)
    {
        return 0.0f;
    }

    TrackEnemy_InitTrigTable();

    start_distance_mm = scan_distances[start_index];
    end_distance_mm = scan_distances[end_index];

    if (TrackEnemy_IsValidDistance(start_distance_mm) == 0U ||
        TrackEnemy_IsValidDistance(end_distance_mm) == 0U)
    {
        return 0.0f;
    }

    start_x = (float)start_distance_mm * s_trig_cos_table[start_index];
    start_y = (float)start_distance_mm * s_trig_sin_table[start_index];
    end_x = (float)end_distance_mm * s_trig_cos_table[end_index];
    end_y = (float)end_distance_mm * s_trig_sin_table[end_index];

    dx = end_x - start_x;
    dy = end_y - start_y;

    return sqrtf(dx * dx + dy * dy);
}

//===========================================================================

//===========================目标信息辅助函数==================================

/* 判断目标段哪一侧靠近屏蔽区边界。
 * 参数说明：
 * start_index 为目标段起始索引；
 * end_index 为目标段结束索引；
 * size 为数组长度。
 * 返回值：
 * TRACK_ENEMY_MASK_EDGE_LEFT_SIDE 表示起点侧边缘范围内有屏蔽区；
 * TRACK_ENEMY_MASK_EDGE_RIGHT_SIDE 表示终点侧边缘范围内有屏蔽区；
 * 0 表示两侧都不靠近屏蔽区。
 */
static uint8_t TrackEnemy_GetSegmentMaskEdgeSide(uint16_t start_index, uint16_t end_index, uint16_t size)
{
    uint16_t step; /* 向候选段外侧查找的原始索引步数。 */
    uint16_t left_index; /* 起点侧向外查找的索引。 */
    uint16_t right_index; /* 终点侧向外查找的索引。 */
    uint8_t edge_side = 0U; /* 候选段贴近屏蔽区的侧边标志。 */

    if (size == 0U)
    {
        return 0U;
    }

    left_index = start_index;
    right_index = end_index;

    for (step = 0U; step < TRACK_ENEMY_MASK_EDGE_MARGIN_INDEX; ++step)
    {
        left_index = (uint16_t)((left_index + size - 1U) % size);
        right_index = (uint16_t)((right_index + 1U) % size);

        if (TrackEnemy_IsMaskedIndex(left_index) != 0U)
        {
            edge_side |= TRACK_ENEMY_MASK_EDGE_LEFT_SIDE;
        }

        if (TrackEnemy_IsMaskedIndex(right_index) != 0U)
        {
            edge_side |= TRACK_ENEMY_MASK_EDGE_RIGHT_SIDE;
        }

        if (edge_side == (TRACK_ENEMY_MASK_EDGE_LEFT_SIDE | TRACK_ENEMY_MASK_EDGE_RIGHT_SIDE))
        {
            break;
        }
    }

    return edge_side;
}

/* 判断目标段是否在屏蔽区边界附近。 */
static uint8_t TrackEnemy_IsSegmentNearMaskEdge(uint16_t start_index, uint16_t end_index, uint16_t size)
{
    return (TrackEnemy_GetSegmentMaskEdgeSide(start_index, end_index, size) != 0U) ? 1U : 0U;
}

/* 根据目标段是否贴近屏蔽区边界，返回当前应使用的最小点数门槛。 */
static uint16_t TrackEnemy_GetSegmentMinPoints(uint16_t start_index, uint16_t end_index, uint16_t size)
{
    if (TrackEnemy_IsSegmentNearMaskEdge(start_index, end_index, size) != 0U)
    {
        return TRACK_ENEMY_MASK_EDGE_MIN_POINTS;
    }

    return TRACK_ENEMY_MIN_POINTS;
}

static uint8_t TrackEnemy_IsBackgroundRunSupported(const uint16_t *scan_distances,
                                                   uint16_t size,
                                                   uint16_t start_index,
                                                   int8_t direction,
                                                   uint16_t min_background_mm)
{
    uint16_t supported_points = 0U;
    uint16_t step;
    uint16_t index = start_index;

    if (scan_distances == NULL || size == 0U)
    {
        return 0U;
    }

    for (step = 0U; step < TRACK_ENEMY_BACKGROUND_MIN_RUN_POINTS; ++step)
    {
        uint16_t distance_mm;

        if (TrackEnemy_IsMaskedIndex(index) != 0U ||
            (index < LIDAR_SCAN_POINT_COUNT && s_raw_zero_mask[index] != 0U))
        {
            return 0U;
        }

        distance_mm = scan_distances[index];
        if (TrackEnemy_IsValidDistance(distance_mm) == 0U ||
            distance_mm < min_background_mm)
        {
            return 0U;
        }

        ++supported_points;
        if (supported_points >= TRACK_ENEMY_BACKGROUND_MIN_RUN_POINTS)
        {
            return 1U;
        }

        if (direction < 0)
        {
            index = (uint16_t)((index + size - 1U) % size);
        }
        else
        {
            index = (uint16_t)((index + 1U) % size);
        }
    }

    return 0U;
}

/* 从候选目标段边界向外寻找有效背景距离，允许跳过少量孤立近点。 */
static uint8_t TrackEnemy_GetBackgroundDistanceMm(const uint16_t *scan_distances,
                                                  uint16_t size,
                                                  uint16_t segment_edge_index,
                                                  int8_t direction,
                                                  uint16_t foreground_distance_mm,
                                                  uint16_t *background_distance_mm)
{
    uint16_t step; /* 当前向外搜索的步数。 */
    uint16_t index; /* 当前检查的原始扫描索引。 */
    uint16_t distance_mm; /* 当前检查到的有效距离。 */
    uint16_t first_valid_distance_mm = 0U; /* 第一个有效点，找不到更好背景时用于保持旧行为。 */
    uint16_t skipped_points = 0U; /* 已跳过的孤立近点数量。 */
    uint16_t zero_gap = 0U; /* 连续无效/0 点数量，避免跨过大段丢点去找假背景。 */
    uint16_t min_background_mm; /* 满足背景跳变所需的最小距离。 */
    uint8_t has_first_valid = 0U; /* 是否已经记录第一个有效点。 */

    if (scan_distances == NULL || background_distance_mm == NULL || size == 0U)
    {
        return 0U;
    }

    index = segment_edge_index;
    for (step = 0U; step < size; ++step)
    {
        if (direction < 0)
        {
            index = (uint16_t)((index + size - 1U) % size);
        }
        else
        {
            index = (uint16_t)((index + 1U) % size);
        }

        if (TrackEnemy_IsMaskedIndex(index) != 0U)
        {
            continue;
        }

        distance_mm = scan_distances[index];
        if (index < LIDAR_SCAN_POINT_COUNT && s_raw_zero_mask[index] != 0U)
        {
            ++zero_gap;
            if (zero_gap > TRACK_ENEMY_BACKGROUND_MAX_ZERO_GAP)
            {
                return 0U;
            }
            continue;
        }

        if (TrackEnemy_IsValidDistance(distance_mm) != 0U)
        {
            zero_gap = 0U;
            min_background_mm = (uint16_t)(foreground_distance_mm + TRACK_ENEMY_BACKGROUND_JUMP_MM);
            if (distance_mm >= min_background_mm)
            {
                if (TrackEnemy_IsBackgroundRunSupported(scan_distances,
                                                        size,
                                                        index,
                                                        direction,
                                                        min_background_mm) != 0U)
                {
                    *background_distance_mm = distance_mm;
                    return 1U;
                }

                if (skipped_points < TRACK_ENEMY_BACKGROUND_SKIP_POINTS)
                {
                    ++skipped_points;
                    continue;
                }

                return 0U;
            }

            if (has_first_valid == 0U)
            {
                first_valid_distance_mm = distance_mm;
                has_first_valid = 1U;
            }

            if (skipped_points < TRACK_ENEMY_BACKGROUND_SKIP_POINTS)
            {
                ++skipped_points;
                continue;
            }

            *background_distance_mm = first_valid_distance_mm;
            return 1U;
        }

        if (distance_mm == 0U)
        {
            ++zero_gap;
            if (zero_gap > TRACK_ENEMY_BACKGROUND_MAX_ZERO_GAP)
            {
                return 0U;
            }
        }
    }

    if (has_first_valid != 0U)
    {
        *background_distance_mm = first_valid_distance_mm;
        return 1U;
    }

    return 0U;
}

/* 判断一个候选段是否同时满足点数、宽度和背景跳变要求。 */
static uint8_t TrackEnemy_IsSegmentEnemyLike(const uint16_t *scan_distances,
                                             uint16_t start_index,
                                             uint16_t end_index,
                                             uint16_t min_distance_mm,
                                             uint16_t point_count,
                                             uint16_t size)
{
    uint16_t min_points_required; /* 当前候选段需要满足的最小连续点数。 */
    uint16_t left_background_mm = 0U; /* 候选段左侧背景距离。 */
    uint16_t right_background_mm = 0U; /* 候选段右侧背景距离。 */
    uint16_t left_jump_mm = 0U; /* 候选段左侧背景相对前景的距离跳变。 */
    uint16_t right_jump_mm = 0U; /* 候选段右侧背景相对前景的距离跳变。 */
    uint8_t mask_edge_side; /* 候选段哪一侧靠近屏蔽区。 */
    uint8_t background_ok = 0U; /* 背景跳变是否满足当前线段规则。 */
#if TRACK_ENEMY_LOW_PROFILE_ENABLE
    uint8_t point_count_ok; /* 当前线段点数是否满足普通门槛。 */
    uint8_t low_profile_ok = 0U; /* 当前线段是否满足低矮目标特例。 */
#endif
    uint8_t has_left_background; /* 左侧是否找到有效背景。 */
    uint8_t has_right_background; /* 右侧是否找到有效背景。 */
    float width_mm; /* 当前候选段估算宽度。 */
#if TRACK_ENEMY_LOW_PROFILE_ENABLE
    float center_abs_angle_deg; /* 当前候选段中心角绝对值。 */
#endif
    uint16_t width_mm_u16; /* 当前候选段估算宽度的整数调试值。 */
#if TRACK_ENEMY_LOW_PROFILE_ENABLE
    uint16_t center_index; /* 当前候选段中心索引。 */
#endif

    min_points_required = TrackEnemy_GetSegmentMinPoints(start_index, end_index, size);
#if TRACK_ENEMY_LOW_PROFILE_ENABLE
    if (point_count < TRACK_ENEMY_LOW_PROFILE_MIN_POINTS)
#else
    if (point_count < min_points_required)
#endif
    {
        TrackEnemy_DebugRecordCandidate(start_index,
                                        end_index,
                                        min_distance_mm,
                                        point_count,
                                        min_points_required,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        TRACK_ENEMY_DEBUG_REJECT_POINT_COUNT);
        return 0U;
    }

#if TRACK_ENEMY_LOW_PROFILE_ENABLE
    point_count_ok = (point_count >= min_points_required) ? 1U : 0U;
#endif
    width_mm = TrackEnemy_ComputeSegmentWidthMm(scan_distances, start_index, end_index);
    width_mm_u16 = (width_mm >= 65535.0f) ? 65535U : (uint16_t)(width_mm + 0.5f);

    has_left_background = TrackEnemy_GetBackgroundDistanceMm(scan_distances,
                                                             size,
                                                             start_index,
                                                             -1,
                                                             min_distance_mm,
                                                             &left_background_mm);
    has_right_background = TrackEnemy_GetBackgroundDistanceMm(scan_distances,
                                                              size,
                                                              end_index,
                                                              1,
                                                              min_distance_mm,
                                                              &right_background_mm);

    if (has_left_background != 0U && left_background_mm > min_distance_mm)
    {
        left_jump_mm = (uint16_t)(left_background_mm - min_distance_mm);
    }

    if (has_right_background != 0U && right_background_mm > min_distance_mm)
    {
        right_jump_mm = (uint16_t)(right_background_mm - min_distance_mm);
    }

    if (width_mm > TRACK_ENEMY_MAX_WIDTH_MM)
    {
        TrackEnemy_DebugRecordCandidate(start_index,
                                        end_index,
                                        min_distance_mm,
                                        point_count,
                                        min_points_required,
                                        width_mm_u16,
                                        left_background_mm,
                                        right_background_mm,
                                        left_jump_mm,
                                        right_jump_mm,
                                        TRACK_ENEMY_DEBUG_REJECT_WIDTH);
        return 0U;
    }

    if (left_jump_mm == 0U || right_jump_mm == 0U)
    {
        TrackEnemy_DebugRecordCandidate(start_index,
                                        end_index,
                                        min_distance_mm,
                                        point_count,
                                        min_points_required,
                                        width_mm_u16,
                                        left_background_mm,
                                        right_background_mm,
                                        left_jump_mm,
                                        right_jump_mm,
                                        TRACK_ENEMY_DEBUG_REJECT_BACKGROUND_JUMP);
        return 0U;
    }

#if TRACK_ENEMY_LOW_PROFILE_ENABLE
    center_index = (uint16_t)((start_index + ((point_count > 0U ? point_count : 1U) - 1U) / 2U) % LIDAR_SCAN_POINT_COUNT);
    center_abs_angle_deg = fabsf(TrackEnemy_IndexToAngleDeg(center_index));
    if (point_count_ok == 0U &&
        center_abs_angle_deg <= TRACK_ENEMY_LOW_PROFILE_MAX_ABS_ANGLE_DEG &&
        min_distance_mm >= TRACK_ENEMY_LOW_PROFILE_MIN_DISTANCE_MM &&
        min_distance_mm <= TRACK_ENEMY_LOW_PROFILE_MAX_DISTANCE_MM &&
        width_mm <= TRACK_ENEMY_LOW_PROFILE_MAX_WIDTH_MM &&
        left_jump_mm >= TRACK_ENEMY_LOW_PROFILE_BACKGROUND_JUMP_MM &&
        right_jump_mm >= TRACK_ENEMY_LOW_PROFILE_BACKGROUND_JUMP_MM)
    {
        low_profile_ok = 1U;
    }

    if (point_count_ok == 0U && low_profile_ok == 0U)
    {
        TrackEnemy_DebugRecordCandidate(start_index,
                                        end_index,
                                        min_distance_mm,
                                        point_count,
                                        min_points_required,
                                        width_mm_u16,
                                        left_background_mm,
                                        right_background_mm,
                                        left_jump_mm,
                                        right_jump_mm,
                                        TRACK_ENEMY_DEBUG_REJECT_POINT_COUNT);
        return 0U;
    }
#endif

    mask_edge_side = TrackEnemy_GetSegmentMaskEdgeSide(start_index, end_index, size);
    if ((mask_edge_side & TRACK_ENEMY_MASK_EDGE_LEFT_SIDE) != 0U &&
        (mask_edge_side & TRACK_ENEMY_MASK_EDGE_RIGHT_SIDE) == 0U)
    {
        background_ok = (right_jump_mm >= TRACK_ENEMY_BACKGROUND_JUMP_MM) ? 1U : 0U;
    }
    else if ((mask_edge_side & TRACK_ENEMY_MASK_EDGE_RIGHT_SIDE) != 0U &&
             (mask_edge_side & TRACK_ENEMY_MASK_EDGE_LEFT_SIDE) == 0U)
    {
        background_ok = (left_jump_mm >= TRACK_ENEMY_BACKGROUND_JUMP_MM) ? 1U : 0U;
    }
    else if (mask_edge_side == (TRACK_ENEMY_MASK_EDGE_LEFT_SIDE | TRACK_ENEMY_MASK_EDGE_RIGHT_SIDE))
    {
        background_ok = (left_jump_mm >= TRACK_ENEMY_BACKGROUND_JUMP_MM ||
                         right_jump_mm >= TRACK_ENEMY_BACKGROUND_JUMP_MM) ? 1U : 0U;
    }
    else
    {
        background_ok = (left_jump_mm >= TRACK_ENEMY_BACKGROUND_JUMP_MM &&
                         right_jump_mm >= TRACK_ENEMY_BACKGROUND_JUMP_MM) ? 1U : 0U;
    }

#if TRACK_ENEMY_LOW_PROFILE_ENABLE
    if (low_profile_ok != 0U)
    {
        background_ok = 1U;
    }
#endif

    if (background_ok != 0U)
    {
        TrackEnemy_DebugRecordCandidate(start_index,
                                        end_index,
                                        min_distance_mm,
                                        point_count,
                                        min_points_required,
                                        width_mm_u16,
                                        left_background_mm,
                                        right_background_mm,
                                        left_jump_mm,
                                        right_jump_mm,
                                        TRACK_ENEMY_DEBUG_REJECT_NONE);
        return 1U;
    }

    TrackEnemy_DebugRecordCandidate(start_index,
                                    end_index,
                                    min_distance_mm,
                                    point_count,
                                    min_points_required,
                                    width_mm_u16,
                                    left_background_mm,
                                    right_background_mm,
                                    left_jump_mm,
                                    right_jump_mm,
                                    TRACK_ENEMY_DEBUG_REJECT_BACKGROUND_JUMP);
    return 0U;
}

/* 清空敌方目标信息结构体。 */
void TrackEnemy_ClearEnemyInfo(EnemyInfo *enemy_info)
{
    if (enemy_info == NULL)
    {
        return;
    }

    enemy_info->is_found = 0U;
    enemy_info->angle_deg = 0;
    enemy_info->distance_mm = 0U;
    enemy_info->center_index = 0U;
    enemy_info->point_count = 0U;
}

//===========================================================================

//=============================索敌数据辅助区域=======================================

static uint8_t TrackEnemy_HasAnyDistance(const uint16_t *scan_distances, uint16_t size)
{
    uint16_t index;

    if (scan_distances == NULL || size == 0U)
    {
        return 0U;
    }

    for (index = 0U; index < size; ++index)
    {
        if (scan_distances[index] != 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

static uint16_t TrackEnemy_CountZeroDistances(const uint16_t *scan_distances, uint16_t size)
{
    uint16_t index;
    uint16_t zero_count = 0U;

    if (scan_distances == NULL || size == 0U)
    {
        return 0U;
    }

    for (index = 0U; index < size; ++index)
    {
        if (scan_distances[index] == 0U)
        {
            ++zero_count;
        }
    }

    return zero_count;
}

static uint16_t TrackEnemy_CountMaxUnmaskedZeroRun(const uint16_t *scan_distances, uint16_t size)
{
    uint16_t break_index = 0U;
    uint16_t step;
    uint16_t max_run = 0U;
    uint16_t current_run = 0U;
    uint8_t has_break = 0U;

    if (scan_distances == NULL || size == 0U)
    {
        return 0U;
    }

    if (size > LIDAR_SCAN_POINT_COUNT)
    {
        size = LIDAR_SCAN_POINT_COUNT;
    }

    for (break_index = 0U; break_index < size; ++break_index)
    {
        if (scan_distances[break_index] != 0U || TrackEnemy_IsMaskedIndex(break_index) != 0U)
        {
            has_break = 1U;
            break;
        }
    }

    if (has_break == 0U)
    {
        return size;
    }

    for (step = 1U; step <= size; ++step)
    {
        uint16_t index = (uint16_t)((break_index + step) % size);
        if (scan_distances[index] == 0U && TrackEnemy_IsMaskedIndex(index) == 0U)
        {
            ++current_run;
            if (current_run > max_run)
            {
                max_run = current_run;
            }
        }
        else
        {
            current_run = 0U;
        }
    }

    return max_run;
}

static void TrackEnemy_RecordRawZeroMask(const uint16_t *scan_distances, uint16_t size)
{
    uint16_t index;
    uint16_t limit = size;

    for (index = 0U; index < LIDAR_SCAN_POINT_COUNT; ++index)
    {
        s_raw_zero_mask[index] = 0U;
    }

    if (scan_distances == NULL || size == 0U)
    {
        return;
    }

    if (limit > LIDAR_SCAN_POINT_COUNT)
    {
        limit = LIDAR_SCAN_POINT_COUNT;
    }

    for (index = 0U; index < limit; ++index)
    {
        if (scan_distances[index] == 0U)
        {
            s_raw_zero_mask[index] = 1U;
        }
    }
}

/* 修补短暂抖动导致的 0 距离洞，避免目标段被意外打断。
 * 修补规则：
 * 1. 洞两侧都有有效值时，使用线性插值补齐；
 * 2. 仅左侧有有效值时，使用左侧值补齐；
 * 3. 仅右侧有有效值时，使用右侧值补齐；
 * 4. 连续 0 过长时不修补，保持为 0。
 * 参数说明：
 * scan_distances 为待修补的距离数组；
 * size 为数组长度。
 */
void TrackEnemy_PatchZeroHoles(uint16_t *scan_distances, uint16_t size)
{
    uint16_t i; /* 当前扫描到的数组索引。 */

    if (scan_distances == NULL || size == 0U)
    {
        return;
    }

    i = 0U;
    while (i < size)
    {
        uint16_t hole_start; /* 当前 0 洞的起始索引。 */
        uint16_t hole_end; /* 当前 0 洞的结束索引。 */
        uint16_t hole_len; /* 当前 0 洞的长度。 */
        uint16_t left_value = 0U; /* 洞左侧最近有效距离。 */
        uint16_t right_value = 0U; /* 洞右侧最近有效距离。 */
        uint8_t has_left = 0U; /* 左侧是否存在有效距离。 */
        uint8_t has_right = 0U; /* 右侧是否存在有效距离。 */
        uint16_t k; /* 修补洞内点时的索引。 */

        if (scan_distances[i] != 0U)
        {
            ++i;
            continue;
        }

        hole_start = i;
        while (i < size && scan_distances[i] == 0U)
        {
            ++i;
        }
        hole_end = (uint16_t)(i - 1U);
        hole_len = (uint16_t)(hole_end - hole_start + 1U);

        if (hole_len > TRACK_ENEMY_ZERO_PATCH_MAX_GAP)
        {
            continue;
        }

        if (hole_start > 0U && scan_distances[hole_start - 1U] != 0U)
        {
            left_value = scan_distances[hole_start - 1U];
            has_left = 1U;
        }

        if (i < size && scan_distances[i] != 0U)
        {
            right_value = scan_distances[i];
            has_right = 1U;
        }

        if (has_left != 0U && has_right != 0U)
        {
            for (k = 0U; k < hole_len; ++k)
            {
                int32_t delta = (int32_t)right_value - (int32_t)left_value; /* 洞两侧距离差。 */
                scan_distances[hole_start + k] = (uint16_t)((int32_t)left_value +
                    (delta * (int32_t)(k + 1U)) / (int32_t)(hole_len + 1U));
            }
            continue;
        }

        if (has_left != 0U)
        {
            for (k = 0U; k < hole_len; ++k)
            {
                scan_distances[hole_start + k] = left_value;
            }
            continue;
        }

        if (has_right != 0U)
        {
            for (k = 0U; k < hole_len; ++k)
            {
                scan_distances[hole_start + k] = right_value;
            }
        }
    }
}

//===========================================================================

/* 获取一圈冻结快照，并在返回前完成 0 洞修补。
 * 当前主策略不直接等待整圈快照；保留该接口用于后续调试、离线分析或更稳定的全局判断。
 */
uint8_t TrackEnemy_GetPatchedSnapshot(uint16_t *scan_distances, uint16_t size)
{
    uint16_t zero_count;
    uint16_t max_zero_run;

    if (scan_distances == NULL || size == 0U)
    {
        return 0U;
    }

    if (LiDAR_IsScanReady() == 0U)
    {
        return 0U;
    }

    LiDAR_GetDistanceArray(scan_distances, size);
    LiDAR_ClearScanReady();
    TrackEnemy_RecordRawZeroMask(scan_distances, size);
    zero_count = TrackEnemy_CountZeroDistances(scan_distances, size);
    max_zero_run = TrackEnemy_CountMaxUnmaskedZeroRun(scan_distances, size);
    if (zero_count > TRACK_ENEMY_MAX_ZERO_POINTS ||
        max_zero_run > TRACK_ENEMY_MAX_ZERO_RUN_POINTS)
    {
        g_track_enemy_debug.last_reject_reason = TRACK_ENEMY_DEBUG_REJECT_TOO_MANY_ZERO;
        return 0U;
    }

    TrackEnemy_PatchZeroHoles(scan_distances, size);

    return 1U;
}

/* 读取实时写缓冲，并在返回前完成 0 洞修补。
 * 索敌不等待 LiDAR 完成一整圈扫描，避免目标更新频率被整圈刷新周期限制。
 */
uint8_t TrackEnemy_GetLatestSnapshot(uint16_t *scan_distances, uint16_t size)
{
    uint16_t zero_count; /* 修补前的一圈原始 0 点数量。 */
    uint16_t max_zero_run; /* 修补前非屏蔽区最长连续 0 点数量。 */

    if (scan_distances == NULL || size == 0U)
    {
        return 0U;
    }

    LiDAR_GetLiveDistanceArray(scan_distances, size);
    TrackEnemy_RecordRawZeroMask(scan_distances, size);

    zero_count = TrackEnemy_CountZeroDistances(scan_distances, size);
    max_zero_run = TrackEnemy_CountMaxUnmaskedZeroRun(scan_distances, size);
    if (zero_count > TRACK_ENEMY_MAX_ZERO_POINTS ||
        max_zero_run > TRACK_ENEMY_MAX_ZERO_RUN_POINTS)
    {
        g_track_enemy_debug.last_reject_reason = TRACK_ENEMY_DEBUG_REJECT_TOO_MANY_ZERO;
        return 0U;
    }

    if (TrackEnemy_HasAnyDistance(scan_distances, size) == 0U)
    {
        g_track_enemy_debug.last_reject_reason = TRACK_ENEMY_DEBUG_REJECT_NO_SNAPSHOT;
        return 0U;
    }

    TrackEnemy_PatchZeroHoles(scan_distances, size);
    return 1U;
}

//===========================================================================

//=============================连续段分割与基础目标提取===============================

/* TODO：
 * 1. 候选目标距离接近时加入前向优先级，避免只按最近距离选侧向碎段；
 * 2. 增加围墙直线特征过滤，继续降低把台下围墙当敌车的概率；
 * 3. 与 AprilTag / 视觉结果融合，降低把能量块当敌方车辆的概率；
 * 4. 增加目标短时记忆与稳定判据，减少单圈抖动导致的丢目标现象。
 */

/* 从一圈已经修补完成的距离快照中提取最基础的目标段。
 * 当前版本仅完成：
 * 1. 屏蔽区剔除；
 * 2. 无效距离剔除；
 * 3. 按距离跳变分段；
 * 4. 使用点数、宽度和背景跳变筛掉不像敌车的候选段；
 * 5. 选择最近的有效候选段。
 * 返回值：
 * 1 表示成功找到目标；
 * 0 表示当前没有找到可用目标。
 * 参数说明：
 * scan_distances 为已经完成 0 洞修补的一圈距离数组；
 * size 为数组长度；
 * enemy_info 为输出的目标信息。
 */
uint8_t TrackEnemy_FindBasicTarget(const uint16_t *scan_distances, uint16_t size, EnemyInfo *enemy_info)
{
    uint16_t offset; /* 相对环形扫描起点的偏移量。 */
    uint16_t index; /* 当前扫描到的原始数组索引。 */
    uint8_t in_segment = 0U; /* 当前是否处于连续目标段内。 */
    uint16_t seg_start = 0U; /* 当前目标段起始索引。 */
    uint16_t seg_min_distance = 0U; /* 当前目标段最近距离。 */
    uint16_t seg_points = 0U; /* 当前目标段点数。 */
    uint16_t prev_distance = 0U; /* 当前目标段上一个点的距离。 */
    TrackEnemySegment best_segment; /* 当前扫描中选出的最优目标段。 */
    uint8_t best_found = 0U; /* 当前扫描中是否已经找到可用目标段。 */

    if (scan_distances == NULL || size == 0U || enemy_info == NULL)
    {
        return 0U;
    }

    TrackEnemy_ClearEnemyInfo(enemy_info);
    TrackEnemy_DebugClearCandidate();
    g_track_enemy_debug.valid_point_count = 0U;
    g_track_enemy_debug.masked_valid_point_count = 0U;
    g_track_enemy_debug.segment_count = 0U;
    g_track_enemy_debug.segment_overflow = 0U;
    g_track_enemy_debug.last_reject_reason = TRACK_ENEMY_DEBUG_REJECT_NO_SEGMENT;
    best_segment.start_index = 0U;
    best_segment.end_index = 0U;
    best_segment.center_index = 0U;
    best_segment.min_distance_mm = 0xFFFFU;
    best_segment.point_count = 0U;

    /* 从后方屏蔽区内的固定起点开始做环形扫描。
       这样数组拼接缝会落在后方遮挡区内，避免目标跨越 499->0 时被误切成两段。 */
    for (offset = 0U; offset < size; ++offset)
    {
        index = (uint16_t)((TRACK_ENEMY_SCAN_START_INDEX + offset) % size);
        uint16_t distance_mm = scan_distances[index]; /* 当前点距离。 */
        uint8_t point_valid; /* 当前点是否允许参与索敌。 */

        /* 如果当前点落在屏蔽区，直接判为无效点。 */
        if (TrackEnemy_IsMaskedIndex(index) != 0U)
        {
            if (TrackEnemy_IsValidDistance(distance_mm) != 0U)
            {
                ++g_track_enemy_debug.masked_valid_point_count;
            }
            point_valid = 0U;
        }
        else
        {
            /* 非屏蔽区内的点再根据距离窗口判断是否有效。 */
            point_valid = TrackEnemy_IsValidDistance(distance_mm);
            if (point_valid != 0U)
            {
                ++g_track_enemy_debug.valid_point_count;
            }
        }

        //如果遇到了一点无效点的情况
        if (point_valid == 0U)
        {
            if (in_segment != 0U)
            {
                uint16_t seg_end = (uint16_t)((index + size - 1U) % size); /* 当前目标段结束索引。 */

                /* 当前点无效，说明前一段连续目标到这里结束。
                   如果该段点数足够，就与当前最佳目标做一次比较。 */
                if (TrackEnemy_IsSegmentEnemyLike(scan_distances,
                                                  seg_start,
                                                  seg_end,
                                                  seg_min_distance,
                                                  seg_points,
                                                  size) != 0U &&
                    (best_found == 0U || seg_min_distance < best_segment.min_distance_mm))
                {
                    best_segment.start_index = seg_start;
                    best_segment.end_index = seg_end;
                    best_segment.center_index = (uint16_t)((seg_start + ((seg_points - 1U) / 2U)) % size);
                    best_segment.min_distance_mm = seg_min_distance;
                    best_segment.point_count = seg_points;
                    best_found = 1U;
                }

                /* 当前目标段已经结算，清空段状态，等待下一段开始。 */
                in_segment = 0U;
                seg_points = 0U;
                prev_distance = 0U;
            }
            continue;
        }

        if (in_segment == 0U)
        {
            /* 当前点有效且之前不在目标段内，说明发现了新目标段的起点。 */
            in_segment = 1U;
            seg_start = index;
            seg_min_distance = distance_mm;
            seg_points = 1U;
            prev_distance = distance_mm;
            continue;
        }

        /* 当前点有效，但如果与上一点距离跳变过大，
           认为前一段已经结束，当前点属于下一段的新起点。 */
        if ((distance_mm > prev_distance ? (distance_mm - prev_distance) : (prev_distance - distance_mm)) > TRACK_ENEMY_POINT_JUMP_MM)
        {
            uint16_t seg_end = (uint16_t)((index + size - 1U) % size); /* 当前目标段结束索引。 */

            if (TrackEnemy_IsSegmentEnemyLike(scan_distances,
                                              seg_start,
                                              seg_end,
                                              seg_min_distance,
                                              seg_points,
                                              size) != 0U &&
                (best_found == 0U || seg_min_distance < best_segment.min_distance_mm))
            {
                best_segment.start_index = seg_start;
                best_segment.end_index = seg_end;
                best_segment.center_index = (uint16_t)((seg_start + ((seg_points - 1U) / 2U)) % size);
                best_segment.min_distance_mm = seg_min_distance;
                best_segment.point_count = seg_points;
                best_found = 1U;
            }

            /* 用当前点重新开启一段新的候选目标段。 */
            seg_start = index;
            seg_min_distance = distance_mm;
            seg_points = 1U;
            prev_distance = distance_mm;
            continue;
        }

        /* 当前点和前一点连续，继续累积当前目标段信息。 */
        ++seg_points;
        if (distance_mm < seg_min_distance)
        {
            /* 持续记录该段内最近距离，后面用于目标择优。 */
            seg_min_distance = distance_mm;
        }
        prev_distance = distance_mm;
    }

    if (in_segment != 0U)
    {
        uint16_t seg_end = (uint16_t)((TRACK_ENEMY_SCAN_START_INDEX + size - 1U) % size); /* 最后一段结束索引。 */

        /* 如果循环结束时仍处于目标段内，说明最后一段还没结算。 */
        if (TrackEnemy_IsSegmentEnemyLike(scan_distances,
                                          seg_start,
                                          seg_end,
                                          seg_min_distance,
                                          seg_points,
                                          size) != 0U &&
            (best_found == 0U || seg_min_distance < best_segment.min_distance_mm))
        {
            best_segment.start_index = seg_start;
            best_segment.end_index = seg_end;
            best_segment.center_index = (uint16_t)((seg_start + ((seg_points - 1U) / 2U)) % size);
            best_segment.min_distance_mm = seg_min_distance;
            best_segment.point_count = seg_points;
            best_found = 1U;
        }
    }

    if (best_found == 0U)
    {
        if (g_track_enemy_debug.last_reject_reason == TRACK_ENEMY_DEBUG_REJECT_NONE)
        {
            g_track_enemy_debug.last_reject_reason = TRACK_ENEMY_DEBUG_REJECT_NO_SEGMENT;
        }
        return 0U;
    }

    /* 将最优目标段转换成上层状态机直接可用的目标信息。 */
    enemy_info->is_found = 1U;
    enemy_info->distance_mm = best_segment.min_distance_mm;
    enemy_info->center_index = best_segment.center_index;
    enemy_info->point_count = best_segment.point_count;
    enemy_info->angle_deg = (int16_t)TrackEnemy_IndexToAngleDeg(best_segment.center_index);
    TrackEnemy_DebugRecordEnemyInfo(enemy_info);
    g_track_enemy_debug.last_reject_reason = TRACK_ENEMY_DEBUG_REJECT_NONE;

    return 1U;
}

void TrackEnemyTracker_Clear(TrackEnemyTracker *tracker)
{
    if (tracker == NULL)
    {
        return;
    }

    TrackEnemy_ClearEnemyInfo(&tracker->enemy_info);
    TrackEnemyTracker_ClearPending(tracker);
    tracker->update_tick = 0U;
    tracker->consecutive_found_count = 0U;
    tracker->last_update_found = 0U;
}

uint8_t TrackEnemyTracker_Update(TrackEnemyTracker *tracker, uint32_t now_tick)
{
    EnemyInfo detected_enemy_info; /* 本轮新识别到的目标；识别失败时不覆盖上一帧缓存。 */

    if (tracker == NULL)
    {
        return 0U;
    }

    ++g_track_enemy_debug.update_count;
    g_track_enemy_debug.update_tick = now_tick;
    g_track_enemy_debug.snapshot_ok = 0U;
    g_track_enemy_debug.find_ok = 0U;
    TrackEnemy_DebugClearCandidate();
    g_track_enemy_debug.enemy_info.is_found = 0U;
    g_track_enemy_debug.enemy_info.angle_deg = 0;
    g_track_enemy_debug.enemy_info.distance_mm = 0U;
    g_track_enemy_debug.enemy_info.center_index = 0U;
    g_track_enemy_debug.enemy_info.point_count = 0U;
    g_track_enemy_debug.valid_point_count = 0U;
    g_track_enemy_debug.masked_valid_point_count = 0U;
    g_track_enemy_debug.segment_count = 0U;
    g_track_enemy_debug.segment_overflow = 0U;

    if (TrackEnemy_GetLatestSnapshot(tracker->scan_distances, LIDAR_SCAN_POINT_COUNT) == 0U)
    {
        if (g_track_enemy_debug.last_reject_reason != TRACK_ENEMY_DEBUG_REJECT_TOO_MANY_ZERO)
        {
            g_track_enemy_debug.last_reject_reason = TRACK_ENEMY_DEBUG_REJECT_NO_SNAPSHOT;
            TrackEnemyTracker_ClearPending(tracker);
            tracker->last_update_found = 0U;
        }
        else if (tracker->pending_found != 0U &&
                 (uint32_t)(now_tick - tracker->pending_last_seen_tick) > TRACK_ENEMY_PENDING_STALE_MS)
        {
            TrackEnemyTracker_ClearPending(tracker);
        }
        return 0U;
    }

    g_track_enemy_debug.snapshot_ok = 1U;

    if (TrackEnemy_FindBasicTarget(tracker->scan_distances, LIDAR_SCAN_POINT_COUNT, &detected_enemy_info) == 0U)
    {
        TrackEnemyTracker_ClearPending(tracker);
        tracker->last_update_found = 0U;
        return 0U;
    }
    TaskTrainLog_NotifyEnemyHit();

    if (tracker->pending_found != 0U &&
        TrackEnemy_IsSameTargetForConfirm(&tracker->pending_enemy_info, &detected_enemy_info) != 0U)
    {
        tracker->pending_enemy_info = detected_enemy_info;
        tracker->pending_last_seen_tick = now_tick;
    }
    else
    {
        tracker->pending_enemy_info = detected_enemy_info;
        tracker->pending_start_tick = now_tick;
        tracker->pending_last_seen_tick = now_tick;
        tracker->pending_found = 1U;
    }

    if ((uint32_t)(now_tick - tracker->pending_start_tick) < TRACK_ENEMY_CONFIRM_TIME_MS)
    {
        return 0U;
    }

    if (tracker->enemy_info.is_found != 0U &&
        TrackEnemy_IsSameTargetForConfirm(&tracker->enemy_info, &detected_enemy_info) != 0U)
    {
        if (tracker->consecutive_found_count < 255U)
        {
            ++tracker->consecutive_found_count;
        }
    }
    else
    {
        tracker->consecutive_found_count = 1U;
    }

    tracker->enemy_info = detected_enemy_info;
    tracker->last_update_found = 1U;
    g_track_enemy_debug.find_ok = 1U;
    tracker->update_tick = now_tick;
    TaskTrainLog_NotifyEnemyFound();
    return 1U;
}

uint8_t TrackEnemyTracker_IsFresh(TrackEnemyTracker *tracker, uint32_t now_tick, uint32_t timeout_ms)
{
    if (tracker == NULL)
    {
        return 0U;
    }

    if (tracker->enemy_info.is_found == 0U)
    {
        return 0U;
    }

    if ((now_tick - tracker->update_tick) > timeout_ms)
    {
        /* 超时只判为不新鲜，不清空目标；攻击态会用 update_tick 做短时保活。 */
        return 0U;
    }

    return 1U;
}
