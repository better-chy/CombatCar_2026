#ifndef __TRACK_ENEMY_H
#define __TRACK_ENEMY_H

#include "main.h"
#include "LiDAR.h"

#define TRACK_ENEMY_PI                       3.1415926f /* 局部几何计算使用的圆周率。 */

/* 屏蔽遮挡区参数。
 *
 * 这些角度是车体结构或雷达安装导致的不可用区域，不参与索敌。
 *
 * LiDAR 原始索引换算公式见 TrackEnemy_IndexToAngleDeg：
 * angle_deg = Normalize((375 - index) * 0.72)。
 *
 * 当前屏蔽区对应原始索引：
 * 后方负角 -180~-126 deg：50~124；
 * 后方正角 126~180 deg：125~200；
 * 右前遮挡 39~49 deg：307~320；
 * 左前遮挡 -49~-39 deg：430~443。
 */
#define TRACK_ENEMY_REAR_MASK_MIN_DEG        126.0f /* 后方遮挡区正角起点。 */
#define TRACK_ENEMY_REAR_MASK_MAX_DEG        180.0f /* 后方遮挡区正角终点。 */
#define TRACK_ENEMY_REAR_MASK_NEG_MIN_DEG   -180.0f /* 后方遮挡区负角起点。 */
#define TRACK_ENEMY_REAR_MASK_NEG_MAX_DEG   -126.0f /* 后方遮挡区负角终点。 */
#define TRACK_ENEMY_FRONT_RIGHT_MASK_MIN_DEG   39.0f /* 右前遮挡区起点。 */
#define TRACK_ENEMY_FRONT_RIGHT_MASK_MAX_DEG   49.0f /* 右前遮挡区终点。 */
#define TRACK_ENEMY_FRONT_LEFT_MASK_MIN_DEG   -49.0f /* 左前遮挡区起点。 */
#define TRACK_ENEMY_FRONT_LEFT_MASK_MAX_DEG   -39.0f /* 左前遮挡区终点。 */
#define TRACK_ENEMY_ZERO_PATCH_MAX_GAP          2U  /* 允许修补的连续 0 距离最大点数。 */

/* 索敌线段筛选 */
#define TRACK_ENEMY_MIN_DISTANCE_MM          80U   /* 索敌允许的最小有效距离。 */
#define TRACK_ENEMY_MAX_DISTANCE_MM         4000U   /* 索敌允许的最大有效距离。 */
#define TRACK_ENEMY_MAX_ZERO_POINTS           40U   /* 一圈原始雷达 0 点总数超过该值时，本轮拒绝索敌。 */
#define TRACK_ENEMY_MAX_ZERO_RUN_POINTS        6U   /* 非屏蔽区连续 0 点超过该值时，本轮拒绝索敌。 */
#define TRACK_ENEMY_POINT_JUMP_MM            140U   /* 相邻点距离跳变超过该值时视为新目标段。 */
#define TRACK_ENEMY_MIN_POINTS                 5U   /* 一个候选目标段至少需要的连续点数。 */
#define TRACK_ENEMY_MASK_EDGE_MIN_POINTS       5U   /* 靠近屏蔽区边界的候选段允许的最小连续点数。 */
#define TRACK_ENEMY_MASK_EDGE_MARGIN_INDEX     5U   /* 候选段距离屏蔽区该索引数以内时启用屏蔽区边缘特例。 */
#define TRACK_ENEMY_BACKGROUND_JUMP_MM       700U   /* 普通候选段要求双侧跳变；屏蔽区边缘候选段只要求远离屏蔽区一侧跳变。 */
#define TRACK_ENEMY_BACKGROUND_MIN_RUN_POINTS 2U    /* 背景跳变至少需要连续若干个远点支撑，过滤空场地单点尖刺。 */
#define TRACK_ENEMY_BACKGROUND_SKIP_POINTS     2U   /* 搜索背景时允许跳过的孤立近点数量，避免单点误差吃掉背景跳变。 */
#define TRACK_ENEMY_BACKGROUND_MAX_ZERO_GAP    2U   /* 搜索背景时允许跨过的连续 0 点数量，超过则认为背景不可用。 */
#define TRACK_ENEMY_CONFIRM_TIME_MS           50U   /* 候选目标连续存在该时间后才真正锁定；0 点跳帧不打断计时。 */
#define TRACK_ENEMY_PENDING_STALE_MS         120U   /* 候选目标超过该时间没有真实命中时，即使中间都是 0 点跳帧也清空。 */
#define TRACK_ENEMY_MAX_WIDTH_MM           600.0f  /* 候选目标段允许的最大估算宽度。 */
#define TRACK_ENEMY_LOW_PROFILE_ENABLE        0U   /* 低矮目标 2 点特例开关；0 表示暂时关闭。 */
#define TRACK_ENEMY_LOW_PROFILE_MIN_POINTS     2U   /* 低矮目标特例允许的最小连续点数。 */
#define TRACK_ENEMY_LOW_PROFILE_MAX_ABS_ANGLE_DEG 30.0f /* 低矮目标特例只在车头附近启用。 */
#define TRACK_ENEMY_LOW_PROFILE_MIN_DISTANCE_MM 500U /* 低矮目标特例允许的最近距离下限，过滤车体近场噪声。 */
#define TRACK_ENEMY_LOW_PROFILE_MAX_DISTANCE_MM 2500U /* 低矮目标特例允许的最近距离上限。 */
#define TRACK_ENEMY_LOW_PROFILE_BACKGROUND_JUMP_MM 800U /* 低矮目标特例要求更强的双侧背景跳变。 */
#define TRACK_ENEMY_LOW_PROFILE_MAX_WIDTH_MM 160.0f /* 低矮目标特例允许的最大估算宽度。 */
#define TRACK_ENEMY_SCAN_START_INDEX         125U   /* 环形扫描起点，放在后方遮挡区内以避免拼接缝切断目标。 */

/* Debug 容量与拒绝原因 */
#define TRACK_ENEMY_DEBUG_MAX_SEGMENTS        32U   /* debug 中保留的一圈候选线段最大数量。 */

#define TRACK_ENEMY_DEBUG_REJECT_NONE          0U   /* 已经找到目标，或还没有拒绝原因。 */
#define TRACK_ENEMY_DEBUG_REJECT_NO_SNAPSHOT   1U   /* 没拿到有效 LiDAR 快照。 */
#define TRACK_ENEMY_DEBUG_REJECT_NO_SEGMENT    2U   /* 没有形成可用候选段。 */
#define TRACK_ENEMY_DEBUG_REJECT_POINT_COUNT   3U   /* 候选段连续点数不足。 */
#define TRACK_ENEMY_DEBUG_REJECT_WIDTH         4U   /* 候选段估算宽度过大。 */
#define TRACK_ENEMY_DEBUG_REJECT_BACKGROUND_JUMP 5U /* 候选段与背景距离跳变不足。 */
#define TRACK_ENEMY_DEBUG_REJECT_TOO_MANY_ZERO 6U   /* 原始雷达 0 点过多，本轮不索敌。 */

typedef struct
{
    uint8_t is_found;
    int16_t angle_deg;
    uint16_t distance_mm;
    uint16_t center_index;
    uint16_t point_count;
} EnemyInfo;

typedef struct
{
    EnemyInfo enemy_info;
    EnemyInfo pending_enemy_info; /* 正在等待 50ms 连续确认的候选目标。 */
    uint16_t scan_distances[LIDAR_SCAN_POINT_COUNT];
    uint32_t update_tick;
    uint32_t pending_start_tick; /* 候选目标第一次出现的 tick。 */
    uint32_t pending_last_seen_tick; /* 候选目标最近一次真实命中的 tick。 */
    uint8_t consecutive_found_count; /* 最近一次成功锁定前连续命中的帧数。 */
    uint8_t last_update_found; /* 上一次索敌刷新是否成功找到目标。 */
    uint8_t pending_found; /* 当前是否存在未发布的候选目标。 */
} TrackEnemyTracker;

typedef struct
{
    uint8_t accepted; /* 该线段是否通过敌车筛选。 */
    uint8_t reject_reason; /* 未通过时的拒绝原因。 */
    uint16_t start_index; /* 线段起始原始索引。 */
    uint16_t end_index; /* 线段结束原始索引。 */
    uint16_t center_index; /* 线段中心原始索引。 */
    int16_t center_angle_deg; /* 线段中心角度。 */
    uint16_t min_distance_mm; /* 线段最近距离。 */
    uint16_t point_count; /* 线段连续点数。 */
    uint16_t min_points_required; /* 当前线段需要满足的最小点数。 */
    uint16_t width_mm; /* 线段估算宽度。 */
    uint16_t left_background_mm; /* 左侧背景距离。 */
    uint16_t right_background_mm; /* 右侧背景距离。 */
    uint16_t left_jump_mm; /* 左侧背景跳变。 */
    uint16_t right_jump_mm; /* 右侧背景跳变。 */
} TrackEnemyDebugSegment;

typedef struct
{
    uint32_t update_count; /* 索敌刷新次数。 */
    uint32_t update_tick; /* 最近一次刷新 tick。 */
    uint8_t snapshot_ok; /* 本轮是否拿到有效 LiDAR 快照。 */
    uint8_t find_ok; /* 本轮是否找到目标。 */
    uint8_t last_reject_reason; /* 最近一次候选段被拒绝的原因。 */
    uint16_t valid_point_count; /* 非屏蔽区内参与索敌的有效点数量。 */
    uint16_t masked_valid_point_count; /* 距离有效但落在屏蔽区内、被剔除的点数量。 */
    uint16_t candidate_start_index; /* 最近候选段起始原始索引。 */
    uint16_t candidate_end_index; /* 最近候选段结束原始索引。 */
    uint16_t candidate_center_index; /* 最近候选段中心原始索引。 */
    int16_t candidate_angle_deg; /* 最近候选段中心角度。 */
    uint16_t candidate_min_distance_mm; /* 最近候选段最近距离。 */
    uint16_t candidate_point_count; /* 最近候选段连续点数。 */
    uint16_t candidate_min_points_required; /* 最近候选段需要满足的最小点数。 */
    uint16_t candidate_width_mm; /* 最近候选段估算宽度。 */
    uint16_t candidate_left_background_mm; /* 最近候选段左侧背景距离。 */
    uint16_t candidate_right_background_mm; /* 最近候选段右侧背景距离。 */
    uint16_t candidate_left_jump_mm; /* 最近候选段左侧背景跳变。 */
    uint16_t candidate_right_jump_mm; /* 最近候选段右侧背景跳变。 */
    uint8_t segment_count; /* 本轮记录到的候选线段数量。 */
    uint8_t segment_overflow; /* 候选线段超过 debug 数组容量。 */
    TrackEnemyDebugSegment segments[TRACK_ENEMY_DEBUG_MAX_SEGMENTS]; /* 本轮所有观测到的候选线段。 */
    EnemyInfo enemy_info; /* 最近一次成功锁定的敌人信息。 */
} TrackEnemyDebugInfo;

extern volatile TrackEnemyDebugInfo g_track_enemy_debug;
extern TrackEnemyTracker g_enemy_tracker;

void TrackEnemy_InitTrigTable(void);
float TrackEnemy_NormalizeAngleDeg(float angle_deg);
uint8_t TrackEnemy_IsMaskedAngle(float angle_deg);
uint8_t TrackEnemy_IsMaskedIndex(uint16_t index);
void TrackEnemy_PatchZeroHoles(uint16_t *scan_distances, uint16_t size);
uint8_t TrackEnemy_GetPatchedSnapshot(uint16_t *scan_distances, uint16_t size);
uint8_t TrackEnemy_GetLatestSnapshot(uint16_t *scan_distances, uint16_t size);
uint8_t TrackEnemy_IsValidDistance(uint16_t distance_mm);
float TrackEnemy_IndexToAngleDeg(uint16_t index);
float TrackEnemy_ComputeSegmentWidthMm(const uint16_t *scan_distances, uint16_t start_index, uint16_t end_index);
void TrackEnemy_ClearEnemyInfo(EnemyInfo *enemy_info);
uint8_t TrackEnemy_FindBasicTarget(const uint16_t *scan_distances, uint16_t size, EnemyInfo *enemy_info);
void TrackEnemyTracker_Clear(TrackEnemyTracker *tracker);
uint8_t TrackEnemyTracker_Update(TrackEnemyTracker *tracker, uint32_t now_tick);
uint8_t TrackEnemyTracker_IsFresh(TrackEnemyTracker *tracker, uint32_t now_tick, uint32_t timeout_ms);

#endif
