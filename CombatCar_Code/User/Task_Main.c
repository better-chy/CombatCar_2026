#include "Task_Main.h"

#include "cmsis_os.h"
#include <math.h>
#include "Motor.h"
#include "Task_Danger.h"
#include "LaserRange.h"
#include "LiDAR.h"
#include "Vision.h"

#define LASER_FRONT 0U /* 前向测距模块编号。 */
#define LASER_RIGHT 1U /* 右向测距模块编号。 */
#define LASER_BACK  2U /* 后向测距模块编号。 */
#define LASER_LEFT  3U /* 左向测距模块编号。 */

#define ENEMY_SEARCH_START_DEG   (-90) /* 索敌扇区起始角，负值表示车头左侧。 */
#define ENEMY_SEARCH_END_DEG      (90) /* 索敌扇区结束角，正值表示车头右侧。 */
#define ENEMY_SEARCH_STEP_DEG   LIDAR_SCAN_ANGLE_STEP_DEG /* 按雷达原生角分辨率进行索敌采样。 */
#define ENEMY_SEARCH_STEP_CDEG   72U /* 索敌步长对应的 0.01 度整数值。 */
#define ENEMY_POINT_JUMP_MM       70U /* 相邻点距离跳变超过该值时视为新目标段。 */
#define ENEMY_MIN_DISTANCE_MM     120U /* 目标段允许的最小距离。 */
#define ENEMY_MAX_DISTANCE_MM    2500U /* 目标段允许的最大距离。 */
#define ENEMY_MIN_POINTS            3U /* 一个目标段至少需要的连续点数。 */
#define ENEMY_MAX_POINTS           80U /* 一个目标段最多允许的连续点数。 */
#define ENEMY_MIN_WIDTH_MM       80.0f /* 敌方目标允许的最小估算宽度。 */
#define ENEMY_MAX_WIDTH_MM       520.0f /* 敌方目标允许的最大估算宽度。 */
#define ENEMY_WALL_MIN_WIDTH_MM  320.0f /* 超过该宽度的目标段开始按墙体特征检查。 */
#define ENEMY_LINE_MAX_ERROR_MM   45.0f /* 墙体拟合时允许的最大点到线误差。 */
#define ENEMY_ATTACK_ANGLE_DEG     12.0f /* 切入攻击前允许的目标角度偏差。 */
#define ENEMY_ATTACK_DISTANCE_MM  550U /* 切入攻击状态的距离阈值。 */
#define ENEMY_SCAN_POINT_CAP    ((uint16_t)((((ENEMY_SEARCH_END_DEG - ENEMY_SEARCH_START_DEG) * 100) / ENEMY_SEARCH_STEP_CDEG) + 1U)) /* 一次索敌扫描最多缓存的点数。 */
#define TASKMAIN_PI              3.1415926f /* 局部几何计算使用的圆周率。 */

typedef enum
{
    TEAM_UNKNOWN = 0,
    TEAM_BLUE,
    TEAM_YELLOW
} TEAM;

#define FRIENDLY_BLOCK_BLUE_TAG_ID          1U /* 蓝方己方能量块的 AprilTag 编号。 */
#define FRIENDLY_BLOCK_YELLOW_TAG_ID        2U /* 黄方己方能量块的 AprilTag 编号。 */
#define FRIENDLY_BLOCK_TAG_MAX_AGE_MS     120U /* 视觉目标数据允许的最长保鲜时间。 */
#define FRIENDLY_BLOCK_TRIGGER_HOLD_MS     80U /* 连续看到己方能量块多久后触发避让。 */
#define FRIENDLY_BLOCK_BACK_SPEED        (-320) /* 避让己方能量块时的后退速度。 */
#define FRIENDLY_BLOCK_BACK_TIME_MS      180U /* 避让己方能量块时的后退持续时间。 */
#define FRIENDLY_BLOCK_TURN_LEFT_SPEED   (-320) /* 避让转向时左轮速度。 */
#define FRIENDLY_BLOCK_TURN_RIGHT_SPEED    320 /* 避让转向时右轮速度。 */
#define FRIENDLY_BLOCK_TURN_TIME_MS      180U /* 避让转向动作持续时间。 */

static TEAM team = TEAM_UNKNOWN; /* 当前己方阵营。 */

typedef struct
{
    uint8_t valid;         /* 当前目标信息是否有效。 */
    float angle_deg;       /* 目标中心相对车头的角度。 */
    uint16_t distance_mm;  /* 目标最近距离。 */
    uint8_t point_count;   /* 目标段包含的连续点数。 */
} EnemyTarget;

/* ==================== 运行时结构体 ==================== */

typedef struct
{
    CarState current_state;   // 当前主状态
    CarState previous_state;  // 上一个主状态
    uint32_t state_enter_tick; /* 进入当前状态时的系统时刻。 */
    uint32_t state_loop_count; /* 当前状态已执行的循环次数。 */
} CarStateManager;

static CarStateManager state_manager; /* 主状态机运行时信息。 */
static EnemyTarget enemy_target; /* 当前选中的敌方目标。 */

/* ==================== 内部函数声明 ==================== */

static void TaskMain_RunState(void);
static void TaskMain_RunInit(void);
static void TaskMain_RunOffStageSearch(void);
static void TaskMain_RunReenter(void);
static void TaskMain_RunPatrol(void);
static void TaskMain_RunTrack(void);
static void TaskMain_RunAttack(void);
static void TaskMain_RunAvoidFriendlyBlock(void);
static void TaskMain_RunEscape(void);
static void TaskMain_UpdateEnemyTarget(void);
static void TaskMain_ClearEnemyTarget(void);
static float TaskMain_NormalizeAngleDeg(float angle_deg);
static uint16_t TaskMain_GetAngleDistance(float angle_deg);
static float TaskMain_ComputeSegmentWidthMm(const float *angles_deg, const uint16_t *distances_mm, uint8_t point_count);
static uint8_t TaskMain_IsSegmentWallLike(const float *angles_deg, const uint16_t *distances_mm, uint8_t point_count, float width_mm);
static void TaskMain_UpdateBestSegment(const float *angles_deg,
                                       const uint16_t *distances_mm,
                                       uint8_t point_count,
                                       uint16_t *best_distance,
                                       float *best_angle,
                                       uint8_t *best_points);
static uint8_t TaskMain_GetFriendlyTagId(uint8_t *tag_id);
static uint8_t TaskMain_IsFriendlyBlockVisible(void);
static uint8_t TaskMain_TryEnterFriendlyBlockAvoid(void);

static uint32_t friendly_block_seen_tick = 0U; /* 首次连续看到己方能量块的时间戳。 */

/* ==================== 主状态机辅助函数 ==================== */

/* 切换主状态并刷新进入该状态的计时信息。 */
void TaskMain_SwitchState(CarState new_state)
{
    if (state_manager.current_state == new_state)
    {
        return;
    }
    
    state_manager.previous_state = state_manager.current_state;
    state_manager.current_state = new_state;
    state_manager.state_enter_tick = osKernelGetTickCount();
    state_manager.state_loop_count = 0U;
}

/* 根据当前主状态分发到对应的处理函数。 */
static void TaskMain_RunState(void)
{
    ++state_manager.state_loop_count;

    switch (state_manager.current_state)
    {
    case STATE_INIT:
        TaskMain_RunInit();
        break;

    case STATE_OFF_STAGE_SEARCH:
        TaskMain_RunOffStageSearch();
        break;

    case STATE_REENTER_STAGE:
        TaskMain_RunReenter();
        break;

    case STATE_PATROL:
        TaskMain_RunPatrol();
        break;

    case STATE_TRACK:
        TaskMain_RunTrack();
        break;

    case STATE_ATTACK:
        TaskMain_RunAttack();
        break;

    case STATE_AVOID_FRIENDLY_BLOCK:
        TaskMain_RunAvoidFriendlyBlock();
        break;

    case STATE_EDGE_ESCAPE:
        TaskMain_RunEscape();
        break;

    default:
        TaskMain_SwitchState(STATE_INIT);
        break;
    }
}

/* ==================== 不同状态处理函数 ==================== */

/* 读取左右测距判断己方阵营，并完成首次上台后的状态切换。 */
static void TaskMain_RunInit(void)
{
    uint16_t left_mm = LaserRange_GetDistanceMm(LASER_LEFT);   /* 左侧测距值，用于判断蓝方起始位。 */
    uint16_t right_mm = LaserRange_GetDistanceMm(LASER_RIGHT); /* 右侧测距值，用于判断黄方起始位。 */

    if (left_mm > 0 && left_mm < 100)
    {
        team = TEAM_BLUE;

        //第一次上台
        Run_Back(-500);
        osDelay(300);

        TaskMain_SwitchState(STATE_PATROL);
    }
    else if (right_mm > 0 && right_mm < 100)
    {
        team = TEAM_YELLOW;

        Run_Back(-500);
        osDelay(300);

        TaskMain_SwitchState(STATE_PATROL);
    }
    else
    {
        return;
    }
}

/* 处理掉台后的台下搜索流程。 */
static void TaskMain_RunOffStageSearch(void)
{
    //参数实地去调
}

/* 处理重新登台阶段的动作控制。 */
static void TaskMain_RunReenter(void)
{
    //参数等实地去调
}

/* 巡逻前进并持续搜索可攻击目标。 */
static void TaskMain_RunPatrol(void)
{
    if (TaskMain_TryEnterFriendlyBlockAvoid() != 0U)
    {
        return;
    }

    TaskMain_UpdateEnemyTarget();

    if (enemy_target.valid != 0U)
    {
        TaskMain_SwitchState(STATE_TRACK);
        return;
    }

    Run_Forward(320);
}

/* 对准已发现目标并向攻击距离逼近。 */
static void TaskMain_RunTrack(void)
{
    if (TaskMain_TryEnterFriendlyBlockAvoid() != 0U)
    {
        return;
    }

    TaskMain_UpdateEnemyTarget();

    if (enemy_target.valid == 0U)
    {
        TaskMain_SwitchState(STATE_PATROL);
        return;
    }

    if (enemy_target.angle_deg > ENEMY_ATTACK_ANGLE_DEG)
    {
        Turn_Right(280, 120);
        return;
    }

    if (enemy_target.angle_deg < -ENEMY_ATTACK_ANGLE_DEG)
    {
        Turn_Left(120, 280);
        return;
    }

    if (enemy_target.distance_mm <= ENEMY_ATTACK_DISTANCE_MM)
    {
        TaskMain_SwitchState(STATE_ATTACK);
        return;
    }

    Run_Forward(280);
}

/* 在近距离正对目标时执行冲击攻击。 */
static void TaskMain_RunAttack(void)
{
    if (TaskMain_TryEnterFriendlyBlockAvoid() != 0U)
    {
        return;
    }

    TaskMain_UpdateEnemyTarget();

    if (enemy_target.valid == 0U)
    {
        TaskMain_SwitchState(STATE_PATROL);
        return;
    }

    if (enemy_target.angle_deg > 20.0f || enemy_target.angle_deg < -20.0f)
    {
        TaskMain_SwitchState(STATE_TRACK);
        return;
    }

    Run_Forward(500);
}

/* 避让己方能量块，先后退再转向脱离。 */
static void TaskMain_RunAvoidFriendlyBlock(void)
{
    uint32_t elapsed_ms = osKernelGetTickCount() - state_manager.state_enter_tick; /* 进入避让状态后的累计时间。 */

    if (elapsed_ms < FRIENDLY_BLOCK_BACK_TIME_MS)
    {
        Run_Back(FRIENDLY_BLOCK_BACK_SPEED);
        return;
    }

    if (elapsed_ms < (FRIENDLY_BLOCK_BACK_TIME_MS + FRIENDLY_BLOCK_TURN_TIME_MS))
    {
        Turn_Left(FRIENDLY_BLOCK_TURN_LEFT_SPEED, FRIENDLY_BLOCK_TURN_RIGHT_SPEED);
        return;
    }

    friendly_block_seen_tick = 0U;
    TaskMain_SwitchState(STATE_PATROL);
}

/* 触发边缘危险后执行统一的脱困动作。 */
static void TaskMain_RunEscape(void)
{
    switch (edge_danger_type)
    {
    case EDGE_LEFT:
        Run_Back(-350);
        osDelay(200);

        Turn_Left(-360, 360);
        osDelay(165);

        edge_danger_type = EDGE_NONE;
        TaskMain_SwitchState(state_manager.previous_state);
        break;

    case EDGE_FRONT:
        Run_Back(-350);
        osDelay(200);

        Turn_Left(-360, 360);
        osDelay(165);

        edge_danger_type = EDGE_NONE;
        TaskMain_SwitchState(state_manager.previous_state);
        break;

    case EDGE_RIGHT:
        Run_Back(-350);
        osDelay(200);

        Turn_Left(-360, 360);
        osDelay(165);

        edge_danger_type = EDGE_NONE;
        TaskMain_SwitchState(state_manager.previous_state);
        break;

    default:
        edge_danger_type = EDGE_NONE;
        TaskMain_SwitchState(state_manager.previous_state);
        break;
    }
}

//==================索敌部分代码=================================================================

/* 清空当前缓存的敌方目标信息。 */
static void TaskMain_ClearEnemyTarget(void)
{
    enemy_target.valid = 0U;
    enemy_target.angle_deg = 0.0f;
    enemy_target.distance_mm = 0U;
    enemy_target.point_count = 0U;
}

/* 将任意角度归一化到 -180 到 180 度。 */
static float TaskMain_NormalizeAngleDeg(float angle_deg)
{
    while (angle_deg > 180)
    {
        angle_deg -= 360;
    }

    while (angle_deg < -180)
    {
        angle_deg += 360;
    }

    return angle_deg;
}

/* 按车体角度读取对应方向的激光雷达距离。 */
static uint16_t TaskMain_GetAngleDistance(float angle_deg)
{
    float normalized = TaskMain_NormalizeAngleDeg(angle_deg); /* 归一化后的车体角度。 */
    float lidar_angle = (normalized >= 0.0f) ? normalized : (360.0f + normalized); /* 转成 0 到 360 度表示。 */
    return LiDAR_GetDistanceByAngle(lidar_angle);
}

/* 根据角跨度和平均距离估算目标段的横向宽度。 */
static float TaskMain_ComputeSegmentWidthMm(const float *angles_deg, const uint16_t *distances_mm, uint8_t point_count)
{
    uint32_t distance_sum = 0U; /* 目标段内所有点距离总和。 */
    float span_deg; /* 目标段两端之间的角跨度。 */
    float mean_distance_mm; /* 目标段平均距离。 */
    uint8_t i; /* 循环遍历目标段点索引。 */

    if (angles_deg == NULL || distances_mm == NULL || point_count == 0U)
    {
        return 0.0f;
    }

    for (i = 0U; i < point_count; ++i)
    {
        distance_sum += distances_mm[i];
    }

    mean_distance_mm = (float)distance_sum / (float)point_count;
    span_deg = (angles_deg[point_count - 1U] - angles_deg[0]) + ENEMY_SEARCH_STEP_DEG;

    return mean_distance_mm * span_deg * (TASKMAIN_PI / 180.0f);
}

/* 判断目标段是否更像一段宽而直的墙体回波。 */
static uint8_t TaskMain_IsSegmentWallLike(const float *angles_deg, const uint16_t *distances_mm, uint8_t point_count, float width_mm)
{
    float start_x; /* 目标段起点的笛卡尔 X 坐标。 */
    float start_y; /* 目标段起点的笛卡尔 Y 坐标。 */
    float end_x; /* 目标段终点的笛卡尔 X 坐标。 */
    float end_y; /* 目标段终点的笛卡尔 Y 坐标。 */
    float dx; /* 首尾连线的 X 方向增量。 */
    float dy; /* 首尾连线的 Y 方向增量。 */
    float line_len; /* 首尾连线长度。 */
    uint8_t i; /* 遍历中间点的索引。 */

    if (angles_deg == NULL || distances_mm == NULL || point_count < 3U)
    {
        return 0U;
    }

    if (width_mm < ENEMY_WALL_MIN_WIDTH_MM)
    {
        return 0U;
    }

    start_x = (float)distances_mm[0] * cosf(angles_deg[0] * (TASKMAIN_PI / 180.0f));
    start_y = (float)distances_mm[0] * sinf(angles_deg[0] * (TASKMAIN_PI / 180.0f));
    end_x = (float)distances_mm[point_count - 1U] * cosf(angles_deg[point_count - 1U] * (TASKMAIN_PI / 180.0f));
    end_y = (float)distances_mm[point_count - 1U] * sinf(angles_deg[point_count - 1U] * (TASKMAIN_PI / 180.0f));
    dx = end_x - start_x;
    dy = end_y - start_y;
    line_len = sqrtf(dx * dx + dy * dy);

    if (line_len < 1.0f)
    {
        return 0U;
    }

    for (i = 1U; i < (uint8_t)(point_count - 1U); ++i)
    {
        float point_x = (float)distances_mm[i] * cosf(angles_deg[i] * (TASKMAIN_PI / 180.0f)); /* 当前点的 X 坐标。 */
        float point_y = (float)distances_mm[i] * sinf(angles_deg[i] * (TASKMAIN_PI / 180.0f)); /* 当前点的 Y 坐标。 */
        float error_mm = fabsf(dy * point_x - dx * point_y + end_x * start_y - end_y * start_x) / line_len; /* 当前点到首尾连线的垂距。 */

        if (error_mm > ENEMY_LINE_MAX_ERROR_MM)
        {
            return 0U;
        }
    }

    return 1U;
}

/* 用宽度和直线特征筛选候选段，并更新当前最优目标。 */
static void TaskMain_UpdateBestSegment(const float *angles_deg,
                                       const uint16_t *distances_mm,
                                       uint8_t point_count,
                                       uint16_t *best_distance,
                                       float *best_angle,
                                       uint8_t *best_points)
{
    uint16_t seg_min_distance; /* 当前候选段中的最近距离。 */
    float width_mm; /* 当前候选段的估算宽度。 */
    uint8_t i; /* 遍历候选段点的索引。 */

    if (angles_deg == NULL || distances_mm == NULL || best_distance == NULL || best_angle == NULL || best_points == NULL)
    {
        return;
    }

    if (point_count < ENEMY_MIN_POINTS || point_count > ENEMY_MAX_POINTS)
    {
        return;
    }

    width_mm = TaskMain_ComputeSegmentWidthMm(angles_deg, distances_mm, point_count);
    if (width_mm < ENEMY_MIN_WIDTH_MM || width_mm > ENEMY_MAX_WIDTH_MM)
    {
        return;
    }

    if (TaskMain_IsSegmentWallLike(angles_deg, distances_mm, point_count, width_mm) != 0U)
    {
        return;
    }

    seg_min_distance = distances_mm[0];
    for (i = 1U; i < point_count; ++i)
    {
        if (distances_mm[i] < seg_min_distance)
        {
            seg_min_distance = distances_mm[i];
        }
    }

    if (seg_min_distance < *best_distance)
    {
        *best_distance = seg_min_distance;
        *best_angle = ((float)angles_deg[0] + (float)angles_deg[point_count - 1U]) * 0.5f;
        *best_points = point_count;
    }
}

/* 根据己方阵营返回对应的己方能量块 Tag 编号。 */
static uint8_t TaskMain_GetFriendlyTagId(uint8_t *tag_id)
{
    if (tag_id == NULL)
    {
        return 0U;
    }

    switch (team)
    {
    case TEAM_BLUE:
        *tag_id = FRIENDLY_BLOCK_BLUE_TAG_ID;
        return 1U;

    case TEAM_YELLOW:
        *tag_id = FRIENDLY_BLOCK_YELLOW_TAG_ID;
        return 1U;

    default:
        return 0U;
    }
}

/* 检查视觉模块当前是否稳定看到了己方能量块。 */
static uint8_t TaskMain_IsFriendlyBlockVisible(void)
{
    uint8_t friendly_tag_id; /* 当前阵营对应的己方能量块 Tag 编号。 */
    Vision_Tag tag; /* 视觉模块最新输出的 Tag 数据。 */
    uint32_t now = osKernelGetTickCount(); /* 当前系统时刻。 */

    if (Vision_IsOnline(FRIENDLY_BLOCK_TAG_MAX_AGE_MS) == 0U)
    {
        return 0U;
    }

    if (Vision_GetLatestTag(&tag) == 0U)
    {
        return 0U;
    }

    if ((now - tag.update_tick) > FRIENDLY_BLOCK_TAG_MAX_AGE_MS)
    {
        return 0U;
    }

    if ((tag.flags & (VISION_FLAG_DETECTED | VISION_FLAG_ID_VALID)) !=
        (VISION_FLAG_DETECTED | VISION_FLAG_ID_VALID))
    {
        return 0U;
    }

    if (TaskMain_GetFriendlyTagId(&friendly_tag_id) == 0U)
    {
        return 0U;
    }

    return (uint8_t)(tag.tag_id == friendly_tag_id);
}

/* 满足持续观测条件后切入己方能量块避让状态。 */
static uint8_t TaskMain_TryEnterFriendlyBlockAvoid(void)
{
    uint32_t now = osKernelGetTickCount(); /* 当前系统时刻。 */

    if (TaskMain_IsFriendlyBlockVisible() == 0U)
    {
        friendly_block_seen_tick = 0U;
        return 0U;
    }

    if (friendly_block_seen_tick == 0U)
    {
        friendly_block_seen_tick = now;
        return 0U;
    }

    if ((now - friendly_block_seen_tick) < FRIENDLY_BLOCK_TRIGGER_HOLD_MS)
    {
        return 0U;
    }

    TaskMain_SwitchState(STATE_AVOID_FRIENDLY_BLOCK);
    return 1U;
}

/* 从激光雷达扫描中提取最可信的敌方目标段。 */
static void TaskMain_UpdateEnemyTarget(void)
{
    uint16_t scan_index; /* 当前扫描到的角度采样索引。 */
    float angle_deg; /* 当前扫描到的车体角度。 */
    uint16_t prev_distance = 0U; /* 当前目标段上一点的距离。 */
    uint8_t in_segment = 0U; /* 当前是否处于一个连续目标段内。 */
    float segment_angles[ENEMY_SCAN_POINT_CAP]; /* 当前目标段缓存的所有角度。 */
    uint16_t segment_distances[ENEMY_SCAN_POINT_CAP]; /* 当前目标段缓存的所有距离。 */
    uint8_t seg_points = 0U; /* 当前目标段累计的连续点数。 */
    uint16_t best_distance = 0xFFFFU; /* 本圈扫描中最佳目标的最近距离。 */
    float best_angle = 0.0f; /* 本圈扫描中最佳目标的中心角。 */
    uint8_t best_points = 0U; /* 本圈扫描中最佳目标的点数。 */

    if (LiDAR_IsScanReady() == 0U)
    {
        return;
    }

    LiDAR_ClearScanReady();
    TaskMain_ClearEnemyTarget();

    for (scan_index = 0U; scan_index < ENEMY_SCAN_POINT_CAP; ++scan_index)
    {
        angle_deg = (float)ENEMY_SEARCH_START_DEG + ((float)scan_index * ENEMY_SEARCH_STEP_DEG);
        uint16_t distance_mm = TaskMain_GetAngleDistance(angle_deg); /* 当前角度对应的雷达距离。 */
        uint8_t point_valid = (uint8_t)(distance_mm >= ENEMY_MIN_DISTANCE_MM && distance_mm <= ENEMY_MAX_DISTANCE_MM); /* 当前点是否落在有效距离窗口内。 */

        if (point_valid == 0U)
        {
            if (in_segment != 0U)
            {
                TaskMain_UpdateBestSegment(segment_angles, segment_distances, seg_points, &best_distance, &best_angle, &best_points);
                in_segment = 0U;
                seg_points = 0U;
            }
            continue;
        }

        if (in_segment == 0U)
        {
            in_segment = 1U;
            segment_angles[0] = angle_deg;
            segment_distances[0] = distance_mm;
            seg_points = 1U;
            prev_distance = distance_mm;
            continue;
        }

        if ((distance_mm > prev_distance ? (distance_mm - prev_distance) : (prev_distance - distance_mm)) > ENEMY_POINT_JUMP_MM)
        {
            TaskMain_UpdateBestSegment(segment_angles, segment_distances, seg_points, &best_distance, &best_angle, &best_points);
            segment_angles[0] = angle_deg;
            segment_distances[0] = distance_mm;
            seg_points = 1U;
            prev_distance = distance_mm;
            continue;
        }

        if (seg_points < (uint8_t)ENEMY_SCAN_POINT_CAP)
        {
            segment_angles[seg_points] = angle_deg;
            segment_distances[seg_points] = distance_mm;
            ++seg_points;
        }
        prev_distance = distance_mm;
    }

    if (in_segment != 0U)
    {
        TaskMain_UpdateBestSegment(segment_angles, segment_distances, seg_points, &best_distance, &best_angle, &best_points);
    }

    if (best_distance != 0xFFFFU)
    {
        enemy_target.valid = 1U;
        enemy_target.angle_deg = best_angle;
        enemy_target.distance_mm = best_distance;
        enemy_target.point_count = best_points;
    }
}

//=======================================================

/* 主任务入口，初始化状态机并周期运行主逻辑。 */
void Task_Main_Run(void *argument)
{
    (void)argument;

    state_manager.current_state = STATE_INIT;
    state_manager.previous_state = STATE_INIT;
    state_manager.state_enter_tick = osKernelGetTickCount();
    state_manager.state_loop_count = 0U;
    TaskMain_ClearEnemyTarget();

    osDelay(1500);

    for (;;)
    {
        TaskMain_RunState();
        osDelay(1);
    }
}
