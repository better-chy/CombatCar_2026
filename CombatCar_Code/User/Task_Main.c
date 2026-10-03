#include "Task_Main.h"

#include <math.h>
#include "cmsis_os.h"
#include "JY901S.h"
#include "LaserRange.h"
#include "LiDAR.h"
#include "Motor.h"
#include "PID.h"
#include "Task_Danger.h"
#include "Task_Stage.h"
#include "CombatCar_TuneConfig.h"
#include "Track_Enemy.h"
#include "Vision.h"

/* 调试开关 */
#define TASKMAIN_USE_FAR_EDGE_SENSORS       1U    /* 编译启用远处二级光电，实际硬保护由 TaskDanger_ShouldUseFarEdgeProtection 控制。 */

/* 基础速度 */
#define TASKMAIN_PATROL_SPEED                 180  /* 巡逻前进速度。 */
#define TASKMAIN_TRACK_FORWARD_SPEED          200  /* 跟踪阶段直行逼近速度。 */
#define TASKMAIN_ATTACK_SPEED                 700  /* 攻击阶段直冲速度。 */

/* 目标锁定与状态切换 */
#define TASKMAIN_TRACK_ENTER_ATTACK_ANGLE_DEG   20  /* TRACK 中目标进入该角度范围，允许切入 ATTACK。 */
#define TASKMAIN_ATTACK_FULL_SPEED_ANGLE_DEG     8  /* ATTACK 中目标保持在该角度范围，才允许 800 全速攻击。 */
#define TASKMAIN_ATTACK_STEER_ANGLE_DEG         25  /* ATTACK 中目标小幅偏离时，使用中速强修正继续压迫。 */
#define TASKMAIN_ATTACK_STEER_SPEED            400  /* ATTACK 中速强修正速度，兼顾前压和转向。 */
#define TASKMAIN_ATTACK_DISTANCE_MM          1400U  /* 进入攻击阶段的距离阈值。 */
#define TASKMAIN_TRACK_ENTER_ATTACK_DELAY_MS  150U  /* TRACK 对准后，连续满足攻击条件该时间才切入 ATTACK。 */
#define TASKMAIN_TARGET_SINGLE_HIT_HOLD_MS    100U  /* 目标只短暂出现一帧时，允许继续使用缓存目标的时间。 */
#define TASKMAIN_TARGET_CONFIRMED_HOLD_MS     400U  /* 目标连续命中或大角度缠斗时，允许继续使用缓存目标的时间。 */
#define TASKMAIN_TARGET_CONFIRM_HIT_COUNT       3U  /* 连续命中达到该帧数后，认为目标比较可信。 */
#define TASKMAIN_TARGET_LARGE_ANGLE_HOLD_DEG   35   /* 近距离大角度互顶时，允许更长保活来继续抢角度。 */
#define TASKMAIN_TO_STAGE_EDGE_BLIND_MS       500U  /* 上台动作结束后短暂屏蔽近/远光电，避免刚上台误触发。 */

/* 近距离纠缠 */
#define TASKMAIN_CLOSE_COMBAT_DISTANCE_MM    1000U  /* 近距离纠缠范围，在该距离内优先边走边急转。 */
#define TASKMAIN_CLOSE_COMBAT_SPEED           400  /* ATTACK 中 8~25 度近距离修正速度。 */
#define TASKMAIN_CLOSE_COMBAT_ANGLE_DEG        25  /* ATTACK 中允许 400 速度修正的最大偏角。 */

/* 跟踪阶段限幅 */
#define TASKMAIN_TRACK_LIMIT_TRIGGER_PWM      280  /* 跟踪阶段平均前进速度超过该值才触发限幅。 */
#define TASKMAIN_TRACK_LIMIT_PWM              300  /* 跟踪阶段触发限幅后，平均前进速度压到该值。 */
#define TASKMAIN_TRACK_TURN_LIMIT_PWM         700  /* 跟踪阶段限速后允许保留的最大转向分量。 */
#define TASKMAIN_TRACK_INNER_MIN_PWM           50  /* 跟踪阶段转向时内侧轮仍保持前进，避免原地超速旋转。 */
#define TASKMAIN_TRACK_ALIGN_SLOW_ANGLE_DEG    12  /* TRACK 中目标角度超过该值时，先减速对正。 */
#define TASKMAIN_TRACK_ALIGN_STOP_ANGLE_DEG    25  /* TRACK 中目标角度超过该值时，先停前进只转向。 */
#define TASKMAIN_TRACK_ALIGN_SLOW_SPEED        80  /* TRACK 中对正阶段的低速前进速度。 */
/* 跟踪角度 PID */
#define TASKMAIN_TRACK_PID_KP                3.5f  /* 跟踪阶段角度 PID 的比例系数。 */
#define TASKMAIN_TRACK_PID_KI                0.0f  /* 跟踪阶段角度 PID 的积分系数；索敌角度会跳变，先关闭积分。 */
#define TASKMAIN_TRACK_PID_KD                6.5f  /* 跟踪阶段角度 PID 的微分系数。 */
#define TASKMAIN_TRACK_PID_INTEGRAL_LIMIT  300.0f /* 跟踪阶段角度 PID 的积分限幅。 */
#define TASKMAIN_TRACK_PID_OUTPUT_LIMIT    400.0f /* 跟踪阶段角度 PID 的输出限幅。 */
#define TASKMAIN_ATTACK_PID_SCALE_PERCENT    200  /* ATTACK 中放大角度 PID 修正，提升高速修正能力。 */

/* 视觉与阵营 */
#define TASKMAIN_VISION_TIMEOUT_MS           300U  /* 视觉标签在线判定超时。 */
#define TASKMAIN_FRIENDLY_TAG_BLUE             1U  /* 规则：蓝方能量块 AprilTag ID。 */
#define TASKMAIN_FRIENDLY_TAG_YELLOW           2U  /* 规则：黄方能量块 AprilTag ID。 */

/* 发车初始化 */
#define TASKMAIN_INIT_IMU_TIMEOUT_MS        300U   /* 发车前要求 IMU 在线的超时阈值。 */
#define TASKMAIN_INIT_YAW_STABLE_DEG        2.0f   /* 连续两次 yaw 变化小于该值，认为姿态趋于稳定。 */
#define TASKMAIN_INIT_YAW_STABLE_COUNT      80U    /* 主循环 1ms 一次，连续稳定约 80ms 后才允许锁定发车姿态。 */
#define TASKMAIN_INIT_ROLL_STABLE_DEG       0.5f   /* INIT 阶段 angle[0] 连续变化小于该值，认为车身初始姿态稳定。 */
#define TASKMAIN_INIT_ROLL_STABLE_COUNT      80U   /* 主循环 1ms 一次，连续稳定约 80ms 后锁定 angle[0] 初始值。 */
#define TASKMAIN_INIT_ROLL_LOCK_TIMEOUT_MS 3000U   /* angle[0] 不够稳定时，最多等 3 秒后用最后一次读数锁定。 */
#define TASKMAIN_EDGE_TILT_DISABLE_DEG        5.0f /* ATTACK 中 angle[0] 相对初始值增加超过该角度时，允许动态屏蔽光电。 */
#define TASKMAIN_EDGE_TILT_GYRO_DPS          25.0f /* ATTACK 中 gyro[0] 大于该角速度时，认为正在被快速抬头。 */
#define TASKMAIN_NOSE_DOWN_STUCK_DEG        -5.0f  /* angle[0] 相对初始值减少超过该角度，认为车头卡在台沿。 */
#define TASKMAIN_NOSE_UP_STUCK_DEG          30.0f  /* angle[0] 相对初始值增加超过该角度，认为车头被长期顶起。 */
#define TASKMAIN_NOSE_DOWN_STUCK_MS        1000U   /* 车头下倾持续超过该时间后，强制前进脱困。 */
#define TASKMAIN_NOSE_UP_STUCK_MS          1000U   /* 车头长期被顶起持续超过该时间后，强制后退脱困。 */
#define TASKMAIN_NOSE_DOWN_FORWARD_SPEED    300    /* 车头卡台沿时的前进脱困速度。 */
#define TASKMAIN_NOSE_UP_BACK_SPEED        -300    /* 车头长期被顶起时的后退脱困速度。 */
#define TASKMAIN_NOSE_DOWN_EDGE_BLIND_MS    100U   /* 姿态脱困时短暂屏蔽近/远光电，避免 Task_Danger 抢回边缘脱困。 */
#define TASKMAIN_OFFSTAGE_STUCK_MS        2000U   /* 台下脱困时 2s 没有姿态/距离变化，认为卡住。 */
#define TASKMAIN_OFFSTAGE_UNSTICK_FORWARD_SPEED 150  /* 台下卡住后，前方空间更大时轻微前进。 */
#define TASKMAIN_OFFSTAGE_UNSTICK_BACK_SPEED   -150  /* 台下卡住后，后方空间更大或相等时轻微后退。 */
#define TASKMAIN_OFFSTAGE_UNSTICK_MS        200U   /* 台下卡住后轻微前进/后退的持续时间。 */
#define TASKMAIN_OFFSTAGE_GYRO_ACTIVE_DPS  10.0f   /* 陀螺仪任一轴角速度超过该值，认为车体还在运动。 */
#define TASKMAIN_OFFSTAGE_DISTANCE_MOVE_MM   80U   /* 台下关键距离相对锚点变化超过该值，认为脱困仍在推进。 */

typedef enum
{
    TEAM_UNKNOWN = 0,
    TEAM_BLUE,
    TEAM_YELLOW
} TEAM;

static TEAM team = TEAM_UNKNOWN; /* 当前己方阵营。 */
CarStateManager state_manager; /* 主状态机运行时信息。 */
TrackEnemyTracker g_enemy_tracker; /* 当前索敌模块运行时数据与输出目标，供 debug 直接观察。 */
static AnglePid track_angle_pid; /* 跟踪阶段用于对准目标角度的 PID。 */
static float s_init_last_yaw_deg = 0.0f; /* INIT 阶段上一轮看到的 yaw。 */
static uint16_t s_init_yaw_stable_count = 0U; /* INIT 阶段 yaw 已连续稳定的计数。 */
static uint8_t s_init_yaw_seen = 0U; /* INIT 阶段是否至少见过一次有效 yaw。 */
static float s_init_last_roll_deg = 0.0f; /* INIT 阶段上一轮看到的 angle[0]。 */
static uint16_t s_init_roll_stable_count = 0U; /* INIT 阶段 angle[0] 已连续稳定的计数。 */
static uint8_t s_init_roll_seen = 0U; /* INIT 阶段是否至少见过一次有效 angle[0]。 */
static uint32_t s_init_roll_wait_start_tick = 0U; /* INIT 阶段等待 angle[0] 稳定的起始 tick。 */
static float s_edge_roll_baseline_deg = 0.0f; /* 上电 INIT 中锁定的 angle[0] 基准。 */
static uint8_t s_edge_roll_baseline_valid = 0U; /* angle[0] 基准是否已经锁定。 */
static uint32_t s_track_attack_ready_start_tick = 0U; /* TRACK 中首次满足攻击条件的 tick，连续对准后才切 ATTACK。 */
static uint32_t s_angle0_stuck_start_tick = 0U; /* angle[0] 持续超出脱困阈值的起始 tick。 */
static int8_t s_angle0_stuck_direction = 0; /* -1 为车头下压，1 为车头上翘。 */
static uint32_t s_offstage_stuck_start_tick = 0U; /* 台下疑似卡住计时起点。 */
static StageType s_offstage_stuck_anchor_type = STAGE_ON_STAGE; /* 台下卡住检测的锚点标签。 */
static uint16_t s_offstage_stuck_anchor_distances[4] = {0U}; /* 台下卡住检测的四个小激光锚点。 */
static int s_offstage_parallel_turn_dir = 1; /* 1 左转，-1 右转；由最近一次四角脱困方向决定。 */

volatile uint8_t g_main_edge_tilt_bypass_active = 0U; /* 车头被翘起时，近/远光电保护是否被禁用。 */
volatile int16_t g_main_edge_roll_baseline_deg_x10 = 0; /* 锁定的 angle[0] 基准，放大 10 倍便于 debug。 */
volatile int16_t g_main_edge_roll_delta_deg_x10 = 0; /* 当前 angle[0] 相对基准差值，放大 10 倍便于 debug。 */
volatile int16_t g_main_edge_tilt_gyro0_dps_x10 = 0; /* 当前 gyro[0]，放大 10 倍便于 debug。 */
volatile uint8_t g_main_nose_down_escape_active = 0U; /* angle[0] 长时间偏离触发姿态脱困。 */
volatile int16_t g_main_nose_down_angle0_deg_x10 = 0; /* 当前 angle[0]，放大 10 倍便于 debug。 */
volatile int16_t g_main_nose_down_delta_deg_x10 = 0; /* 当前 angle[0] 相对初始值的差值，放大 10 倍便于 debug。 */
volatile uint32_t g_main_nose_down_elapsed_ms = 0U; /* angle[0] 连续超出姿态脱困阈值的时间。 */
volatile int8_t g_main_angle0_stuck_direction = 0; /* -1 前进脱困，1 后退脱困。 */
volatile uint8_t g_main_offstage_stuck_escape_active = 0U; /* 台下卡住检测触发轻微前进/后退。 */
volatile uint32_t g_main_offstage_stuck_elapsed_ms = 0U; /* 台下疑似卡住累计时间。 */
volatile uint16_t g_main_offstage_stuck_distance_delta_mm = 0U; /* 台下距离特征相对锚点的最大变化。 */
volatile int16_t g_main_offstage_stuck_gyro_max_dps_x10 = 0; /* 台下卡住检测看到的最大角速度，放大 10 倍。 */


//状态机函数
static void TaskMain_RunState(void);
static void TaskMain_RunInit(void);
static void TaskMain_RunOffStageSearch(void);
static void TaskMain_RunReenter(void);
static void TaskMain_RunPatrol(void);
static void TaskMain_RunTrack(void);
static void TaskMain_RunAttack(void);
static void TaskMain_RunAvoidFriendlyBlock(void);
static void TaskMain_RunEscape(void);
static void TaskMain_LockInitRollBaseline(float roll_deg);
static uint8_t TaskMain_UpdateInitRollBaseline(void);
static uint8_t TaskMain_RunAngle0StuckEscape(void);
static uint16_t TaskMain_AbsDiffU16(uint16_t a, uint16_t b);
static uint16_t TaskMain_MaxU16(uint16_t a, uint16_t b);
static void TaskMain_CaptureOffStageStuckAnchor(void);
static uint8_t TaskMain_AreOffStageStuckLasersValid(void);
static uint16_t TaskMain_GetOffStageStuckDistanceDeltaMm(void);
static uint8_t TaskMain_RunOffStageStuckRecovery(void);
#if TASKMAIN_USE_FAR_EDGE_SENSORS == 0U
static uint8_t TaskMain_AreNearEdgeSensorsSafe(GPIO_PinState left_state, GPIO_PinState right_state);
#endif

//更新敌人信息
static uint8_t TaskMain_UpdateEnemyInfo(void);
// 最近一段时间内是否还看到过敌人？如果没有，说明敌人超时，数据不可信，切回巡台。
static uint8_t TaskMain_IsEnemyTargetAlive(uint32_t now_tick);
static uint32_t TaskMain_GetEnemyTargetHoldMs(void);
// 取角度绝对值。
//比如敌人在左侧是 - 30°，右侧是 30°，这个函数都返回 30。主要用于判断“偏角大不大”，不关心左右方向。
static int16_t TaskMain_GetAbsAngleDeg(int16_t angle_deg);
// 跟踪阶段高速转向时限制前进和转向分量，避免变成高速原地旋转。
static void TaskMain_LimitTrackSpeeds(int *left_speed, int *right_speed);
// 根据敌人角度，用 PID 算左右轮速度。
static void TaskMain_RunAimWithPid(int forward_speed);
// 判断视觉模块是否看到了己方能量块。
static uint8_t TaskMain_IsFriendlyBlockDetected(void);
// 检查左右边缘传感器是否触发。
static uint8_t TaskMain_CheckEdgeDanger(void);
/* 固定上台动作 */
void To_Stage(void)
{
    Run_Back(-80);
    osDelay(100);

    Run_Back(-180);
    osDelay(100);

    Run_Back(-225);
    osDelay(100);

    Run_Back(-335);
    osDelay(100);

    // Run_Back(-415);
    // osDelay(110);

    Run_Back(-500);
    osDelay(650);

    Run_Back(-200);
    osDelay(100);

    if (state_manager.current_state == STATE_INIT)
    {
        Turn_Left(-250, 250);
        osDelay(290);
    }
    else
    {
        Turn_Left(-250, 250);
        osDelay(330);
    }

    // Motor_StopAll();
    // while (1)
    // {
    //    continue;
    // }

    TaskDanger_SuppressEdgeProtection(TASKMAIN_TO_STAGE_EDGE_BLIND_MS);
}
// void To_Stage(void)
// {
//     osDelay(100);

//     // Motor_StopAll();
//     // while (1)
//     // {
//     //    continue;
//     // }

//     TaskDanger_SuppressEdgeProtection(TASKMAIN_TO_STAGE_EDGE_BLIND_MS);
// }

//===================敌人信息========

/* 使用 LiDAR 实时缓冲刷新目标，不等待完整一圈快照。
 * 返回值：
 * 1 表示本次循环成功刷新出了新的敌方目标；
 * 0 表示当前实时缓冲不可用，或本次数据里没找到目标。
 */
static uint8_t TaskMain_UpdateEnemyInfo(void)
{
    uint32_t now_tick = osKernelGetTickCount();
    uint8_t enemy_updated = TrackEnemyTracker_Update(&g_enemy_tracker, now_tick);

    return enemy_updated;
}

/* 根据目标可信度决定缓存目标的保活时间。 */
static uint32_t TaskMain_GetEnemyTargetHoldMs(void)
{
    int16_t abs_angle_deg;

    if (g_enemy_tracker.consecutive_found_count >= TASKMAIN_TARGET_CONFIRM_HIT_COUNT)
    {
        return TASKMAIN_TARGET_CONFIRMED_HOLD_MS;
    }

    abs_angle_deg = TaskMain_GetAbsAngleDeg(g_enemy_tracker.enemy_info.angle_deg);
    if (abs_angle_deg >= TASKMAIN_TARGET_LARGE_ANGLE_HOLD_DEG)
    {
        return TASKMAIN_TARGET_CONFIRMED_HOLD_MS;
    }

    return TASKMAIN_TARGET_SINGLE_HIT_HOLD_MS;
}

/* 判断当前缓存目标是否仍允许使用；过期后立即清空，避免旧目标回魂。 */
static uint8_t TaskMain_IsEnemyTargetAlive(uint32_t now_tick)
{
    uint32_t hold_ms;

    if (g_enemy_tracker.enemy_info.is_found == 0U)
    {
        return 0U;
    }

    hold_ms = TaskMain_GetEnemyTargetHoldMs();
    if ((now_tick - g_enemy_tracker.update_tick) > hold_ms)
    {
        TrackEnemyTracker_Clear(&g_enemy_tracker);
        return 0U;
    }

    return 1U;
}

//========================================

/* 取整型角度的绝对值，便于状态机直接比较左右偏差。 */
static int16_t TaskMain_GetAbsAngleDeg(int16_t angle_deg)
{
    if (angle_deg < 0)
    {
        return (int16_t)(-angle_deg);
    }

    return angle_deg;
}

/* 跟踪阶段高速转向时限制前进和转向分量。
 * 内侧轮保持低速前进，既能边走边修正，也避免单侧反转导致原地超速旋转。 */
static void TaskMain_LimitTrackSpeeds(int *left_speed, int *right_speed)
{
    int forward_speed;
    int turn_speed;

    if (left_speed == NULL || right_speed == NULL)
    {
        return;
    }

    forward_speed = (*left_speed + *right_speed) / 2;
    if (forward_speed <= TASKMAIN_TRACK_LIMIT_TRIGGER_PWM)
    {
        return;
    }

    turn_speed = (*right_speed - *left_speed) / 2;
    if (turn_speed > TASKMAIN_TRACK_TURN_LIMIT_PWM)
    {
        turn_speed = TASKMAIN_TRACK_TURN_LIMIT_PWM;
    }
    else if (turn_speed < -TASKMAIN_TRACK_TURN_LIMIT_PWM)
    {
        turn_speed = -TASKMAIN_TRACK_TURN_LIMIT_PWM;
    }

    forward_speed = TASKMAIN_TRACK_LIMIT_PWM;

    *left_speed = forward_speed - turn_speed;
    *right_speed = forward_speed + turn_speed;

    if (*left_speed < TASKMAIN_TRACK_INNER_MIN_PWM)
    {
        *left_speed = TASKMAIN_TRACK_INNER_MIN_PWM;
    }

    if (*right_speed < TASKMAIN_TRACK_INNER_MIN_PWM)
    {
        *right_speed = TASKMAIN_TRACK_INNER_MIN_PWM;
    }
}

/* 使用角度 PID 计算当前逼近阶段的左右轮速度。 */
static void TaskMain_RunAimWithPid(int forward_speed)
{
    int left_speed; /* 当前计算得到的左轮速度。 */
    int right_speed; /* 当前计算得到的右轮速度。 */
    int forward_component; /* 当前左右轮平均前进分量。 */
    int turn_component; /* 当前左右轮差速转向分量。 */

    AnglePid_ComputeTrackSpeeds(&track_angle_pid,
                                (uint8_t)(state_manager.state_loop_count == 1U),
                                g_enemy_tracker.enemy_info.angle_deg,
                                forward_speed,
                                &left_speed,
                                &right_speed);

    if (state_manager.current_state == STATE_ATTACK)
    {
        forward_component = (left_speed + right_speed) / 2;
        turn_component = (right_speed - left_speed) / 2;
        turn_component = (turn_component * TASKMAIN_ATTACK_PID_SCALE_PERCENT) / 100;
        left_speed = forward_component - turn_component;
        right_speed = forward_component + turn_component;
        if (forward_speed > 0)
        {
            if (left_speed < 0)
            {
                left_speed = 0;
            }
            if (right_speed < 0)
            {
                right_speed = 0;
            }
        }
    }

    if (state_manager.current_state == STATE_TRACK)
    {
        TaskMain_LimitTrackSpeeds(&left_speed, &right_speed);
    }

    Motor_SetSpeeds(left_speed, right_speed);
}

/* 读取视觉标签，判断当前是否稳定看到了己方能量块。 */
static uint8_t TaskMain_IsFriendlyBlockDetected(void)
{
    Vision_Tag tag;

    if (Vision_IsOnline(TASKMAIN_VISION_TIMEOUT_MS) == 0U)
    {
        return 0U;
    }

    if (Vision_GetLatestTag(&tag) == 0U)
    {
        return 0U;
    }

    if ((tag.flags & VISION_FLAG_DETECTED) == 0U ||
        (tag.flags & VISION_FLAG_ID_VALID) == 0U)
    {
        return 0U;
    }

    if (team == TEAM_BLUE)
    {
        return (uint8_t)(tag.tag_id == TASKMAIN_FRIENDLY_TAG_BLUE);
    }

    if (team == TEAM_YELLOW)
    {
        return (uint8_t)(tag.tag_id == TASKMAIN_FRIENDLY_TAG_YELLOW);
    }

    return 0U;
}

static void TaskMain_LockInitRollBaseline(float roll_deg)
{
    s_edge_roll_baseline_deg = roll_deg;
    s_edge_roll_baseline_valid = 1U;
    s_init_roll_wait_start_tick = 0U;
    g_main_edge_roll_baseline_deg_x10 = (int16_t)(s_edge_roll_baseline_deg * 10.0f);
    g_main_edge_roll_delta_deg_x10 = 0;
    g_main_edge_tilt_gyro0_dps_x10 = 0;
    g_main_edge_tilt_bypass_active = 0U;
}

static uint8_t TaskMain_UpdateInitRollBaseline(void)
{
    float roll_deg;
    uint32_t now_tick;
    uint8_t timed_out;

    if (s_edge_roll_baseline_valid != 0U)
    {
        return 1U;
    }

    now_tick = osKernelGetTickCount();
    if (s_init_roll_wait_start_tick == 0U)
    {
        s_init_roll_wait_start_tick = now_tick;
    }
    timed_out = ((uint32_t)(now_tick - s_init_roll_wait_start_tick) >= TASKMAIN_INIT_ROLL_LOCK_TIMEOUT_MS) ? 1U : 0U;

    if (JY901S_IsOnline(TASKMAIN_INIT_IMU_TIMEOUT_MS) == 0U)
    {
        s_init_roll_seen = 0U;
        s_init_roll_stable_count = 0U;
        return 0U;
    }

    roll_deg = JY901S_GetRollDeg();
    if (s_init_roll_seen == 0U)
    {
        s_init_last_roll_deg = roll_deg;
        s_init_roll_seen = 1U;
        s_init_roll_stable_count = 0U;
        if (timed_out != 0U)
        {
            TaskMain_LockInitRollBaseline(roll_deg);
            return 1U;
        }
        return 0U;
    }

    if (fabsf(TrackEnemy_NormalizeAngleDeg(roll_deg - s_init_last_roll_deg)) <= TASKMAIN_INIT_ROLL_STABLE_DEG)
    {
        if (s_init_roll_stable_count < TASKMAIN_INIT_ROLL_STABLE_COUNT)
        {
            ++s_init_roll_stable_count;
        }
    }
    else
    {
        s_init_roll_stable_count = 0U;
    }
    s_init_last_roll_deg = roll_deg;

    if (s_init_roll_stable_count < TASKMAIN_INIT_ROLL_STABLE_COUNT &&
        timed_out == 0U)
    {
        return 0U;
    }

    TaskMain_LockInitRollBaseline(roll_deg);
    return 1U;
}

static uint16_t TaskMain_AbsDiffU16(uint16_t a, uint16_t b)
{
    return (a > b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static uint16_t TaskMain_MaxU16(uint16_t a, uint16_t b)
{
    return (a > b) ? a : b;
}

static void TaskMain_CaptureOffStageStuckAnchor(void)
{
    s_offstage_stuck_anchor_type = g_stage_type;
    s_offstage_stuck_anchor_distances[0] = g_stage_debug_info.laser_front_mm;
    s_offstage_stuck_anchor_distances[1] = g_stage_debug_info.laser_right_mm;
    s_offstage_stuck_anchor_distances[2] = g_stage_debug_info.laser_back_mm;
    s_offstage_stuck_anchor_distances[3] = g_stage_debug_info.laser_left_mm;
}

static uint8_t TaskMain_AreOffStageStuckLasersValid(void)
{
    return (g_stage_debug_info.laser_front_mm != 0U ||
            g_stage_debug_info.laser_right_mm != 0U ||
            g_stage_debug_info.laser_back_mm != 0U ||
            g_stage_debug_info.laser_left_mm != 0U) ? 1U : 0U;
}

static uint16_t TaskMain_GetOffStageStuckDistanceDeltaMm(void)
{
    uint16_t delta_mm = 0U;

    delta_mm = TaskMain_MaxU16(delta_mm, TaskMain_AbsDiffU16(g_stage_debug_info.laser_front_mm, s_offstage_stuck_anchor_distances[0]));
    delta_mm = TaskMain_MaxU16(delta_mm, TaskMain_AbsDiffU16(g_stage_debug_info.laser_right_mm, s_offstage_stuck_anchor_distances[1]));
    delta_mm = TaskMain_MaxU16(delta_mm, TaskMain_AbsDiffU16(g_stage_debug_info.laser_back_mm, s_offstage_stuck_anchor_distances[2]));
    delta_mm = TaskMain_MaxU16(delta_mm, TaskMain_AbsDiffU16(g_stage_debug_info.laser_left_mm, s_offstage_stuck_anchor_distances[3]));
    return delta_mm;
}

static uint8_t TaskMain_RunOffStageStuckRecovery(void)
{
    JY901S_Data imu_data;
    float gyro_max_dps;
    uint16_t distance_delta_mm;
    uint32_t now_tick;
    int unstick_speed;

    if (state_manager.current_state != STATE_OFF_STAGE_SEARCH ||
        g_stage_type == STAGE_ON_STAGE ||
        g_stage_type == STAGE_OFF_REENTER_READY ||
        TaskMain_AreOffStageStuckLasersValid() == 0U ||
        JY901S_IsOnline(TASKMAIN_INIT_IMU_TIMEOUT_MS) == 0U ||
        JY901S_GetData(&imu_data) == 0U ||
        (imu_data.valid_mask & 0x02U) == 0U)
    {
        s_offstage_stuck_start_tick = 0U;
        g_main_offstage_stuck_escape_active = 0U;
        g_main_offstage_stuck_elapsed_ms = 0U;
        g_main_offstage_stuck_distance_delta_mm = 0U;
        return 0U;
    }

    gyro_max_dps = fabsf(imu_data.gyro[0]);
    if (fabsf(imu_data.gyro[1]) > gyro_max_dps)
    {
        gyro_max_dps = fabsf(imu_data.gyro[1]);
    }
    if (fabsf(imu_data.gyro[2]) > gyro_max_dps)
    {
        gyro_max_dps = fabsf(imu_data.gyro[2]);
    }
    g_main_offstage_stuck_gyro_max_dps_x10 = (int16_t)(gyro_max_dps * 10.0f);

    now_tick = osKernelGetTickCount();
    if (s_offstage_stuck_start_tick == 0U)
    {
        s_offstage_stuck_start_tick = now_tick;
        TaskMain_CaptureOffStageStuckAnchor();
        g_main_offstage_stuck_elapsed_ms = 0U;
        g_main_offstage_stuck_distance_delta_mm = 0U;
        return 0U;
    }

    distance_delta_mm = TaskMain_GetOffStageStuckDistanceDeltaMm();
    g_main_offstage_stuck_distance_delta_mm = distance_delta_mm;

    if (g_stage_type != s_offstage_stuck_anchor_type ||
        distance_delta_mm >= TASKMAIN_OFFSTAGE_DISTANCE_MOVE_MM ||
        gyro_max_dps >= TASKMAIN_OFFSTAGE_GYRO_ACTIVE_DPS)
    {
        s_offstage_stuck_start_tick = now_tick;
        TaskMain_CaptureOffStageStuckAnchor();
        g_main_offstage_stuck_escape_active = 0U;
        g_main_offstage_stuck_elapsed_ms = 0U;
        return 0U;
    }

    g_main_offstage_stuck_elapsed_ms = now_tick - s_offstage_stuck_start_tick;
    if (g_main_offstage_stuck_elapsed_ms < TASKMAIN_OFFSTAGE_STUCK_MS)
    {
        g_main_offstage_stuck_escape_active = 0U;
        return 0U;
    }

    g_main_offstage_stuck_escape_active = 1U;
    if (g_stage_debug_info.laser_front_mm > g_stage_debug_info.laser_back_mm)
    {
        unstick_speed = TASKMAIN_OFFSTAGE_UNSTICK_FORWARD_SPEED;
        Run_Forward(unstick_speed);
    }
    else
    {
        unstick_speed = TASKMAIN_OFFSTAGE_UNSTICK_BACK_SPEED;
        Run_Back(unstick_speed);
    }
    osDelay(TASKMAIN_OFFSTAGE_UNSTICK_MS);
    s_offstage_stuck_start_tick = osKernelGetTickCount();
    TaskMain_CaptureOffStageStuckAnchor();
    g_main_offstage_stuck_elapsed_ms = 0U;
    g_main_offstage_stuck_escape_active = 0U;
    return 1U;
}

uint8_t TaskMain_IsEdgeProtectionDisabledByTilt(void)
{
    JY901S_Data imu_data;
    float delta_deg;

    if (s_edge_roll_baseline_valid == 0U ||
        state_manager.current_state != STATE_ATTACK ||
        JY901S_IsOnline(TASKMAIN_INIT_IMU_TIMEOUT_MS) == 0U ||
        JY901S_GetData(&imu_data) == 0U ||
        (imu_data.valid_mask & 0x06U) != 0x06U)
    {
        g_main_edge_tilt_bypass_active = 0U;
        g_main_edge_tilt_gyro0_dps_x10 = 0;
        return 0U;
    }

    delta_deg = TrackEnemy_NormalizeAngleDeg(imu_data.angle[0] - s_edge_roll_baseline_deg);
    g_main_edge_roll_delta_deg_x10 = (int16_t)(delta_deg * 10.0f);
    g_main_edge_tilt_gyro0_dps_x10 = (int16_t)(imu_data.gyro[0] * 10.0f);

    if (delta_deg > TASKMAIN_EDGE_TILT_DISABLE_DEG &&
        imu_data.gyro[0] > TASKMAIN_EDGE_TILT_GYRO_DPS)
    {
        g_main_edge_tilt_bypass_active = 1U;
        return 1U;
    }

    g_main_edge_tilt_bypass_active = 0U;
    return 0U;
}

static uint8_t TaskMain_RunAngle0StuckEscape(void)
{
    float angle0_deg;
    float delta_deg;
    uint32_t now_tick;
    uint32_t required_ms;
    int8_t stuck_direction;

    if (state_manager.current_state == STATE_INIT ||
        state_manager.current_state == STATE_OFF_STAGE_SEARCH ||
        state_manager.current_state == STATE_REENTER_STAGE ||
        s_edge_roll_baseline_valid == 0U ||
        JY901S_IsOnline(TASKMAIN_INIT_IMU_TIMEOUT_MS) == 0U)
    {
        s_angle0_stuck_start_tick = 0U;
        s_angle0_stuck_direction = 0;
        g_main_nose_down_escape_active = 0U;
        g_main_nose_down_delta_deg_x10 = 0;
        g_main_nose_down_elapsed_ms = 0U;
        g_main_angle0_stuck_direction = 0;
        return 0U;
    }

    angle0_deg = JY901S_GetRollDeg();
    delta_deg = TrackEnemy_NormalizeAngleDeg(angle0_deg - s_edge_roll_baseline_deg);
    g_main_nose_down_angle0_deg_x10 = (int16_t)(angle0_deg * 10.0f);
    g_main_nose_down_delta_deg_x10 = (int16_t)(delta_deg * 10.0f);

    if (delta_deg < TASKMAIN_NOSE_DOWN_STUCK_DEG)
    {
        stuck_direction = -1;
    }
    else if (delta_deg > TASKMAIN_NOSE_UP_STUCK_DEG)
    {
        stuck_direction = 1;
    }
    else
    {
        s_angle0_stuck_start_tick = 0U;
        s_angle0_stuck_direction = 0;
        g_main_nose_down_escape_active = 0U;
        g_main_nose_down_elapsed_ms = 0U;
        g_main_angle0_stuck_direction = 0;
        return 0U;
    }

    now_tick = osKernelGetTickCount();
    if (s_angle0_stuck_start_tick == 0U ||
        s_angle0_stuck_direction != stuck_direction)
    {
        s_angle0_stuck_start_tick = now_tick;
        s_angle0_stuck_direction = stuck_direction;
    }
    g_main_nose_down_elapsed_ms = now_tick - s_angle0_stuck_start_tick;
    g_main_angle0_stuck_direction = stuck_direction;
    required_ms = (stuck_direction < 0) ? TASKMAIN_NOSE_DOWN_STUCK_MS : TASKMAIN_NOSE_UP_STUCK_MS;

    if (g_main_nose_down_elapsed_ms < required_ms)
    {
        g_main_nose_down_escape_active = 0U;
        return 0U;
    }

    g_main_nose_down_escape_active = 1U;
    edge_danger_type = EDGE_NONE;
    TaskDanger_SuppressEdgeProtection(TASKMAIN_NOSE_DOWN_EDGE_BLIND_MS);
    if (state_manager.current_state == STATE_EDGE_ESCAPE)
    {
        TaskMain_SwitchState(STATE_PATROL);
    }
    if (stuck_direction < 0)
    {
        Run_Forward(TASKMAIN_NOSE_DOWN_FORWARD_SPEED);
    }
    else
    {
        Run_Back(TASKMAIN_NOSE_UP_BACK_SPEED);
    }
    return 1U;
}

static uint8_t TaskMain_CheckEdgeDanger(void)
{
    GPIO_PinState left_state;
    GPIO_PinState right_state;
#if TASKMAIN_USE_FAR_EDGE_SENSORS != 0U
    GPIO_PinState left_far_state;
    GPIO_PinState right_far_state;
    uint8_t use_far_edge;
#endif
    uint8_t left_edge;
    uint8_t right_edge;

    if (TaskDanger_IsEdgeProtectionSuppressed() != 0U ||
        TaskMain_IsEdgeProtectionDisabledByTilt() != 0U)
    {
        return 0U;
    }

    left_state = HAL_GPIO_ReadPin(LEFT_LIGHT_GPIO_Port, LEFT_LIGHT_Pin);
    right_state = HAL_GPIO_ReadPin(RIGHT_LIGHT_GPIO_Port, RIGHT_LIGHT_Pin);
#if TASKMAIN_USE_FAR_EDGE_SENSORS != 0U
    left_far_state = HAL_GPIO_ReadPin(LEFT_LIGHT_2_GPIO_Port, LEFT_LIGHT_2_Pin);
    right_far_state = HAL_GPIO_ReadPin(RIGHT_LIGHT_2_GPIO_Port, RIGHT_LIGHT_2_Pin);
    use_far_edge = TaskDanger_ShouldUseFarEdgeProtection();
    left_edge = (left_state == GPIO_PIN_RESET ||
                 (use_far_edge != 0U && left_far_state == GPIO_PIN_SET)) ? 1U : 0U;
    right_edge = (right_state == GPIO_PIN_RESET ||
                  (use_far_edge != 0U && right_far_state == GPIO_PIN_SET)) ? 1U : 0U;
#else
    left_edge = (left_state == GPIO_PIN_RESET) ? 1U : 0U;
    right_edge = (right_state == GPIO_PIN_RESET) ? 1U : 0U;
#endif

    if (left_edge != 0U && right_edge != 0U)
    {
        edge_danger_type = EDGE_FRONT;
        TaskMain_SwitchState(STATE_EDGE_ESCAPE);
        return 1U;
    }

    if (left_edge != 0U)
    {
        edge_danger_type = EDGE_LEFT;
        TaskMain_SwitchState(STATE_EDGE_ESCAPE);
        return 1U;
    }

    if (right_edge != 0U)
    {
        edge_danger_type = EDGE_RIGHT;
        TaskMain_SwitchState(STATE_EDGE_ESCAPE);
        return 1U;
    }

    return 0U;
}

#if TASKMAIN_USE_FAR_EDGE_SENSORS == 0U
static uint8_t TaskMain_AreNearEdgeSensorsSafe(GPIO_PinState left_state, GPIO_PinState right_state)
{
    return (left_state == GPIO_PIN_SET && right_state == GPIO_PIN_SET) ? 1U : 0U;
}
#endif

/* 切换状态机 */
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
    s_track_attack_ready_start_tick = 0U;
    if (new_state == STATE_OFF_STAGE_SEARCH)
    {
        s_offstage_parallel_turn_dir = 1;
    }
}

/* 循环运行的状态机管理函数 */
static void TaskMain_RunState(void)
{
    ++state_manager.state_loop_count;

    if (TaskMain_RunAngle0StuckEscape() != 0U)
    {
        return;
    }

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

/* 第一次发车读取左右测距判断己方阵营，并完成首次上台后的状态切换。 */
static void TaskMain_RunInit(void)
{
    uint16_t left_mm = LaserRange_GetDistanceMm(LASER_LEFT);   /* 左侧测距值，用于判断蓝方起始位。 */
    uint16_t right_mm = LaserRange_GetDistanceMm(LASER_RIGHT); /* 右侧测距值，用于判断黄方起始位。 */
    float yaw_deg;

    if (TaskMain_UpdateInitRollBaseline() == 0U)
    {
        return;
    }

    //如果前面的debug标志位为1，那么自动设置为蓝方，防止影响后面的逻辑执行
    if (COMBATCAR_MOTOR_DEBUG_FORCE_STOP != 0U || COMBATCAR_DEBUG_AUTO_TEAM != 0U) {
        osDelay(2000);
        left_mm = 1;
        right_mm = 200;
    }

    //等待陀螺仪稳定，原本为全场定位服务，但是yaw轴不准，暂时不用
    /* 返回 1 表示 IMU 在超时时间内更新过有效数据；返回 0 表示暂时离线。
     * 陀螺仪不在线，会阻塞在init状态。
     */
    // if (JY901S_IsOnline(TASKMAIN_INIT_IMU_TIMEOUT_MS) == 0U)
    // {
    //     s_init_yaw_seen = 0U;
    //     s_init_yaw_stable_count = 0U;
    //     return;
    // }

    // yaw_deg = JY901S_GetYawDeg();
    // if (s_init_yaw_seen == 0U)
    // {
    //     s_init_last_yaw_deg = yaw_deg;
    //     s_init_yaw_seen = 1U;
    //     s_init_yaw_stable_count = 0U;
    //     return;
    // }

    // if (fabsf(TrackEnemy_NormalizeAngleDeg(yaw_deg - s_init_last_yaw_deg)) <= TASKMAIN_INIT_YAW_STABLE_DEG)
    // {
    //     if (s_init_yaw_stable_count < TASKMAIN_INIT_YAW_STABLE_COUNT)
    //     {
    //         ++s_init_yaw_stable_count;
    //     }
    // }
    // else
    // {
    //     s_init_yaw_stable_count = 0U;
    // }
    // s_init_last_yaw_deg = yaw_deg;

    // if (s_init_yaw_stable_count < TASKMAIN_INIT_YAW_STABLE_COUNT)
    // {
    //     return;
    // }

    if (left_mm > 0U && left_mm < 100U)
    {
        team = TEAM_BLUE;
        TrackEnemyTracker_Clear(&g_enemy_tracker);
        s_init_yaw_seen = 0U;
        s_init_yaw_stable_count = 0U;
        s_init_roll_seen = 0U;
        s_init_roll_stable_count = 0U;
        To_Stage();
        TaskMain_SwitchState(STATE_PATROL);
    }
    else if (right_mm > 0U && right_mm < 100U)
    {
        team = TEAM_YELLOW;
        TrackEnemyTracker_Clear(&g_enemy_tracker);
        s_init_yaw_seen = 0U;
        s_init_yaw_stable_count = 0U;
        s_init_roll_seen = 0U;
        s_init_roll_stable_count = 0U;
        To_Stage();
        TaskMain_SwitchState(STATE_PATROL);
    }
}

/*
台下逻辑：
1. 平行但未就绪时原地旋转，等待模型输出 STAGE_OFF_REENTER_READY；
2. 已经车头对外墙、车尾对台边时，进入最终后退上台；
3. 四角已对齐时固定直走/直退脱困；
4. 重新进入平行区后，根据上一轮左右角决定左转还是右转，尽快转到 STAGE_OFF_REENTER_READY；
5. 四角未对齐时原地旋转等模型进入四角脱困标签。
*/
static void TaskMain_RunOffStageSearch(void)
{
    if (TaskMain_RunOffStageStuckRecovery() != 0U)
    {
        return;
    }

    switch (g_stage_type)
    {
    case STAGE_OFF_PARALLEL:
        if (s_offstage_parallel_turn_dir < 0)
        {
            Motor_SetSpeeds(120, -120);
        }
        else
        {
            Motor_SetSpeeds(-120, 120);
        }
        osDelay(10);
        break;

    case STAGE_OFF_REENTER_READY:
        TaskMain_SwitchState(STATE_REENTER_STAGE);
        break;

    case STAGE_OFF_CORNER_FRONT_RIGHT:
    {
        s_offstage_parallel_turn_dir = -1;
        Run_Back(-200);
        osDelay(1200);
        break;
    }

    case STAGE_OFF_CORNER_FRONT_LEFT:
    {
        s_offstage_parallel_turn_dir = 1;
        Run_Back(-200);
        osDelay(1200);
        break;
    }

    case STAGE_OFF_CORNER_REAR_RIGHT:
    {
        s_offstage_parallel_turn_dir = -1;
        Run_Forward(200);
        osDelay(1500);
        break;
    }

    case STAGE_OFF_CORNER_REAR_LEFT:
    {
        s_offstage_parallel_turn_dir = 1;
        Run_Forward(200);
        osDelay(1500);
        break;
    }

    case STAGE_OFF_CORNER_UNALIGNED:
    {
        Turn_Left(-80, 80);
        osDelay(10);
        break;
    }

    default:
        /* 当前区域结果已经恢复成台上，退出台下搜索并回到巡逻。 */
        TaskMain_SwitchState(STATE_PATROL);
        break;
    }
}

/* 处理重新登台阶段的动作控制。
 * 在已经到达可上台区域后执行上台动作。 */
static void TaskMain_RunReenter(void)
{
    /* 统一重登台动作：
     * 1. OFF_STAGE_SEARCH 等到模型输出 STAGE_OFF_REENTER_READY 后才进入这里；
     * 2. 这里不再做区域搜索，只执行最终上台。 */
    /* 车头先轻贴 3.8m 外墙，利用外墙把车身摆正，再高速倒车上 2.4m 高台。 */
    Run_Forward(190);
    osDelay(800);
    To_Stage();
    TaskStage_NotifyReenterComplete();
    TaskMain_SwitchState(STATE_PATROL);
}

/* 巡台，直线前进并配合前方的接近传感器防掉台，且持续等待新的雷达索敌结果。 */
static void TaskMain_RunPatrol(void)
{
    // if (TaskMain_CheckEdgeDanger() != 0U)
    // {
    //     return;
    // }

    if (TaskMain_IsFriendlyBlockDetected() != 0U)
    {
        TaskMain_SwitchState(STATE_AVOID_FRIENDLY_BLOCK);
        return;
    }

    (void)TaskMain_UpdateEnemyInfo();

    if (TaskMain_IsEnemyTargetAlive(osKernelGetTickCount()) != 0U)
    {
        TaskMain_SwitchState(STATE_TRACK);
        return;
    }

    Run_Forward(TASKMAIN_PATROL_SPEED);
}

/* 根据当前目标角度进行 PID 转向对正，并持续逼近。 */
static void TaskMain_RunTrack(void)
{
    uint32_t now_tick;
    int16_t abs_angle_deg;

    (void)TaskMain_UpdateEnemyInfo();
    now_tick = osKernelGetTickCount();

    // if (TaskMain_CheckEdgeDanger() != 0U)
    // {
    //     return;
    // }

    //如果发现己方方块
    if (TaskMain_IsFriendlyBlockDetected() != 0U)
    {
        TaskMain_SwitchState(STATE_AVOID_FRIENDLY_BLOCK);
        return;
    }

    //目标存在，但是超时不可信
    if (TaskMain_IsEnemyTargetAlive(now_tick) == 0U)
    {
        TaskMain_SwitchState(STATE_PATROL);
        return;
    }

    abs_angle_deg = TaskMain_GetAbsAngleDeg(g_enemy_tracker.enemy_info.angle_deg);

    //在前方攻击区域内，连续对准 150ms 后再切攻击，避免刚扫到目标就立刻前冲过头。
    if (g_enemy_tracker.enemy_info.distance_mm <= TASKMAIN_ATTACK_DISTANCE_MM &&
        abs_angle_deg <= TASKMAIN_TRACK_ENTER_ATTACK_ANGLE_DEG)
    {
        if (s_track_attack_ready_start_tick == 0U)
        {
            s_track_attack_ready_start_tick = now_tick;
        }

        if ((uint32_t)(now_tick - s_track_attack_ready_start_tick) >= TASKMAIN_TRACK_ENTER_ATTACK_DELAY_MS)
        {
            TaskMain_SwitchState(STATE_ATTACK);
            return;
        }

        TaskMain_RunAimWithPid(TASKMAIN_TRACK_FORWARD_SPEED);
        return;
    }

    s_track_attack_ready_start_tick = 0U;

    //不符合攻击入口条件，就保持 TRACK，用 200 速度持续修正车头。
    TaskMain_RunAimWithPid(TASKMAIN_TRACK_FORWARD_SPEED);
}

/* 攻击阶段在目标角度基本对正后保持高速前冲。 */
static void TaskMain_RunAttack(void)
{
    uint32_t now_tick;
    int16_t abs_angle_deg;
    uint8_t enemy_updated;

    enemy_updated = TaskMain_UpdateEnemyInfo();
    now_tick = osKernelGetTickCount();

    if (TaskMain_CheckEdgeDanger() != 0U)
    {
        return;
    }

    if (TaskMain_IsFriendlyBlockDetected() != 0U)
    {
        TaskMain_SwitchState(STATE_AVOID_FRIENDLY_BLOCK);
        return;
    }

    if (TaskMain_IsEnemyTargetAlive(now_tick) == 0U)
    {
        TaskMain_SwitchState(STATE_PATROL);
        return;
    }

    abs_angle_deg = TaskMain_GetAbsAngleDeg(g_enemy_tracker.enemy_info.angle_deg);

    if (g_enemy_tracker.enemy_info.distance_mm > TASKMAIN_ATTACK_DISTANCE_MM)
    {
        TaskMain_SwitchState(STATE_TRACK);
        return;
    }

    if (abs_angle_deg > TASKMAIN_ATTACK_STEER_ANGLE_DEG)
    {
        TaskMain_SwitchState(STATE_TRACK);
        return;
    }

    if (enemy_updated == 0U)
    {
        /* 短时丢目标只按最后角度中低速修正，不盲目全速冲。 */
        TaskMain_RunAimWithPid(TASKMAIN_ATTACK_STEER_SPEED);
        return;
    }

    if (abs_angle_deg <= TASKMAIN_ATTACK_FULL_SPEED_ANGLE_DEG)
    {
        TaskMain_RunAimWithPid(TASKMAIN_ATTACK_SPEED);
        return;
    }

    if (abs_angle_deg <= TASKMAIN_ATTACK_STEER_ANGLE_DEG)
    {
        TaskMain_RunAimWithPid(TASKMAIN_ATTACK_STEER_SPEED);
        return;
    }
}

/* 躲避己方方块 */
static void TaskMain_RunAvoidFriendlyBlock(void)
{
    Motor_StopAll();
    osDelay(80);
    Run_Back(-310);
    osDelay(400);
    Turn_Left(-200,200);
    osDelay(400);
    Run_Forward(100);
    osDelay(190);
    TaskMain_SwitchState(STATE_PATROL);
}

/* 触发边缘危险后执行统一的脱困动作。 */
static void TaskMain_RunEscape(void)
{
    int left_danger;
    int right_danger;
    int near_edge_seen = 0;
#if TASKMAIN_USE_FAR_EDGE_SENSORS != 0U
    int left_danger_2;
    int right_danger_2;
#endif
    uint32_t start_tick = HAL_GetTick();

    switch (edge_danger_type)
    {
    case EDGE_LEFT:
        while ((HAL_GetTick() - start_tick) < 250U)
        {
            left_danger = HAL_GPIO_ReadPin(LEFT_LIGHT_GPIO_Port, LEFT_LIGHT_Pin);
            right_danger = HAL_GPIO_ReadPin(RIGHT_LIGHT_GPIO_Port, RIGHT_LIGHT_Pin);
#if TASKMAIN_USE_FAR_EDGE_SENSORS != 0U
            left_danger_2 = HAL_GPIO_ReadPin(LEFT_LIGHT_2_GPIO_Port, LEFT_LIGHT_2_Pin);
            right_danger_2 = HAL_GPIO_ReadPin(RIGHT_LIGHT_2_GPIO_Port, RIGHT_LIGHT_2_Pin);
#endif
            (void)left_danger;
            (void)right_danger;
            if (left_danger == GPIO_PIN_RESET || right_danger == GPIO_PIN_RESET)
            {
                near_edge_seen = 1;
            }

#if TASKMAIN_USE_FAR_EDGE_SENSORS != 0U
            if ((left_danger_2 + right_danger_2) == 0)
#else
            if (TaskMain_AreNearEdgeSensorsSafe((GPIO_PinState)left_danger, (GPIO_PinState)right_danger) != 0U)
#endif
            {
                Motor_SetSpeeds(near_edge_seen != 0 ? -370 : -220,
                                near_edge_seen != 0 ? -370 : -220);
                osDelay(150);
                break;
            }

            Motor_SetSpeeds(near_edge_seen != 0 ? -370 : -220,
                            near_edge_seen != 0 ? -370 : -220);
            osDelay(1);
        }

        Motor_SetSpeeds(-280, -220);
        osDelay(150);

        Run_Back(near_edge_seen != 0 ? -350 : -200);
        osDelay(50);

        Motor_StopAll();
        osDelay(60);

        Turn_Left(380, -380);
        osDelay(150);

        edge_danger_type = EDGE_NONE;
        TaskMain_SwitchState(STATE_PATROL);
        break;

    case EDGE_FRONT:
        while ((HAL_GetTick() - start_tick) < 250U)
        {
            left_danger = HAL_GPIO_ReadPin(LEFT_LIGHT_GPIO_Port, LEFT_LIGHT_Pin);
            right_danger = HAL_GPIO_ReadPin(RIGHT_LIGHT_GPIO_Port, RIGHT_LIGHT_Pin);
#if TASKMAIN_USE_FAR_EDGE_SENSORS != 0U
            left_danger_2 = HAL_GPIO_ReadPin(LEFT_LIGHT_2_GPIO_Port, LEFT_LIGHT_2_Pin);
            right_danger_2 = HAL_GPIO_ReadPin(RIGHT_LIGHT_2_GPIO_Port, RIGHT_LIGHT_2_Pin);
#endif
            (void)left_danger;
            (void)right_danger;
            if (left_danger == GPIO_PIN_RESET || right_danger == GPIO_PIN_RESET)
            {
                near_edge_seen = 1;
            }

#if TASKMAIN_USE_FAR_EDGE_SENSORS != 0U
            if ((left_danger_2 + right_danger_2) == 0)
#else
            if (TaskMain_AreNearEdgeSensorsSafe((GPIO_PinState)left_danger, (GPIO_PinState)right_danger) != 0U)
#endif
            {
                Motor_SetSpeeds(near_edge_seen != 0 ? -370 : -220,
                                near_edge_seen != 0 ? -370 : -220);
                osDelay(100);
                break;
            }

            Motor_SetSpeeds(near_edge_seen != 0 ? -370 : -220,
                            near_edge_seen != 0 ? -370 : -220);
            osDelay(1);
        }

        Motor_SetSpeeds(-250, -280);
        osDelay(150);

        Run_Back(near_edge_seen != 0 ? -350 : -200);
        osDelay(50);

        Motor_StopAll();
        osDelay(60);

        Turn_Left(-380, 380);
        osDelay(150);

        edge_danger_type = EDGE_NONE;
        TaskMain_SwitchState(STATE_PATROL);
        break;

    case EDGE_RIGHT:
        while ((HAL_GetTick() - start_tick) < 250U)
        {
            left_danger = HAL_GPIO_ReadPin(LEFT_LIGHT_GPIO_Port, LEFT_LIGHT_Pin);
            right_danger = HAL_GPIO_ReadPin(RIGHT_LIGHT_GPIO_Port, RIGHT_LIGHT_Pin);
#if TASKMAIN_USE_FAR_EDGE_SENSORS != 0U
            left_danger_2 = HAL_GPIO_ReadPin(LEFT_LIGHT_2_GPIO_Port, LEFT_LIGHT_2_Pin);
            right_danger_2 = HAL_GPIO_ReadPin(RIGHT_LIGHT_2_GPIO_Port, RIGHT_LIGHT_2_Pin);
#endif
            (void)left_danger;
            (void)right_danger;
            if (left_danger == GPIO_PIN_RESET || right_danger == GPIO_PIN_RESET)
            {
                near_edge_seen = 1;
            }

#if TASKMAIN_USE_FAR_EDGE_SENSORS != 0U
            if ((left_danger_2 + right_danger_2) == 0)
#else
            if (TaskMain_AreNearEdgeSensorsSafe((GPIO_PinState)left_danger, (GPIO_PinState)right_danger) != 0U)
#endif
            {
                Motor_SetSpeeds(near_edge_seen != 0 ? -370 : -220,
                                near_edge_seen != 0 ? -370 : -220);
                osDelay(150);
                break;
            }

            Motor_SetSpeeds(near_edge_seen != 0 ? -370 : -220,
                            near_edge_seen != 0 ? -370 : -220);
            osDelay(1);
        }

        Motor_SetSpeeds(-220, -280);
        osDelay(150);

        Run_Back(near_edge_seen != 0 ? -350 : -200);
        osDelay(50);

        Motor_StopAll();
        osDelay(60);

        Turn_Left(-380, 380);
        osDelay(150);

        edge_danger_type = EDGE_NONE;
        TaskMain_SwitchState(STATE_PATROL);
        break;

    default:
        while ((HAL_GetTick() - start_tick) < 250U)
        {
            left_danger = HAL_GPIO_ReadPin(LEFT_LIGHT_GPIO_Port, LEFT_LIGHT_Pin);
            right_danger = HAL_GPIO_ReadPin(RIGHT_LIGHT_GPIO_Port, RIGHT_LIGHT_Pin);
#if TASKMAIN_USE_FAR_EDGE_SENSORS != 0U
            left_danger_2 = HAL_GPIO_ReadPin(LEFT_LIGHT_2_GPIO_Port, LEFT_LIGHT_2_Pin);
            right_danger_2 = HAL_GPIO_ReadPin(RIGHT_LIGHT_2_GPIO_Port, RIGHT_LIGHT_2_Pin);
#endif
            (void)left_danger;
            (void)right_danger;
            if (left_danger == GPIO_PIN_RESET || right_danger == GPIO_PIN_RESET)
            {
                near_edge_seen = 1;
            }

#if TASKMAIN_USE_FAR_EDGE_SENSORS != 0U
            if ((left_danger_2 + right_danger_2) == 0)
#else
            if (TaskMain_AreNearEdgeSensorsSafe((GPIO_PinState)left_danger, (GPIO_PinState)right_danger) != 0U)
#endif
            {
                Motor_SetSpeeds(near_edge_seen != 0 ? -370 : -220,
                                near_edge_seen != 0 ? -370 : -220);
                osDelay(100);
                break;
            }

            Motor_SetSpeeds(near_edge_seen != 0 ? -370 : -220,
                            near_edge_seen != 0 ? -370 : -220);
            osDelay(1);
        }

        Motor_SetSpeeds(-250, -280);
        osDelay(150);

        Run_Back(near_edge_seen != 0 ? -350 : -200);
        osDelay(50);

        Motor_StopAll();
        osDelay(60);

        Turn_Left(-380, 380);
        osDelay(150);

        edge_danger_type = EDGE_NONE;
        TaskMain_SwitchState(STATE_PATROL);
        break;
    }
}

/* 主任务入口，初始化状态机并周期运行主逻辑。 */
void Task_Main_Run(void *argument)
{
    (void)argument;

    state_manager.current_state = STATE_INIT;
    state_manager.previous_state = STATE_INIT;
    state_manager.state_enter_tick = osKernelGetTickCount();
    state_manager.state_loop_count = 0U;
    TrackEnemyTracker_Clear(&g_enemy_tracker);
    AnglePid_Init(&track_angle_pid,
                  TASKMAIN_TRACK_PID_KP,
                  TASKMAIN_TRACK_PID_KI,
                  TASKMAIN_TRACK_PID_KD,
                  TASKMAIN_TRACK_PID_INTEGRAL_LIMIT,
                  TASKMAIN_TRACK_PID_OUTPUT_LIMIT);

    osDelay(1500);

    for (;;)
    {
        TaskMain_RunState();
        osDelay(1);
    }
}
