#include "Task_Main.h"

#include "cmsis_os.h"

/* ==================== 主状态机 ==================== */

typedef enum
{
    STATE_INIT = 0,
    STATE_OFF_STAGE_SEARCH,    // 车在台下，统一处理上台前的搜索和机动
    STATE_REENTER_STAGE,       // 已经具备上台条件，执行正式上台动作
    STATE_PATROL,              // 车在台上且没有稳定目标时，执行巡台和常规搜索
    STATE_TRACK,               // 已经发现目标，正在调整方位和姿态
    STATE_ATTACK,              // 攻击执行状态，例如直冲、弧线冲击、推挤
    STATE_EDGE_ESCAPE          // 边缘危险触发后的高优先级脱险状态
} CarState;

/* ==================== 运行时结构体 ==================== */

typedef struct
{
    CarState current_state;   // 当前主状态
    CarState previous_state;  // 上一个主状态
    uint32_t state_enter_tick;
    uint32_t state_loop_count;
} CarStateManager;

static CarStateManager state_manager;

/* ==================== 内部函数声明 ==================== */

static void TaskMain_SwitchState(CarState new_state);
static void TaskMain_RunState(void);
static void TaskMain_RunInit(void);
static void TaskMain_RunOffStage(void);
static void TaskMain_RunReenter(void);
static void TaskMain_RunPatrol(void);
static void TaskMain_RunTrack(void);
static void TaskMain_RunAttack(void);
static void TaskMain_RunEscape(void);

/* ==================== 主状态机辅助函数 ==================== */

static void TaskMain_SwitchState(CarState new_state)
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

static void TaskMain_RunState(void)
{
    ++state_manager.state_loop_count;

    switch (state_manager.current_state)
    {
    case STATE_INIT:
        TaskMain_RunInit();
        break;

    case STATE_OFF_STAGE_SEARCH:
        TaskMain_RunOffStage();
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

    case STATE_EDGE_ESCAPE:
        TaskMain_RunEscape();
        break;

    default:
        TaskMain_SwitchState(STATE_INIT);
        break;
    }
}

/* ==================== 主状态处理函数 ==================== */

static void TaskMain_RunInit(void)
{
    TaskMain_SwitchState(STATE_OFF_STAGE_SEARCH);
}

static void TaskMain_RunOffStage(void)
{

}

static void TaskMain_RunReenter(void)
{

}

static void TaskMain_RunPatrol(void)
{

}

static void TaskMain_RunTrack(void)
{

}

static void TaskMain_RunAttack(void)
{
    
}

static void TaskMain_RunEscape(void)
{

}

void Task_Main_Run(void *argument)
{
    (void)argument;

    state_manager.current_state = STATE_INIT;
    state_manager.previous_state = STATE_INIT;
    state_manager.state_enter_tick = osKernelGetTickCount();
    state_manager.state_loop_count = 0U;

    for (;;)
    {
        TaskMain_RunState();
        osDelay(1);
    }
}
