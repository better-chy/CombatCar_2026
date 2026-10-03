#ifndef __TASK_MAIN_H
#define __TASK_MAIN_H

#include <stdint.h>

typedef enum
{
    STATE_INIT = 0,
    STATE_OFF_STAGE_SEARCH,
    STATE_REENTER_STAGE,
    STATE_PATROL,
    STATE_TRACK,
    STATE_ATTACK,
    STATE_AVOID_FRIENDLY_BLOCK,
    STATE_EDGE_ESCAPE
} CarState;

typedef struct
{
    CarState current_state;    /* 当前主状态。 */
    CarState previous_state;   /* 上一个主状态。 */
    uint32_t state_enter_tick; /* 进入当前状态时的系统时刻。 */
    uint32_t state_loop_count; /* 当前状态已执行的循环次数。 */
} CarStateManager;

extern CarStateManager state_manager;

void Task_Main_Run(void *argument);
void TaskMain_SwitchState(CarState new_state);

#endif
