#ifndef __TASK_MAIN_H
#define __TASK_MAIN_H

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

void Task_Main_Run(void *argument);
void TaskMain_SwitchState(CarState new_state);

#endif
