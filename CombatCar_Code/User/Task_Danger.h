#ifndef __TASK_DANGER_H
#define __TASK_DANGER_H

typedef enum
{
    EDGE_NONE = 0,
    EDGE_LEFT,
    EDGE_RIGHT,
    EDGE_FRONT
} EdgeDangerType;

extern EdgeDangerType edge_danger_type;

void Task_Danger_Run(void *argument);

#endif
