#include "Task_Danger.h"

#include "cmsis_os.h"
#include "main.h"
#include "Task_Main.h"

static int left_danger = 1;
static int right_danger = 1;

EdgeDangerType edge_danger_type = EDGE_NONE;

void Task_Danger_Run(void *argument)
{
    (void)argument;

    for (;;)
    {
        left_danger = HAL_GPIO_ReadPin(LEFT_LIGHT_GPIO_Port, LEFT_LIGHT_Pin);
        right_danger = HAL_GPIO_ReadPin(RIGHT_LIGHT_GPIO_Port, RIGHT_LIGHT_Pin);

        if (left_danger == 0 && right_danger != 0)
        {
            edge_danger_type = EDGE_LEFT;
            TaskMain_SwitchState(STATE_EDGE_ESCAPE);
        }
        else if (right_danger == 0 && left_danger != 0)
        {
            edge_danger_type = EDGE_RIGHT;
            TaskMain_SwitchState(STATE_EDGE_ESCAPE);
        }
        else if (right_danger == 0 && left_danger == 0)
        {
            edge_danger_type = EDGE_FRONT;
            TaskMain_SwitchState(STATE_EDGE_ESCAPE);
        }

        osDelay(1);
    }
}
