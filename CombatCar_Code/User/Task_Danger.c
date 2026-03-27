#include "Task_Danger.h"

#include "cmsis_os.h"
#include "main.h"

static int left_danger = 1;
static int right_danger = 1;

void Task_Danger_Run(void *argument)
{
    for (;;)
    {
        left_danger = HAL_GPIO_ReadPin(LEFT_LIGHT_GPIO_Port, LEFT_LIGHT_Pin);
        right_danger = HAL_GPIO_ReadPin(RIGHT_LIGHT_GPIO_Port, RIGHT_LIGHT_Pin);

        if (left_danger == 0 && right_danger != 0)
        {

        }
        else if (right_danger == 0 && left_danger != 0)
        {

        }
        else if (right_danger == 0 && left_danger == 0)
        {
            
        }
        osDelay(1);
    }
}
