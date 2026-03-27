#include "Task_Sensor.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "JY901S.h"
#include "LaserRange.h"
#include "LiDAR.h"
#include "task.h"
#include "usart.h"

static TaskHandle_t s_sensor_task_handle = NULL;

void Task_Sensor_NotifyFromISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (s_sensor_task_handle == NULL)
    {
        return;
    }

    vTaskNotifyGiveFromISR(s_sensor_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Task_Sensor_Run(void *argument)
{
    (void)argument;
    s_sensor_task_handle = xTaskGetCurrentTaskHandle();

    LiDAR_Init(&huart2);
    LiDAR_StartRxDMA();
    LiDAR_CmdStartMotor();
    LaserRange_Init();
    JY901S_Init(&huart1);

    for (;;)
    {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
        LiDAR_PollDma();
        LaserRange_ProcessPending();
        JY901S_ProcessPending();
    }
}
