#include "Task_Sensor.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "JY901S.h"
#include "LaserRange.h"
#include "LiDAR.h"
#include "task.h"
#include "usart.h"
#include "Vision.h"

static TaskHandle_t s_sensor_task_handle = NULL;

/*
Task_Sensor运行流程，从这里开始看，先讲激光雷达，每个函数里面也写了注释，你以这里为入口，一个个跳转看注释

由于为了最大化利用H7效率，开启了DCache，DCache会把内存数据缓存，cpu会读缓存里面的数据，但是dma写内存，不会告诉cpu缓存也要更新，所以会出现数据不同步
因此在mpu区域指定一片数据区域是非d缓存，让dma写在那边，让cpu读出来是最新的
1.在for循环之前先写初始化代码，LiDAR_Init()函数会先初始化激光雷达用的是哪一个串口，还有他的内存地址，且清空缓冲区的数据，解析的状态
2. LiDAR_StartRxDMA();  比普通串口稍微复杂，你平时自己用不是先在前面receive一次，才能开启吗，这里把一些东西封装了，具体内容跳进去看，有注释
3.LiDAR_CmdStartMotor();发送串口命令，启动激光雷达的扫描
4.注意到Task_Uart里面的代码，先看串口中断回调，然后里面有接着这里的注释
5.Task_Sensor_NotifyFromISR(void)这个函数是用来唤醒某一个线程，task_sensor里面获取了一次任务的句柄，如果没有句柄，那么就会出现问题，保护性代码
6.顺着LiDAR_PollDma();函数的流程你一个个跳转看代码，全部有注释，都在lidar.c里面
*/

/* 串口中断唤醒 SensorTask，具体解析放在线程中做，减少中断内耗时。 */
void Task_Sensor_NotifyFromISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; // 中断通知后，是否有唤醒一个更高优先级的任务

    //假如说任务没有句柄，也就是任务没获取到数据的话，就不要叫醒了
    if (s_sensor_task_handle == NULL)
    {
        return;
    }

    vTaskNotifyGiveFromISR(s_sensor_task_handle, &xHigherPriorityTaskWoken);  //给sensortask发信号，让其赶紧工作
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);   // 如果刚刚有唤醒有更高优先级任务（也就是假如sensortask优先级高），中断退出后立刻切换到那个任务
}

void Task_Sensor_Run(void *argument)
{
    (void)argument;
    s_sensor_task_handle = xTaskGetCurrentTaskHandle();   //保护性代码，获取任务句柄

    /* 统一启动所有传感器接收链路。 */
    LiDAR_Init(&huart2);
    LiDAR_StartRxDMA();
    LiDAR_CmdStartMotor();
    LaserRange_Init();
    JY901S_Init(&huart1);
    Vision_Init(&huart3);

    for (;;)
    {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));   //如果前面的唤醒成功了，立刻启动，如果没有成功，那么也会1ms运行一次
        LiDAR_PollDma();
        LaserRange_ProcessPending();
        JY901S_ProcessPending();
        Vision_ProcessPending();
    }
}
