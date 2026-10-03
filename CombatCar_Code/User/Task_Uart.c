#include "Task_Uart.h"

#include "JY901S.h"
#include "LaserRange.h"
#include "LiDAR.h"
#include "Task_Sensor.h"
#include "Task_TrainLog.h"
#include "Vision.h"
#include "usart.h"

/*
接着Task_sensor的注释
HAL_UARTEx_RxEventCallback是串口空闲中断回调的注释，前面提到过空闲中断会因为稍微有空闲就启动一次中断回调
因此会携带这次一共接收了多少数据，也就是size参数
激光雷达是usart2，会调用Task_Sensor_NotifyFromISR();，该函数会唤醒sensortask
然后回到task_sensor文件

*/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  (void)Size;

  /* DMA/空闲中断场景：只通知 SensorTask，避免在 ISR 中解析大块数据。 */
  if (huart->Instance == USART2)
  {
    Task_Sensor_NotifyFromISR();
    return;
  }

  if (huart->Instance == USART6)
  {
    TaskTrainLog_HandleRxEventFromISR(huart, Size);
    return;
  }

  if (huart->Instance == UART4 || huart->Instance == UART5 ||
      huart->Instance == UART7 || huart->Instance == UART8)
  {
    Task_Sensor_NotifyFromISR();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* 定长中断接收场景：先把一帧交给对应驱动缓存，再唤醒 SensorTask。 */
  if (huart->Instance == USART1)
  {
    JY901S_HandleRxEventFromISR(huart, JY901S_RX_BUF_LEN);
    Task_Sensor_NotifyFromISR();
    return;
  }

  if (huart->Instance == USART3)
  {
    Vision_HandleRxEventFromISR(huart, VISION_RX_BUF_LEN);
    Task_Sensor_NotifyFromISR();
    return;
  }

  if (huart->Instance == USART6)
  {
    TaskTrainLog_HandleRxEventFromISR(huart, 64U);
    return;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* 不同模块分别恢复串口，避免单个传感器异常拖住整车控制。 */
  if (huart->Instance == USART2)
  {
    (void)HAL_UART_DMAStop(huart);
    LiDAR_StartRxDMA();
    return;
  }

  if (huart->Instance == USART3)
  {
    Vision_HandleError(huart);
    return;
  }

  if (huart->Instance == USART1)
  {
    JY901S_HandleError(huart);
    return;
  }

  if (huart->Instance == USART6)
  {
    TaskTrainLog_HandleError(huart);
    return;
  }

  if (huart->Instance == UART4 || huart->Instance == UART5 ||
      huart->Instance == UART7 || huart->Instance == UART8)
  {
    LaserRange_HandleError(huart);
  }
}
