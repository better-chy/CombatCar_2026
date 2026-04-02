#include "Task_Uart.h"

#include "JY901S.h"
#include "LaserRange.h"
#include "LiDAR.h"
#include "Task_Sensor.h"
#include "Vision.h"
#include "usart.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  (void)Size;

  if (huart->Instance == USART2)
  {
    Task_Sensor_NotifyFromISR();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
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
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
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

  if (huart->Instance == UART4 || huart->Instance == UART5 ||
      huart->Instance == UART7 || huart->Instance == UART8)
  {
    LaserRange_HandleError(huart);
  }
}
