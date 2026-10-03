/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CombatCar_TuneConfig.h"
#include "Task_Danger.h"
#include "Task_Main.h"
#include "Task_Sensor.h"
#include "Task_Stage.h"
#include "Task_TrainLog.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile uint32_t g_freertos_stack_overflow_hit = 0U;
volatile uint32_t g_freertos_malloc_failed_hit = 0U;
volatile char g_freertos_stack_overflow_task[16] = {0};

/* USER CODE END Variables */
/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
  .name = "MainTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DangerTask */
osThreadId_t DangerTaskHandle;
const osThreadAttr_t DangerTask_attributes = {
  .name = "DangerTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for StageTask */
osThreadId_t StageTaskHandle;
const osThreadAttr_t StageTask_attributes = {
  .name = "StageTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TrainLogTask */
#if COMBATCAR_ENABLE_TRAINLOG != 0U
osThreadId_t TrainLogTaskHandle;
const osThreadAttr_t TrainLogTask_attributes = {
  .name = "TrainLogTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
#endif

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMainTask(void *argument);
void StartSensorTask(void *argument);
void StartDangerTask(void *argument);
void StartStageTask(void *argument);
#if COMBATCAR_ENABLE_TRAINLOG != 0U
void StartTrainLogTask(void *argument);
#endif

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MainTask */
  MainTaskHandle = osThreadNew(StartMainTask, NULL, &MainTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of DangerTask */
  DangerTaskHandle = osThreadNew(StartDangerTask, NULL, &DangerTask_attributes);

  /* creation of StageTask */
  StageTaskHandle = osThreadNew(StartStageTask, NULL, &StageTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* creation of TrainLogTask */
#if COMBATCAR_ENABLE_TRAINLOG != 0U
  TrainLogTaskHandle = osThreadNew(StartTrainLogTask, NULL, &TrainLogTask_attributes);
#endif

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the MainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN StartMainTask */
  Task_Main_Run(argument);
  /* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  Task_Sensor_Run(argument);
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartDangerTask */
/**
* @brief Function implementing the DangerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDangerTask */
void StartDangerTask(void *argument)
{
  /* USER CODE BEGIN StartDangerTask */
  Task_Danger_Run(argument);
  /* USER CODE END StartDangerTask */
}

/* USER CODE BEGIN Header_StartStageTask */
/**
* @brief Function implementing the StageTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStageTask */
void StartStageTask(void *argument)
{
  /* USER CODE BEGIN StartStageTask */
  Task_Stage_Run(argument);
  /* USER CODE END StartStageTask */
}

/* USER CODE BEGIN Header_StartTrainLogTask */
/**
* @brief Function implementing the TrainLogTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTrainLogTask */
#if COMBATCAR_ENABLE_TRAINLOG != 0U
void StartTrainLogTask(void *argument)
{
  /* USER CODE BEGIN StartTrainLogTask */
  Task_TrainLog_Run(argument);
  /* USER CODE END StartTrainLogTask */
}
#endif

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  uint8_t i;

  (void)xTask;
  g_freertos_stack_overflow_hit = 1U;
  for (i = 0U; i < (uint8_t)(sizeof(g_freertos_stack_overflow_task) - 1U); ++i)
  {
    if (pcTaskName == NULL || pcTaskName[i] == '\0')
    {
      break;
    }
    g_freertos_stack_overflow_task[i] = pcTaskName[i];
  }
  g_freertos_stack_overflow_task[i] = '\0';

  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

void vApplicationMallocFailedHook(void)
{
  g_freertos_malloc_failed_hit = 1U;
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

/* USER CODE END Application */

