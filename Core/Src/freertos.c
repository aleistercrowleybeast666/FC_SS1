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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for KEY_SCAN_TASK */
osThreadId_t KEY_SCAN_TASKHandle;
const osThreadAttr_t KEY_SCAN_TASK_attributes = {
  .name = "KEY_SCAN_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TEST_TASK */
osThreadId_t TEST_TASKHandle;
const osThreadAttr_t TEST_TASK_attributes = {
  .name = "TEST_TASK",
  .stack_size = 1280 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RADIO_TXRX_TASK */
osThreadId_t RADIO_TXRX_TASKHandle;
const osThreadAttr_t RADIO_TXRX_TASK_attributes = {
  .name = "RADIO_TXRX_TASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SENSOR_R_TASK */
osThreadId_t SENSOR_R_TASKHandle;
const osThreadAttr_t SENSOR_R_TASK_attributes = {
  .name = "SENSOR_R_TASK",
  .stack_size = 640 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartKEY_SCAN_TASK(void *argument);
void StartTEST_TASK(void *argument);
void StartRADIO_TXRX_TASK(void *argument);
void StartSENSOR_R_TASK(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of KEY_SCAN_TASK */
  KEY_SCAN_TASKHandle = osThreadNew(StartKEY_SCAN_TASK, NULL, &KEY_SCAN_TASK_attributes);

  /* creation of TEST_TASK */
  TEST_TASKHandle = osThreadNew(StartTEST_TASK, NULL, &TEST_TASK_attributes);

  /* creation of RADIO_TXRX_TASK */
  RADIO_TXRX_TASKHandle = osThreadNew(StartRADIO_TXRX_TASK, NULL, &RADIO_TXRX_TASK_attributes);

  /* creation of SENSOR_R_TASK */
  SENSOR_R_TASKHandle = osThreadNew(StartSENSOR_R_TASK, NULL, &SENSOR_R_TASK_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartKEY_SCAN_TASK */
/**
* @brief Function implementing the KEY_SCAN_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKEY_SCAN_TASK */
__weak void StartKEY_SCAN_TASK(void *argument)
{
  /* USER CODE BEGIN StartKEY_SCAN_TASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartKEY_SCAN_TASK */
}

/* USER CODE BEGIN Header_StartTEST_TASK */
/**
* @brief Function implementing the TEST_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTEST_TASK */
__weak void StartTEST_TASK(void *argument)
{
  /* USER CODE BEGIN StartTEST_TASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTEST_TASK */
}

/* USER CODE BEGIN Header_StartRADIO_TXRX_TASK */
/**
* @brief Function implementing the RADIO_TXRX_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRADIO_TXRX_TASK */
__weak void StartRADIO_TXRX_TASK(void *argument)
{
  /* USER CODE BEGIN StartRADIO_TXRX_TASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRADIO_TXRX_TASK */
}

/* USER CODE BEGIN Header_StartSENSOR_R_TASK */
/**
* @brief Function implementing the SENSOR_R_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSENSOR_R_TASK */
__weak void StartSENSOR_R_TASK(void *argument)
{
  /* USER CODE BEGIN StartSENSOR_R_TASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSENSOR_R_TASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

