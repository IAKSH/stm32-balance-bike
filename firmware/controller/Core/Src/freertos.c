/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "system.h"
#include "tasks.h"
#include "THB001P.h"
#include "stdio.h"
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
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
    .name = "oledTask",
    .stack_size = 128 * 3,
    .priority = (osPriority_t)osPriorityNormal,
};
osThreadId_t mpu6050TaskHandle;
const osThreadAttr_t mpu6050Task_attributes = {
    .name = "mpu6050Task",
    .stack_size = 128 * 3,
    .priority = (osPriority_t)osPriorityNormal1,
};
osThreadId_t thb001pTaskHandle;
const osThreadAttr_t thb001pTask_attributes = {
    .name = "thb001pTask",
    .stack_size = 128 * 2,
    .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t wirelessTxTaskHandle;
const osThreadAttr_t wirelessTxTask_attributes = {
    .name = "wirelessTxTask",
    .stack_size = 128 * 7,
    .priority = (osPriority_t)osPriorityNormal,
};

osSemaphoreId_t thb001pDataReadySemaphore;
const osSemaphoreAttr_t thb001pDataReadySemaphore_attributes = {
    .name = "thb001pDataReadySemaphore"};

osSemaphoreId_t mpu6050DataReadySemaphore;
const osSemaphoreAttr_t mpu6050DataReadySemaphore_attributes = {
    .name = "mpu6050DataReadySemaphore"};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  system_init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  thb001pDataReadySemaphore = osSemaphoreNew(1, 0, &thb001pDataReadySemaphore_attributes);
  mpu6050DataReadySemaphore = osSemaphoreNew(1, 0, &mpu6050DataReadySemaphore_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  oledTaskHandle = osThreadNew(oled_task, NULL, &oledTask_attributes);
  mpu6050TaskHandle = osThreadNew(mpu6050_task,NULL,&mpu6050Task_attributes);
  thb001pTaskHandle = osThreadNew(thb001p_task, NULL, &thb001pTask_attributes);
  wirelessTxTaskHandle = osThreadNew(wireless_send_task, NULL, &wirelessTxTask_attributes);
  if (wirelessTxTaskHandle == NULL)
  {
    printf("failed\n");
  }
  else
  {
    printf("success\n");
  }

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
//   /* Infinite loop */
//   for (;;)
//   {
//     // printf("当前剩余堆空间：%u 字节\r\n", (unsigned int)xPortGetFreeHeapSize());
//     osDelay(1);
//   }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

