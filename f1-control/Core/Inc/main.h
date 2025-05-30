/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern osMutexId_t i2c_bus_mutex;
extern osEventFlagsId_t mpu6050_init_event;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

extern osMutexId_t i2c_bus_mutex;
extern osEventFlagsId_t mpu6050_init_event;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IRQ_Pin GPIO_PIN_4
#define IRQ_GPIO_Port GPIOA
#define IRQ_EXTI_IRQn EXTI4_IRQn
#define CE_Pin GPIO_PIN_0
#define CE_GPIO_Port GPIOB
#define CSN_Pin GPIO_PIN_1
#define CSN_GPIO_Port GPIOB
#define MPU6050_EXTI_Pin GPIO_PIN_5
#define MPU6050_EXTI_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define EVENT_FLAG_GYRO_INITIALIZED 0x01
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
