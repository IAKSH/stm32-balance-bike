/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <drivers/ssd1306/ssd1306.h>
#include <drivers/nrf24l01p/nrf24l01p.h>
#include <protocols/wireless.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define I2C_PORT hi2c1
#define I2C_ADDR (0x3C << 1)

#define COMMAND_FRAG_NUM_MAX 128

#define NRF24L01P_SPI                     (&hspi1)
#define NRF24L01P_SPI_CS_PIN_PORT         WIRELESS_CSN_GPIO_Port 
#define NRF24L01P_SPI_CS_PIN_NUMBER       WIRELESS_CSN_Pin
#define NRF24L01P_CE_PIN_PORT             WIRELESS_CE_GPIO_Port
#define NRF24L01P_CE_PIN_NUMBER           WIRELESS_CE_Pin
#define NRF24L01P_IRQ_PIN_PORT            EXIT7_WIRELESS_IRQ_GPIO_Port
#define NRF24L01P_IRQ_PIN_NUMBER          EXIT7_WIRELESS_IRQ_Pin
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
static uint8_t tx_address[5] = {0x0,0x0,0x0,0x0,0x01};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void oled_write_cmd(uint8_t byte) {
  HAL_I2C_Mem_Write(&I2C_PORT,I2C_ADDR,0x00,1,&byte,1,HAL_MAX_DELAY);
}

static void oled_write_data(uint8_t* buf,uint16_t len) {
  HAL_I2C_Mem_Write(&I2C_PORT,I2C_ADDR,0x40,1,buf,len,HAL_MAX_DELAY);
}

static void cs_high(void) {
  HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_SET);
}

static void cs_low(void) {
  HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_RESET);
}

static void ce_high(void) {
  HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_SET);
}

static void ce_low(void) {
  HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_RESET);
}

static void spi_transmit(uint8_t *data, uint16_t len) {
  HAL_SPI_Transmit(NRF24L01P_SPI, data, len, 2000);
}

static void spi_receive(uint8_t *data, uint16_t len) {
  HAL_SPI_Receive(NRF24L01P_SPI, data, len, 2000);
}

static void spi_trans_receive(uint8_t *tx, uint8_t *rx, uint16_t len) {
  HAL_SPI_TransmitReceive(NRF24L01P_SPI, tx, rx, len, 2000);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // init ssd1306
  SSD1306State oled_state;
  ssd1306_create_state(&oled_state);

  oled_state.__impl.delay_ms = HAL_Delay;
  oled_state.__impl.write_cmd = oled_write_cmd;
  oled_state.__impl.write_data = oled_write_data;
  oled_state.font = &SSD1306Font_16x24;

  ssd1306_use_state(&oled_state);
  ssd1306_init();

  NRF24L01PState nrf24_state;
  WirelessState wireless_state;
  nrf24l01p_create_state(&nrf24_state);
  wireless_create_state(&wireless_state);

  nrf24_state.__impl.ce_high = ce_high;
  nrf24_state.__impl.ce_low = ce_low;
  nrf24_state.__impl.cs_high = cs_high;
  nrf24_state.__impl.cs_low = cs_low;
  nrf24_state.__impl.spi_receive = spi_receive;
  nrf24_state.__impl.spi_transmit = spi_transmit;
  nrf24_state.__impl.spi_transmit_receive = spi_trans_receive;
  nrf24_state.__impl.get_tick = HAL_GetTick;

  wireless_state.nrf24 = &nrf24_state;
  //wireless_state.__impl.get_queue = get_command_queue;

  nrf24l01p_use_state(&nrf24_state);
  wireless_use_state(&wireless_state);

  // init nrf24l01p
  nrf24l01p_set_mode_tx(2500,NRF24L01P_AIR_DATA_RATE_1Mbps);
  if(!nrf24l01p_check()) {
    ssd1306_write_string("nrf24l01p died!");
    ssd1306_flush();
    Error_Handler();
  }

  nrf24l01p_set_tx_addr(tx_address,5);
  nrf24l01p_set_rx_addr(0,tx_address,5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  CommandPacket command = {
    .version = 0x00,
    .type = COMMAND_MOVE,
    .payload.move.speed = {INT16_MAX, INT16_MAX}
  };

  int i = 0;
  char buf[16];
  while (1)
  {
    snprintf(buf, sizeof(buf), "i = %d", i);
    oled_state.cursor.x = oled_state.cursor.y = 0;
    ssd1306_write_string(buf);
    ssd1306_flush();

    // 更新 command 中的数据
    command.payload.move.speed[0] = i;
    command.payload.move.speed[1] = INT16_MAX - i;
    i++;

    wireless_send(&command,sizeof(CommandPacket));
    HAL_Delay(100);

    //command.type = COMMAND_PID;
    //command.payload.pid.write = false;
    //wireless_send(&command,sizeof(CommandPacket));
    //HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WIRELESS_CSN_GPIO_Port, WIRELESS_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WIRELESS_CE_GPIO_Port, WIRELESS_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : WIRELESS_IRQ_Pin */
  GPIO_InitStruct.Pin = WIRELESS_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WIRELESS_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WIRELESS_CSN_Pin */
  GPIO_InitStruct.Pin = WIRELESS_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WIRELESS_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WIRELESS_CE_Pin */
  GPIO_InitStruct.Pin = WIRELESS_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WIRELESS_CE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
