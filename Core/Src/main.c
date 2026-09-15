/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;
UART_HandleTypeDef huart2;

#define BMP280_BUFFER_SIZE (uint16_t)4 //Clock out bytes from 0xF7 to 0xFC (the extra byte is the control byte that actually sets the address)
#define LIS3DH_BUFFER_SIZE (uint16_t)6 //Clock out bytes from 0x28 to 0x2D (the extra byte is the control byte that actually sets the address)
static void UART_PrintBuffer(const char *label, uint8_t *buf, uint16_t size);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t BMP280_TX_Buffer[BMP280_BUFFER_SIZE];
uint8_t BMP280_RX_Buffer[BMP280_BUFFER_SIZE];
uint8_t LIS3DH_TX_Buffer[LIS3DH_BUFFER_SIZE];
uint8_t LIS3DH_RX_Buffer[LIS3DH_BUFFER_SIZE];

//States for the rocket state machine
typedef enum{
  ON_PAD,
  MOTOR_BURNING,
  COASTING,
  APOGEE,
  MAIN_DEPLOYMENT,
  TOUCHDOWN
} FLIGHT_STATES;

//Flags
int t_standby_elapsed;
int spi1_done; //0 if spi1 is currently in use, 1 if spi1 is free
int spi2_done; //0 if spi2 is currently in use, 1 is spi2 is free
int bmp280_config_mode;

//Global variables/buffers
uint8_t* data;
int pressure_ptr;
int acceleration_ptr;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
 //Malloc a 9x300 buffer-this will be a "mock" NVM storage
  data = (uint8_t*)malloc((9*300)*sizeof(uint8_t));
  pressure_ptr = 0;
  acceleration_ptr = 5;

  memset(BMP280_RX_Buffer, 0, BMP280_BUFFER_SIZE);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  uint16_t code_var = 0x0000; //Seems like an obscure name but it's the variable for all the 'error' codes. mainly for debug purposes
  spi1_done = 1;
  spi2_done = 1;
  bmp280_config_mode = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  FLIGHT_STATES current_state;
  current_state = ON_PAD;

  while (1)
  {
	  /*
	  State machine for the rocket states
	  */
	  switch (current_state)
	  {
	  case ON_PAD: /* code for when the rocket is upright on the PAD */

		if(spi1_done) //Get pressure/temperature data from the BMP280 module
		{
		  UART_PrintBuffer("BMP280 Raw ADC pressure data", BMP280_RX_Buffer, BMP280_BUFFER_SIZE);
		  if(bmp280_config_mode)
		  {
			struct bmp280_config bmp280_comfig_params;
			bmp280_comfig_params.t_standby = _62_5_ms;
			bmp280_comfig_params.temperature_oversampling = SKIP_MEASURMENT;
			bmp280_comfig_params.spi3w_enable = no;
			bmp280_comfig_params.pressure_oversampling = ULTRA_HIGH_RESOLUTION;
			bmp280_comfig_params.power_setting = NORMAL;
			bmp280_comfig_params.filter_coeff = FILTER_OFF;
			code_var = BMP280_CONFIG(bmp280_comfig_params, hspi1, &spi1_done, BMP280_TX_Buffer, BMP280_RX_Buffer, 4); //Only writing 2 bytes to these config registers
		  }
		  else
		  {
			/* Call the BMP280_READ function */
			code_var = BMP280_READ(hspi1, &spi1_done, BMP280_TX_Buffer, BMP280_RX_Buffer, BMP280_BUFFER_SIZE);
		  }
		}

		break;
	  case MOTOR_BURNING:
		break;
	  case COASTING:
		break;
	  case APOGEE:
		break;
	  case MAIN_DEPLOYMENT:
		break;
	  case TOUCHDOWN:
		break;
	  default:
		break;
	  }



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) //This is the BMP280 path.
  {
    spi1_done = 1;
    if(bmp280_config_mode) //even if doing this adds latency, it doesn't matter cause its the config stage and only happens once
    {
      bmp280_config_mode=0;
      BMP280_TX_Buffer[0] = 0xF7;
      BMP280_TX_Buffer[1] = 0x00;
      BMP280_TX_Buffer[2] = 0x00;
      BMP280_TX_Buffer[3] = 0x00;
    }
    else //Add the rx'd data to the mega-buffer. It's 3 bytes of data
    {
      //data[pressure_ptr] = BMP280_RX_Buffer[1];
      //data[pressure_ptr+1] = BMP280_RX_Buffer[2];
      //data[pressure_ptr+2] = BMP280_RX_Buffer[3];
      pressure_ptr+=7; //jump 7 positions ahead to leave room for accelerometer data
    }
    while(SPI1->SR & SPI_SR_BSY){} //Wait for the BSY flag for SPI1 to clear
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //Write the GPIO high to end the SPI transaction
  }
  else if (hspi->Instance == SPI2)
  {
    spi2_done = 1;
  }
}


static void UART_PrintBuffer(const char *label, uint8_t *buf, uint16_t size)
{
  char line[64];
  int len = snprintf(line, sizeof(line), "%s:", label);
  for (uint16_t i = 0; i < size && len < (int)sizeof(line) - 4; i++)
  {
    len += snprintf(line + len, sizeof(line) - len, " %02X", buf[i]);
  }
  len += snprintf(line + len, sizeof(line) - len, "\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, HAL_MAX_DELAY);
}
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
#ifdef USE_FULL_ASSERT
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
