
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_PIN   GPIO_PIN_5
#define BUTTON_PORT  GPIOA
#define REG_RED   0x09  // Example register for RED (verify for MAX30105)
#define REG_IR    0x0A
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t dht_temp = 0;
uint8_t dht_humi = 0;
float lm35_temp = 0;
uint32_t max_red = 0, max_ir = 0;
char gps_buffer[128], gps_lat[20], gps_lon[20];
uint8_t gps_rx, gps_ready = 0, gps_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while (__HAL_TIM_GET_COUNTER(&htim2) < us)
		;
}

void DHT11_Start() {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
	HAL_Delay(18);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
	delay_us(20);

	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

uint8_t DHT11_ReadByte() {
	uint8_t byte = 0;
	for (int i = 0; i < 8; i++) {
		while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))
			;
		delay_us(40);
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)) {
			byte |= (1 << (7 - i));
		}
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))
			;
	}
	return byte;
}

void DHT11_Read() {
	DHT11_Start();
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET) {
	    HAL_Delay(20);  // simple debounce delay
	    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET) {
	        // confirmed press
	    }
	}

	uint8_t humi_int = DHT11_ReadByte();
	DHT11_ReadByte();
	uint8_t temp_int = DHT11_ReadByte();
	DHT11_ReadByte();
	DHT11_ReadByte();

	dht_temp = temp_int;
	dht_humi = humi_int;
}

void LM35_Read() {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t val = HAL_ADC_GetValue(&hadc1);
	lm35_temp = (val * 3.3 / 4095.0) * 100.0;
	HAL_ADC_Stop(&hadc1);
}

void MAX30105_Write(uint8_t reg, uint8_t value) {
	uint8_t data[2] = { reg, value };
	HAL_I2C_Master_Transmit(&hi2c1, 0xAE, data, 2, HAL_MAX_DELAY);
}

uint8_t MAX30105_Read(uint8_t reg) {
	uint8_t val;
	HAL_I2C_Master_Transmit(&hi2c1, 0xAE, &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, 0xAE, &val, 1, HAL_MAX_DELAY);
	return val;
}

void MAX30105_Init() {
	MAX30105_Write(0x09, 0x03); // LED mode
	MAX30105_Write(0x0A, 0x27); // SpO2 config
	MAX30105_Write(0x0C, 0x24); // LED pulse
	MAX30105_Write(0x02, 0xC0); // FIFO config
	MAX30105_Write(0x01, 0x00); // FIFO write ptr
	MAX30105_Write(0x03, 0x00); // FIFO read ptr
}

void MAX30105_ReadData() {
	uint8_t fifo_data[6];
	uint8_t reg = 0x07;
	HAL_I2C_Master_Transmit(&hi2c1, 0xAE, &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, 0xAE, fifo_data, 6, HAL_MAX_DELAY);

	max_red = ((uint32_t) fifo_data[0] << 16) | ((uint32_t) fifo_data[1] << 8)
			| fifo_data[2];
	max_red &= 0x03FFFF;

	max_ir = ((uint32_t) fifo_data[3] << 16) | ((uint32_t) fifo_data[4] << 8)
			| fifo_data[5];
	max_ir &= 0x03FFFF;
}

void GPS_Parse(char *nmea) {
	char *fields[10];
	int i = 0;
	char *token = strtok(nmea, ",");
	while (token && i < 10) {
		fields[i++] = token;
		token = strtok(NULL, ",");
	}

	if (i >= 5 && fields[1] && fields[3]) {
		float lat_raw = atof(fields[1]);
		float lon_raw = atof(fields[3]);

		int lat_deg = lat_raw / 100;
		float lat_min = lat_raw - lat_deg * 100;
		float lat_dec = lat_deg + lat_min / 60.0;

		int lon_deg = lon_raw / 100;
		float lon_min = lon_raw - lon_deg * 100;
		float lon_dec = lon_deg + lon_min / 60.0;

		snprintf(gps_lat, sizeof(gps_lat), "%.5f %s", lat_dec, fields[2]);
		snprintf(gps_lon, sizeof(gps_lon), "%.5f %s", lon_dec, fields[4]);
	}
}

// ========== GSM ==========
char sms_buf[160];
void GSM_Send(const char *lat, const char *lon, float temp, float lm,
		uint8_t humi, uint32_t red, uint32_t ir) {
	const char *num = "+919985798499";

	sprintf(sms_buf, "AT\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t*) sms_buf, strlen(sms_buf),
	HAL_MAX_DELAY);
	HAL_Delay(1000);

	sprintf(sms_buf, "AT+CMGF=1\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t*) sms_buf, strlen(sms_buf),
	HAL_MAX_DELAY);
	HAL_Delay(1000);

	sprintf(sms_buf, "AT+CMGS=\"%s\"\r\n", num);
	HAL_UART_Transmit(&huart3, (uint8_t*) sms_buf, strlen(sms_buf),
	HAL_MAX_DELAY);
	HAL_Delay(1000);

	sprintf(sms_buf,
			"GPS: Lat: %s, Lon: %s\nDHT: T=%.1f H=%.1f\nLM35: %.2f\nMAX: R=%lu I=%lu\n",
			lat, lon, (float) temp, (float) humi, lm, red, ir);
	HAL_UART_Transmit(&huart3, (uint8_t*) sms_buf, strlen(sms_buf),
	HAL_MAX_DELAY);
	HAL_Delay(1000);

	uint8_t ctrl_z = 0x1A;
	HAL_UART_Transmit(&huart3, &ctrl_z, 1, HAL_MAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		if (gps_rx == '\n') {
			gps_buffer[gps_index] = '\0';
			gps_ready = 1;
			gps_index = 0;
		} else {
			if (gps_index < sizeof(gps_buffer) - 1)
				gps_buffer[gps_index++] = gps_rx;
		}
		HAL_UART_Receive_IT(&huart2, &gps_rx, 1);
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);
	HAL_UART_Receive_IT(&huart2, &gps_rx, 1);
	MAX30105_Init();

	HAL_UART_Transmit(&huart1, (uint8_t*) "System Ready\r\n", 14,
	HAL_MAX_DELAY);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		if (HAL_GPIO_ReadPin(GPIOA, BUTTON_PIN) == GPIO_PIN_SET) {

		    HAL_UART_Transmit(&huart1, (uint8_t*)"Button Press Detected on PA5\r\n", 30, HAL_MAX_DELAY);
		    HAL_Delay(200);  // simple debounce

		    const char *msg =
		        "\r\n===== Sensor Triggered by Button PA5 =====\r\n";
		    HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		    // Read DHT11
		    DHT11_Read();
		    sprintf(sms_buf, "DHT11 -> Temp: %d°C, Humidity: %d%%\r\n", dht_temp, dht_humi);
		    HAL_UART_Transmit(&huart1, (uint8_t*) sms_buf, strlen(sms_buf), HAL_MAX_DELAY);

		    // Read LM35
		    LM35_Read();
		    sprintf(sms_buf, "LM35  -> Temp: %.2f°C\r\n", lm35_temp);
		    HAL_UART_Transmit(&huart1, (uint8_t*) sms_buf, strlen(sms_buf), HAL_MAX_DELAY);

		    // Read MAX30105
		    MAX30105_ReadData();
		    sprintf(sms_buf, "MAX30105 -> RED: %lu, IR: %lu\r\n", max_red, max_ir);
		    HAL_UART_Transmit(&huart1, (uint8_t*) sms_buf, strlen(sms_buf), HAL_MAX_DELAY);

		    // Wait for GPS ready
		    HAL_UART_Transmit(&huart1, (uint8_t*)"Waiting for GPS data...\r\n", 26, HAL_MAX_DELAY);
		    while (!gps_ready);
		    gps_ready = 0;

		    // Parse and Print GPS
		    if (strncmp(gps_buffer, "$GNGLL", 6) == 0) {
		        GPS_Parse(gps_buffer);
		    }
		    sprintf(sms_buf, "GPS   -> Latitude: %s, Longitude: %s\r\n", gps_lat, gps_lon);
		    HAL_UART_Transmit(&huart1, (uint8_t*) sms_buf, strlen(sms_buf), HAL_MAX_DELAY);

		    // Send SMS
		    HAL_UART_Transmit(&huart1, (uint8_t*) "Sending SMS...\r\n", 16, HAL_MAX_DELAY);
		    GSM_Send(gps_lat, gps_lon, dht_temp, lm35_temp, dht_humi, max_red, max_ir);
		    HAL_UART_Transmit(&huart1, (uint8_t*) "SMS Sent ✅\r\n", 13, HAL_MAX_DELAY);

		    const char *end = "===== End of Data Transmission =====\r\n\r\n";
		    HAL_UART_Transmit(&huart1, (uint8_t*) end, strlen(end), HAL_MAX_DELAY);

		    // Wait for button release to avoid repeated triggers
		    HAL_UART_Transmit(&huart1, (uint8_t*)"Waiting for button release...\r\n", 32, HAL_MAX_DELAY);
		    while (HAL_GPIO_ReadPin(GPIOA, BUTTON_PIN) == GPIO_PIN_SET);
		    HAL_UART_Transmit(&huart1, (uint8_t*)"Button Released\r\n", 17, HAL_MAX_DELAY);

		    HAL_Delay(500);  // optional delay after release
		}


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
	while (1) {
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
