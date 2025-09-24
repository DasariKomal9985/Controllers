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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "dht11.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_PORT GPIOA
#define BUTTON_PIN GPIO_PIN_5
#define MAX_DHT_ATTEMPTS 3
#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint8_t flag_1s = 0;
uint8_t temp = 0, hum = 0;
char uart_buf[512];
uint16_t adc_value = 0;
float tempC = 0;
uint32_t red = 0, ir = 0;
char uart_buffer[512];
char gps_buffer[128];
volatile uint8_t gps_ready = 0;
uint8_t rx_data;
uint8_t rx_index = 0;
char sms[512];
char latitude[16] = "NA";
char longitude[16] = "NA";
char sms_message[512];
#define BP_HEADER 0xAA
uint8_t bp_rx_byte = 0;
uint8_t bp_packet[4]; // header + SYS + DIA + BPM
uint8_t bp_index = 0;
volatile uint8_t bp_ready = 0;
uint8_t success = 0;
uint8_t sys;
uint8_t dia;
uint8_t bpm;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t DHT11_Read(uint8_t *temp, uint8_t *humi);
void delay_us(uint16_t us);
void parse_GNGLL(const char *sentence);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float Read_LM35(void) {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
	return ((float) adc_value * 3.3f / 4095.0f) * 100.0f;
}

void GSM_SendSMS(char *number, char *message) {
	char cmd[100];

	// Initialize GSM Module
	HAL_UART_Transmit(&huart3, (uint8_t*) "AT\r\n", 4, 1000);
	HAL_Delay(1000);

	HAL_UART_Transmit(&huart3, (uint8_t*) "AT+CMGF=1\r\n", 11, 1000); // Set to text mode
	HAL_Delay(1000);

	// Set recipient number
	sprintf(cmd, "AT+CMGS=\"%s\"\r\n", number);
	HAL_UART_Transmit(&huart3, (uint8_t*) cmd, strlen(cmd), 1000);
	HAL_Delay(1000);

	// Send message content
	HAL_UART_Transmit(&huart3, (uint8_t*) message, strlen(message), 1000);
	HAL_Delay(500);

	// Send Ctrl+Z to send the SMS
	uint8_t end_code = 0x1A; // ASCII for Ctrl+Z
	HAL_UART_Transmit(&huart3, &end_code, 1, 1000);

	HAL_Delay(3000); // Wait until SMS is sent
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		flag_1s = 1;
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
	MX_TIM2_Init();
	MX_USART1_UART_Init();
	MX_ADC1_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start(&hadc1);

	HAL_UART_Transmit(&huart1, (uint8_t*) "Hello from STM32 UART1\r\n", 25,
	HAL_MAX_DELAY);
	HAL_UART_Receive_IT(&huart1, &bp_rx_byte, 1); // Start ESP32 reception
	HAL_UART_Receive_IT(&huart2, &rx_data, 1);   // Start GPS reception
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	/* USER CODE BEGIN 3 */
	/* USER CODE BEGIN 3 */
	while (1) {
		// --- 1. Continuous live printing every 1 second using TIM2 flag ---
		if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE)) {
			if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET) {
				__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

				// Check if a full BP packet is ready
				if (bp_ready) {
					char dbg[64];
					sprintf(dbg, "RX Packet: %02X %02X %02X %02X\r\n",
							bp_packet[0], bp_packet[1], bp_packet[2],
							bp_packet[3]);
					HAL_UART_Transmit(&huart1, (uint8_t*) dbg, strlen(dbg),
					HAL_MAX_DELAY);
				}

				// Compose live UART string
				snprintf(uart_buffer, sizeof(uart_buffer),
						"DHT11: T=%dC H=%d%% | LM35: %.2fC | MAX30102: RED=%lu IR=%lu | GPS: Lat=%s Lon=%s | BP: SYS=%d DIA=%d BPM=%d\r\n",
						temp, hum, tempC, red, ir, latitude, longitude, sys,
						dia, bpm);

				snprintf(sms_message, sizeof(sms_message),
						"DHT11: T=%dC H=%d%%\nLM35: %.2fC\nMAX30102: RED=%lu IR=%lu\nGPS: Lat=%s Lon=%s\nBP: SYS=%d DIA=%d BPM=%d",
						temp, hum, tempC, red, ir, latitude, longitude, sys,
						dia, bpm);

				HAL_UART_Transmit(&huart1, (uint8_t*) uart_buffer,
						strlen(uart_buffer), HAL_MAX_DELAY);
			}
		}

		// --- 2. Check push button for SMS sending ---
		if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_RESET) {
			// --- Read sensors ---
			success = 0;
			for (int i = 0; i < MAX_DHT_ATTEMPTS; i++) {
				if (DHT11_Read(&temp, &hum) == 0) {
					success = 1;
					break;
				}
				HAL_Delay(100);
			}

			tempC = Read_LM35();  // LM35 temperature

			if (gps_ready) {
				gps_ready = 0;
				if (strncmp(gps_buffer, "$GNGLL", 6) == 0) {
					parse_GNGLL(gps_buffer);  // update latitude and longitude
				}
			} else {
				strcpy(latitude, "NA");
				strcpy(longitude, "NA");
			}

			// Use latest BP values if available
			if (bp_ready) {
				sys = bp_packet[1];
				dia = bp_packet[2];
				bpm = bp_packet[3];
				bp_ready = 0; // reset flag
			} else {
				sys = 20;
				dia = 30;
				bpm = 40;
			}

			// Compose SMS content
			sprintf(sms_message,
					"DHT11: T=%dC H=%d%%\nLM35: %.2fC\nMAX30102: RED=%lu IR=%lu\nGPS: Lat=%s Lon=%s\nBP: SYS=%d DIA=%d BPM=%d",
					temp, hum, tempC, red, ir, latitude, longitude, sys, dia,
					bpm);

			// Send SMS
			GSM_SendSMS("+919985798499", sms_message);

			// --- Debounce ---
			while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_RESET)
				;
			HAL_Delay(300);
		}

		// --- 3. Optional small delay for CPU relief ---
		HAL_Delay(10);
	}
	/* USER CODE END 3 */
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 71;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
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
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_USART3_UART_Init(void) {

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
	if (HAL_UART_Init(&huart3) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
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
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	if (huart->Instance == USART2) {
//		if (rx_index < sizeof(gps_buffer) - 1) {
//			if (rx_data == '\n') {
//				gps_buffer[rx_index] = '\0';
//				gps_ready = 1;
//				rx_index = 0;
//			} else {
//				gps_buffer[rx_index++] = rx_data;
//			}
//		}
//		HAL_UART_Receive_IT(&huart2, &rx_data, 1);  // Continue reception
//	}
//}
void parse_GNGLL(const char *sentence) {
	char buffer[128];
	strcpy(buffer, sentence);

	char *token;
	char *fields[10] = { 0 };
	int i = 0;

	token = strtok(buffer, ",");
	while (token && i < 10) {
		fields[i++] = token;
		token = strtok(NULL, ",");
	}

	if (i < 6 || !fields[1] || !fields[3])
		return;

	float raw_lat = atof(fields[1]);
	float raw_lon = atof(fields[3]);

	int lat_deg = (int) (raw_lat / 100);
	float lat_min = raw_lat - (lat_deg * 100);
	float lat_decimal = lat_deg + (lat_min / 60.0f);

	int lon_deg = (int) (raw_lon / 100);
	float lon_min = raw_lon - (lon_deg * 100);
	float lon_decimal = lon_deg + (lon_min / 60.0f);

	snprintf(latitude, sizeof(latitude), "%.5f%s", lat_decimal, fields[2]);
	snprintf(longitude, sizeof(longitude), "%.5f%s", lon_decimal, fields[4]);

	char result[128];
	snprintf(result, sizeof(result), "Latitude: %s\r\nLongitude: %s\r\n\r\n",
			latitude, longitude);
	HAL_UART_Transmit(&huart1, (uint8_t*) result, strlen(result),
	HAL_MAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) { // ESP32 BP
		// Simple state machine
		if (bp_index == 0) {
			if (bp_rx_byte == BP_HEADER) {
				bp_packet[bp_index++] = bp_rx_byte;
			}
		} else {
			bp_packet[bp_index++] = bp_rx_byte;
			if (bp_index >= 4) { // header + 3 bytes
				bp_index = 0;
				bp_ready = 1; // packet complete
			}
		}
		HAL_UART_Receive_IT(&huart1, &bp_rx_byte, 1); // restart reception
	}

	if (huart->Instance == USART2) { // GPS
		if (rx_index < sizeof(gps_buffer) - 1) {
			if (rx_data == '\n') {
				gps_buffer[rx_index] = '\0';
				gps_ready = 1;
				rx_index = 0;
			} else {
				gps_buffer[rx_index++] = rx_data;
			}
		}
		HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
