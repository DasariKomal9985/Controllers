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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define GPS_BUFFER_SIZE 256
uint8_t rx2_byte;
volatile char gps_buffer[GPS_BUFFER_SIZE];
volatile uint16_t gps_index = 0;
volatile uint8_t sentence_ready = 0;
char nmea_sentence[GPS_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void parse_nmea(char *nmea);
void process_gpgga(char *sentence);
void process_gprmc(char *sentence);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, HAL_MAX_DELAY);
	return len;
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
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	printf("\r\n7Semi NEO-M9N GPS Reader (STM32)\r\n");
	printf("Waiting for satellite data...\r\n\r\n");

	HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (sentence_ready) {
			__disable_irq();
			strcpy(nmea_sentence, (char*) gps_buffer);
			sentence_ready = 0;
			__enable_irq();

			parse_nmea(nmea_sentence);
		}
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
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
	huart1.Init.BaudRate = 115200;
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
	huart2.Init.BaudRate = 38400;
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		if (rx2_byte == '\n') {
			gps_buffer[gps_index] = '\0';
			gps_index = 0;
			sentence_ready = 1;
		} else {
			if (gps_index < GPS_BUFFER_SIZE - 1)
				gps_buffer[gps_index++] = rx2_byte;
		}

		HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
	}
}

void parse_nmea(char *nmea) {
	if (strstr(nmea, "$GPGGA") || strstr(nmea, "$GNGGA")) {
		process_gpgga(nmea);
	} else if (strstr(nmea, "$GPRMC") || strstr(nmea, "$GNRMC")) {
		process_gprmc(nmea);
	}
}

// --- GGA: Altitude, Satellites, HDOP ---
void process_gpgga(char *sentence) {
	char copy[GPS_BUFFER_SIZE];
	strcpy(copy, sentence);

	char *fields[15];
	int i = 0;
	char *token = strtok(copy, ",");
	while (token && i < 15) {
		fields[i++] = token;
		token = strtok(NULL, ",");
	}

	if (i < 10)
		return;

	double raw_lat = atof(fields[2]);
	double raw_lon = atof(fields[4]);
	char ns = fields[3][0];
	char ew = fields[5][0];
	int sats = atoi(fields[7]);
	double hdop = atof(fields[8]);
	double alt = atof(fields[9]);

	if (raw_lat == 0 || raw_lon == 0)
		return;

	int lat_deg = (int) (raw_lat / 100);
	double lat_min = raw_lat - (lat_deg * 100);
	double latitude = lat_deg + (lat_min / 60.0);

	int lon_deg = (int) (raw_lon / 100);
	double lon_min = raw_lon - (lon_deg * 100);
	double longitude = lon_deg + (lon_min / 60.0);

	if (ns == 'S')
		latitude = -latitude;
	if (ew == 'W')
		longitude = -longitude;

	printf("===============================\r\n");
	printf("Latitude  : %.7f\r\n", latitude);
	printf("Longitude : %.7f\r\n", longitude);
	printf("Satellites: %d\r\n", sats);
	printf("Accuracy  : ±%.1f m\r\n", hdop);
	printf("Altitude  : %.1f m\r\n", alt);
	printf("===============================\r\n\r\n");
}

// --- RMC: Optional, only prints coords if GGA missed ---
void process_gprmc(char *sentence) {
	char copy[GPS_BUFFER_SIZE];
	strcpy(copy, sentence);

	char *fields[15];
	int i = 0;
	char *token = strtok(copy, ",");
	while (token && i < 15) {
		fields[i++] = token;
		token = strtok(NULL, ",");
	}

	if (i < 7)
		return;

	double raw_lat = atof(fields[3]);
	double raw_lon = atof(fields[5]);
	char ns = fields[4][0];
	char ew = fields[6][0];

	if (raw_lat == 0 || raw_lon == 0)
		return;

	int lat_deg = (int) (raw_lat / 100);
	double lat_min = raw_lat - (lat_deg * 100);
	double latitude = lat_deg + (lat_min / 60.0);

	int lon_deg = (int) (raw_lon / 100);
	double lon_min = raw_lon - (lon_deg * 100);
	double longitude = lon_deg + (lon_min / 60.0);

	if (ns == 'S')
		latitude = -latitude;
	if (ew == 'W')
		longitude = -longitude;

	printf("[RMC backup] Lat: %.7f, Lon: %.7f\r\n", latitude, longitude);
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
