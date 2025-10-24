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
#include <stdbool.h>  // ✅ Required for bool, true, false

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
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define RX_BUFFER_SIZE 512

char esp_buffer[RX_BUFFER_SIZE];
char esp_line[RX_BUFFER_SIZE];
uint16_t esp_index = 0;

uint8_t rx_byte;
uint32_t last_sms_time = 0;
uint8_t sms_pending = 0;
uint8_t buzzer_on = 0;
uint32_t buzzer_start = 0;

// Buzzer and button control

uint8_t button_muted = 0;
uint32_t button_mute_start = 0;


char latitude[16] = "17.46206N";
char longitude[16] = "78.59537E";

const char *phone_numbers[] = { "9985798499", "9100504603", "7286996386",
		"9154797255" };
const uint8_t phone_count = sizeof(phone_numbers) / sizeof(phone_numbers[0]);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void send_alert_to_all(char *message);
bool detect_snake(const char *text);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool detect_snake(const char *text) {
	const char *snake_names[] = { "green_mamba", "horned_viper",
			"boa_constrictor", "garter_snake", "sea_snake", "diamondback",
			"Indian_cobra", "green_snake", "rock_python", "banded_krait" };

	for (uint8_t i = 0; i < sizeof(snake_names) / sizeof(snake_names[0]); i++) {
		if (strstr(text, snake_names[i]) != NULL) {
			return true;
		}
	}
	return false;
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
	MX_USART3_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Transmit(&huart1, (uint8_t*) "STM32 Ready...\r\n", 16,
	HAL_MAX_DELAY);

	HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		uint32_t now = HAL_GetTick();

		// --- Button to temporarily mute buzzer ---
		uint8_t button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
		if (button_state == GPIO_PIN_RESET && !button_muted) {
			button_muted = 1;
			button_mute_start = now;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // turn off buzzer
		}

		// Unmute after 3 seconds
		if (button_muted && (now - button_mute_start >= 3000)) {
			button_muted = 0;
		}

		// --- Turn off buzzer automatically after 1 second ---
		if (!button_muted && buzzer_on && (now - buzzer_start >= 1000)) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			buzzer_on = 0;
		}

		// --- Manual SMS button ---
		if (button_state == GPIO_PIN_RESET && !sms_pending) {
			sms_pending = 1;
			last_sms_time = now;
		}

		if (sms_pending && (now - last_sms_time >= 0)) {
			char sms_payload[200];
			snprintf(sms_payload, sizeof(sms_payload),
					" DATA=%.100s", esp_line);
			send_alert_to_all(sms_payload);

			HAL_UART_Transmit(&huart1, (uint8_t*) "Manual SMS Sent to All!\r\n",
					25, HAL_MAX_DELAY);
			sms_pending = 2;
		}

		if (button_state == GPIO_PIN_SET && sms_pending == 2) {
			sms_pending = 0;
			HAL_Delay(300);
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
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		if (esp_index < RX_BUFFER_SIZE - 1) {
			if (rx_byte == '\n') {
				esp_buffer[esp_index] = '\0';
				strcpy(esp_line, esp_buffer);
				esp_index = 0;

				HAL_UART_Transmit(&huart1, (uint8_t*) esp_line,
						strlen(esp_line), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n", 2, HAL_MAX_DELAY);

				// --- Snake detection ---
				if (detect_snake(esp_line) && !buzzer_on && !button_muted) {
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // buzzer ON
					buzzer_on = 1;
					buzzer_start = HAL_GetTick();

					HAL_UART_Transmit(&huart1, (uint8_t*) "Snake Detected!\r\n",
							17, HAL_MAX_DELAY);
				}
			} else {
				esp_buffer[esp_index++] = rx_byte;
			}
		} else {
			esp_index = 0; // overflow protection
		}

		HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
	}
}

void send_sms(char *number, char *message) {
	char cmd[160];
	sprintf(cmd, "AT+CMGF=1\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_Delay(500);

	sprintf(cmd, "AT+CMGS=\"%s\"\r\n", number);
	HAL_UART_Transmit(&huart2, (uint8_t*) cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_Delay(500);

	HAL_UART_Transmit(&huart2, (uint8_t*) message, strlen(message),
	HAL_MAX_DELAY);
	HAL_Delay(500);

	uint8_t ctrl_z = 26;
	HAL_UART_Transmit(&huart2, &ctrl_z, 1, HAL_MAX_DELAY);
	HAL_Delay(5000);
}
void send_alert_to_all(char *message) {
	for (uint8_t i = 0; i < phone_count; i++) {
		send_sms((char*) phone_numbers[i], message);
		HAL_UART_Transmit(&huart1, (uint8_t*) "SMS sent to: ", 13,
		HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t*) phone_numbers[i],
				strlen(phone_numbers[i]), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n", 2, HAL_MAX_DELAY);
		HAL_Delay(2000); // short delay between each message
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
