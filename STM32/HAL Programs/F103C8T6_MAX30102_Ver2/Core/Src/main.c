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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX30102_ADDR      (0x57 << 1)
#define REG_MODE_CONFIG    0x09
#define REG_SPO2_CONFIG    0x0A
#define REG_LED1_PA        0x0C
#define REG_LED2_PA        0x0D
#define REG_FIFO_DATA      0x07
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t rx_byte;
char buf[128];
uint32_t red, ir;
int sample_index = 0;

#define SAMPLE_SIZE 100
uint32_t red_samples[SAMPLE_SIZE];
uint32_t ir_samples[SAMPLE_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void uart_print(char *msg) {
	HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

HAL_StatusTypeDef MAX30102_WriteReg(uint8_t reg, uint8_t data) {
	return HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
			&data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MAX30102_ReadFIFO(uint32_t *red, uint32_t *ir) {
	uint8_t data[6];
	if (HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR, REG_FIFO_DATA,
	I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY) != HAL_OK)
		return HAL_ERROR;
	*red = ((uint32_t) data[0] << 16) | ((uint32_t) data[1] << 8) | data[2];
	*ir = ((uint32_t) data[3] << 16) | ((uint32_t) data[4] << 8) | data[5];
	return HAL_OK;
}

void MAX30102_Init(void) {
	HAL_Delay(100);
	MAX30102_WriteReg(REG_MODE_CONFIG, 0x03);
	MAX30102_WriteReg(REG_LED1_PA, 0x1F); // Red
	MAX30102_WriteReg(REG_LED2_PA, 0x1F); // IR
	MAX30102_WriteReg(REG_SPO2_CONFIG, 0x27);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		HAL_UART_Transmit(&huart1, &rx_byte, 1, HAL_MAX_DELAY);
		HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
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
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

	uart_print("Initializing MAX30102...\r\n");
	MAX30102_Init();
	HAL_Delay(100);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (MAX30102_ReadFIFO(&red, &ir) == HAL_OK) {
			// store samples
			red_samples[sample_index] = red;
			ir_samples[sample_index] = ir;
			sample_index++;

			if (sample_index >= SAMPLE_SIZE) {
				// --- Heart Rate & SpO2 Calculation ---
				// Compute DC (average)
				double red_dc = 0, ir_dc = 0;
				for (int i = 0; i < SAMPLE_SIZE; i++) {
					red_dc += red_samples[i];
					ir_dc += ir_samples[i];
				}
				red_dc /= SAMPLE_SIZE;
				ir_dc /= SAMPLE_SIZE;

				// Compute AC (peak-to-peak)
				double red_ac = 0, ir_ac = 0;
				uint32_t red_min = red_samples[0], red_max = red_samples[0];
				uint32_t ir_min = ir_samples[0], ir_max = ir_samples[0];
				for (int i = 0; i < SAMPLE_SIZE; i++) {
					if (red_samples[i] < red_min)
						red_min = red_samples[i];
					if (red_samples[i] > red_max)
						red_max = red_samples[i];
					if (ir_samples[i] < ir_min)
						ir_min = ir_samples[i];
					if (ir_samples[i] > ir_max)
						ir_max = ir_samples[i];
				}
				red_ac = red_max - red_min;
				ir_ac = ir_max - ir_min;

				// SpO2 estimation
				double ratio = (red_ac / red_dc) / (ir_ac / ir_dc);
				double spo2 = 110 - 25 * ratio;

				// Heart rate estimation (very basic: count IR peaks)
				int beats = 0;
				for (int i = 1; i < SAMPLE_SIZE - 1; i++) {
					if (ir_samples[i] > ir_samples[i - 1]
							&& ir_samples[i] > ir_samples[i + 1]
							&& ir_samples[i] > ir_dc) {
						beats++;
					}
				}
				// Extrapolate BPM for 100 samples (~10 sec at 10Hz)
				double bpm = beats * 6.0; // 10 sec window → 60 sec

				// Print results
				sprintf(buf, "HR: %.0f BPM, SpO2: %.1f %%\r\n", bpm, spo2);
				uart_print(buf);

				sample_index = 0; // reset sample buffer
			}
		}
		HAL_Delay(100); // 10Hz sample rate
	}
}
/* USER CODE END 3 */


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
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
	Error_Handler();
}
/* USER CODE BEGIN I2C1_Init 2 */

/* USER CODE END I2C1_Init 2 */

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
__HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
