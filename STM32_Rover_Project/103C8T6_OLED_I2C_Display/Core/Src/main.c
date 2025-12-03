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
#include "ssd1306.h"
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t cursor = 1;
uint8_t menu = 0;   // 0 = main menu, 1 = test equipment
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void ShowMainMenu(uint8_t sel);
void ShowTestEquipmentMenu(uint8_t sel);
void Box(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Box(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
	SSD1306_DrawRect(x, y, w, h);
}

/* MAIN MENU */
void ShowMainMenu(uint8_t sel) {
	SSD1306_Clear();

	SSD1306_WriteStringXY(0, 10,
			(sel == 1) ? "> 1) Test Equipment" : "  1) Test Equipment");

	SSD1306_WriteStringXY(0, 25,
			(sel == 2) ? "> 2) NRF Control" : "  2) NRF Control");

	SSD1306_WriteStringXY(0, 40,
			(sel == 3) ? "> 3) Autonomus Ctrl" : "  3) Autonomus Ctrl");

	SSD1306_Update();
}

/* TEST EQUIPMENT SUB-MENU */
void ShowTestEquipmentMenu(uint8_t sel) {
	SSD1306_Clear();

	SSD1306_WriteStringXY(0, 5,
			(sel == 1) ? "> 1) Test Joystick" : "  1) Test Joystick");

	SSD1306_WriteStringXY(0, 20,
			(sel == 2) ? "> 2) ESP32 Serial" : "  2) ESP32 Serial");

	SSD1306_WriteStringXY(0, 35,
			(sel == 3) ? "> 3) NRF Module" : "  3) NRF Module");

	SSD1306_WriteStringXY(0, 55,
			(sel == 4) ? "> 4) Potentiometer" : "  4) Potentiometer");

	SSD1306_Update();
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	SSD1306_Init();

	// -------- FIRST SCREEN --------
	SSD1306_Clear();
	SSD1306_WriteStringXY(0, 0, "Hello");
	SSD1306_WriteStringXY(0, 15, "Dasari!");
	SSD1306_Update();
	HAL_Delay(3000);

	// -------- PASSCODE SCREEN --------
	SSD1306_Clear();
	SSD1306_WriteStringXY(0, 0, "Enter PassCode:");
	Box(0, 20, 120, 20);
	SSD1306_Update();

	char pass[10] = { 0 };
	uint8_t idx = 0;
	uint8_t ch;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY);

		if (ch == '\r')   // ENTER to verify
				{
			if (strcmp(pass, "9985") == 0)
				break;   // correct → unlock
			else {
				// --- SHOW WRONG MESSAGE ---
				SSD1306_Clear();                    // clear FIRST
				SSD1306_WriteStringXY(0, 0, "Enter PassCode:");
				Box(0, 20, 120, 20);
				SSD1306_WriteStringXY(0, 45, "Wrong! Try Again");
				SSD1306_Update();
				HAL_Delay(800);

				// reset
				memset(pass, 0, sizeof(pass));
				idx = 0;

				// --- SHOW CLEAN EMPTY PASSCODE SCREEN ---
				SSD1306_Clear();
				SSD1306_WriteStringXY(0, 0, "Enter PassCode:");
				Box(0, 20, 120, 20);
				SSD1306_Update();

			}
		} else if (idx < 4)   // store pass digits
				{
			pass[idx++] = ch;
			pass[idx] = '\0';

			SSD1306_Clear();
			SSD1306_WriteStringXY(0, 0, "Enter PassCode:");
			Box(0, 20, 120, 20);
			SSD1306_WriteStringXY(5, 25, pass);
			SSD1306_Update();
		}
	}

	// -------- OPEN MAIN MENU --------
	menu = 0;
	cursor = 1;
	ShowMainMenu(cursor);

	/* USER CODE END 2 */

	/* USER CODE BEGIN WHILE */
	while (1) {
		HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY);

		// ---- ARROW KEYS (TeraTerm ESC sequences) ----
		if (ch == 0x1B) {
			uint8_t b1, b2;
			HAL_UART_Receive(&huart2, &b1, 1, HAL_MAX_DELAY);
			HAL_UART_Receive(&huart2, &b2, 1, HAL_MAX_DELAY);

			if (b1 == '[') {
				if (b2 == 'A')
					cursor--;   // UP
				if (b2 == 'B')
					cursor++;   // DOWN
			}
		}

		// ---- LIMITS ----
		if (menu == 0) {
			if (cursor < 1)
				cursor = 3;
			if (cursor > 3)
				cursor = 1;
		} else if (menu == 1) {
			if (cursor < 1)
				cursor = 4;
			if (cursor > 4)
				cursor = 1;
		}

		// ---- DISPLAY MENUS ----
		if (menu == 0)
			ShowMainMenu(cursor);
		if (menu == 1)
			ShowTestEquipmentMenu(cursor);

		// ---- ENTER KEY ----
		if (ch == '\r') {
			if (menu == 0) {
				if (cursor == 1) {
					menu = 1;
					cursor = 1;
					ShowTestEquipmentMenu(cursor);
				}
				if (cursor == 2) { /* open NRF control */
				}
				if (cursor == 3) { /* open autonomous ctrl */
				}
			} else if (menu == 1) {
				if (cursor == 1) { /* joystick test */
				}
				if (cursor == 2) { /* ESP32 serial test */
				}
				if (cursor == 3) { /* NRF module test */
				}
				if (cursor == 4) { /* potentiometer test */
				}
			}
		}

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
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
