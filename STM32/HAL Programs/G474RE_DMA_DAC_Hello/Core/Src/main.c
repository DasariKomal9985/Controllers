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
#include <math.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DAC_MAX       4095
#define SAMPLE_RATE   40000   // 40 kHz DAC update
#define BIT_RATE      800     // bits per second
#define MARK_FREQ     2400    // Hz
#define SPACE_FREQ    1200    // Hz
#define SAMPLES_PER_BIT (SAMPLE_RATE / BIT_RATE)
#define BUFFER_SIZE   (SAMPLES_PER_BIT * 8*10) // enough for 10 chars
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t dac_buffer[BUFFER_SIZE];

/* Bitstream */
const char *msg = "hello";
uint8_t bitstream[8 * 10];  // 8 bits per char, max 10 chars
uint32_t total_bits;
uint16_t adc_samples[SAMPLES_PER_BIT];
uint8_t received_bits[8 * 10];
uint32_t rx_idx = 0;
// store decoded characters here
#define MAX_DECODED 20
char decoded_msg[MAX_DECODED + 1] = { 0 };  // +1 for null terminator
uint32_t decoded_idx = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac2;
DMA_HandleTypeDef hdma_dac2_ch1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void StringToBits(const char *str, uint8_t *bits, uint32_t *n_bits) {
	uint32_t idx = 0;
	for (uint32_t i = 0; i < strlen(str); i++) {
		for (int b = 7; b >= 0; b--) {
			bits[idx++] = (str[i] >> b) & 0x1;
		}
	}
	*n_bits = idx;
}

void FillDACBufferAFSK(void) {
	uint32_t idx = 0;
	for (uint32_t b = 0; b < total_bits; b++) {
		double freq = (bitstream[b]) ? MARK_FREQ : SPACE_FREQ;
		for (int i = 0; i < SAMPLES_PER_BIT; i++) {
			double t = (double) i / SAMPLE_RATE;
			dac_buffer[idx++] = (uint16_t) ((sin(2 * M_PI * freq * t) + 1.0)
					* (DAC_MAX / 2));
		}
	}
}

/* Simple Goertzel frequency detection for one block */
uint8_t DetectBit(uint16_t *samples) {
	double s_prev = 0, s_prev2 = 0;
	double normalized_mark = 2.0 * cos(2 * M_PI * MARK_FREQ / SAMPLE_RATE);
	double normalized_space = 2.0 * cos(2 * M_PI * SPACE_FREQ / SAMPLE_RATE);
	double s, power_mark, power_space;

	// MARK detection
	for (int i = 0; i < SAMPLES_PER_BIT; i++) {
		s = samples[i] + normalized_mark * s_prev - s_prev2;
		s_prev2 = s_prev;
		s_prev = s;
	}
	power_mark = s_prev2 * s_prev2 + s_prev * s_prev
			- normalized_mark * s_prev * s_prev2;

	s_prev = s_prev2 = 0;
	// SPACE detection
	for (int i = 0; i < SAMPLES_PER_BIT; i++) {
		s = samples[i] + normalized_space * s_prev - s_prev2;
		s_prev2 = s_prev;
		s_prev = s;
	}
	power_space = s_prev2 * s_prev2 + s_prev * s_prev
			- normalized_space * s_prev * s_prev2;

	return (power_mark > power_space) ? 1 : 0;
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
	MX_DMA_Init();
	MX_DAC2_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	StringToBits(msg, bitstream, &total_bits);

	// Fill DAC buffer
	FillDACBufferAFSK();

	// Start DAC streaming via DMA
	HAL_TIM_Base_Start(&htim2);
	HAL_DAC_Start_DMA(&hdac2, DAC_CHANNEL_1, (uint32_t*) dac_buffer,
			total_bits * SAMPLES_PER_BIT, DAC_ALIGN_12B_R);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		for (int i = 0; i < SAMPLES_PER_BIT; i++) {
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 10);
			adc_samples[i] = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
		}

		// Detect bit
		received_bits[rx_idx++] = DetectBit(adc_samples);

		// Reconstruct character every 8 bits
		if (rx_idx % 8 == 0) {
			uint8_t c = 0;
			for (int b = 0; b < 8; b++) {
				c = (c << 1) | received_bits[rx_idx - 8 + b];
			}

			// Store in buffer for Live Expressions
			if (decoded_idx < MAX_DECODED) {
				decoded_msg[decoded_idx++] = c;
				decoded_msg[decoded_idx] = '\0';  // null-terminate
			}
		}

		// Reset rx_idx if all bits processed
		if (rx_idx >= total_bits)
			rx_idx = 0;

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

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

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
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.GainCompensation = 0;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief DAC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC2_Init(void) {

	/* USER CODE BEGIN DAC2_Init 0 */

	/* USER CODE END DAC2_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC2_Init 1 */

	/* USER CODE END DAC2_Init 1 */

	/** DAC Initialization
	 */
	hdac2.Instance = DAC2;
	if (HAL_DAC_Init(&hdac2) != HAL_OK) {
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
	sConfig.DAC_DMADoubleDataMode = DISABLE;
	sConfig.DAC_SignedFormat = DISABLE;
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
	sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_BOTH;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC2_Init 2 */

	/* USER CODE END DAC2_Init 2 */

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
	htim2.Init.Prescaler = 15;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 265;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

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
