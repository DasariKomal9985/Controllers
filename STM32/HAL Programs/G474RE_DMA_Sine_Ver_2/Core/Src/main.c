
/* USER CODE END Header */

#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DAC_MAX       4095
#define SAMPLE_RATE   40000      // timer triggers per second (TIM2 TRGO) ~40kHz
#define BUFFER_LEN    200        // DAC DMA buffer length (even number)

/* AFSK tones */
#define TONE_MARK     1200.0     // Hz (MARK)
#define TONE_SPACE    2400.0     // Hz (SPACE)

/* ADC / Goertzel */
#define ADC_SAMPLES   64         // number of ADC samples per Goertzel window (choose ~32..128)
#define GOERTZEL_LEN  ADC_SAMPLES

/* Toggle period for demonstration */
#define TOGGLE_MS     1000       // toggle tone every 1000 ms so you can see both on scope

/* USER CODE END PTD */

uint16_t dac_buffer[BUFFER_LEN];            // DAC circular buffer (DMA)
uint16_t adc_buffer[ADC_SAMPLES];           // ADC circular buffer (DMA)

/* continuous-phase generator state */
volatile double phase = 0.0;
volatile double phase_inc = 0.0;            // set to MARK initially below

/* last Goertzel results (expose as volatile so CubeIDE Live Expression can read them) */
volatile double last_power_mark = 0.0;
volatile double last_power_space = 0.0;
volatile uint8_t detected_tone = 0xFF; // 1 = MARK, 0 = SPACE, 0xFF = undefined/noise
volatile uint8_t goertzel_ready_flag = 0;  // main loop reads and clears

/* Goertzel coefficients */
double coeff_mark = 0.0;
double coeff_space = 0.0;

/* UART4 printing buffer */
char uart_msg[80];

/* UART4 printing buffer */
char uart_msg[80];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* prototypes */
void InitGoertzel(void);
uint8_t GoertzelDetect(uint16_t *samples, int N);
void FillDACHalf(int offset, int length);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac2;
DMA_HandleTypeDef hdma_dac2_ch1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC2_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Fill a half of dac_buffer (offset can be 0 or BUFFER_LEN/2) in continuous-phase mode.
 This keeps phase continuous across calls. Must be called from DAC DMA callbacks. */
void FillDACHalf(int offset, int length) {
	// local copy of phase and phase_inc for speed (volatile updated)
	double p = phase;
	double inc = phase_inc;

	for (int i = 0; i < length; ++i) {
		double s = sin(p);
		uint16_t sample = (uint16_t) ((s + 1.0) * (DAC_MAX / 2.0));
		dac_buffer[offset + i] = sample;

		p += inc;
		if (p >= 2.0 * M_PI)
			p -= 2.0 * M_PI;
		if (p < 0.0)
			p += 2.0 * M_PI;
	}

	// write back the phase atomically (volatile)
	phase = p;
}

/* Goertzel init — compute 2*cos(omega) coefficients for each target tone */
void InitGoertzel(void) {
	double omega_mark = 2.0 * M_PI * (double) TONE_MARK / (double) SAMPLE_RATE;
	double omega_space = 2.0 * M_PI * (double) TONE_SPACE / (double) SAMPLE_RATE;
	coeff_mark = 2.0 * cos(omega_mark);
	coeff_space = 2.0 * cos(omega_space);
}

/* Run Goertzel on `N` samples and return 1 if MARK (1200 Hz) stronger, 0 if SPACE stronger.
 Also update last_power_mark / last_power_space for inspection. */
uint8_t GoertzelDetect(uint16_t *samples, int N) {
	double mean = 0.0;
	for (int i = 0; i < N; ++i)
		mean += samples[i];
	mean /= N;

	double s_prev_mark = 0.0, s_prev2_mark = 0.0;
	double s_prev_space = 0.0, s_prev2_space = 0.0;

	for (int i = 0; i < N; ++i) {
		double x = (double) samples[i] - mean; // simple DC removal

		double s_mark = x + coeff_mark * s_prev_mark - s_prev2_mark;
		s_prev2_mark = s_prev_mark;
		s_prev_mark = s_mark;

		double s_space = x + coeff_space * s_prev_space - s_prev2_space;
		s_prev2_space = s_prev_space;
		s_prev_space = s_space;
	}

	double power_mark = s_prev_mark * s_prev_mark + s_prev2_mark * s_prev2_mark
			- coeff_mark * s_prev_mark * s_prev2_mark;
	double power_space = s_prev_space * s_prev_space
			+ s_prev2_space * s_prev2_space
			- coeff_space * s_prev_space * s_prev2_space;

	// store for live inspection
	last_power_mark = power_mark;
	last_power_space = power_space;

	// simple thresholding: if both powers are very small treat as noise (skip)
	double total_power = power_mark + power_space;
	if (total_power < 1e6) { // threshold — tune for your signal level
		return 0xFF; // no reliable tone
	}
	return (power_mark > power_space) ? 1 : 0;
}

/* --------------------------------------------------------------------------
 DMA Callbacks for DAC (these are called by HAL when DMA half/full completes)
 We refill the half that just completed so the DMA always reads valid data.
 --------------------------------------------------------------------------*/
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	(void) hdac;
	// refill first half
	FillDACHalf(0, BUFFER_LEN / 2);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	(void) hdac;
	// refill second half
	FillDACHalf(BUFFER_LEN / 2, BUFFER_LEN / 2);
}

/* ADC DMA complete callback — process the full ADC buffer */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	(void) hadc;
	// Process the just-completed ADC buffer (adc_buffer)
	uint8_t tone = GoertzelDetect(adc_buffer, GOERTZEL_LEN);
	if (tone != 0xFF) {
		detected_tone = tone;
	} else {
		detected_tone = 0xFF;
	}
	// notify main loop (do not printf from here)
	goertzel_ready_flag = 1;
}

/* Safe putchar / printf redirection to UART4 (used in main loop only) */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart4, (uint8_t*) ptr, len, HAL_MAX_DELAY);
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
	MX_DMA_Init();
	MX_TIM2_Init();
	MX_DAC2_Init();
	MX_ADC1_Init();
	MX_UART4_Init();
	/* USER CODE BEGIN 2 */
	/* USER CODE BEGIN 2 */

	phase_inc = 2.0 * M_PI * TONE_MARK / (double) SAMPLE_RATE;
	FillDACHalf(0, BUFFER_LEN / 2);
	FillDACHalf(BUFFER_LEN / 2, BUFFER_LEN / 2);

	/* start timer (triggers DAC + ADC), start DAC DMA circular and ADC DMA circular */
	HAL_TIM_Base_Start(&htim2);

	HAL_DAC_Start_DMA(&hdac2, DAC_CHANNEL_1, (uint32_t*) dac_buffer, BUFFER_LEN,
	DAC_ALIGN_12B_R);

	/* Start ADC DMA circular: buffer length ADC_SAMPLES */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, ADC_SAMPLES);

	/* Goertzel constants */
	InitGoertzel();

	/* notify via UART that system started */
	snprintf(uart_msg, sizeof(uart_msg),
			"AFSK generator started: MARK=%dHz, SPACE=%dHz\r\n",
			(int) TONE_MARK, (int) TONE_SPACE);
	HAL_UART_Transmit(&huart4, (uint8_t*) uart_msg, strlen(uart_msg),
	HAL_MAX_DELAY);

	uint32_t last_toggle = HAL_GetTick();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		uint32_t now = HAL_GetTick();
		if ((now - last_toggle) >= TOGGLE_MS) {
			last_toggle = now;
			if (phase_inc == (2.0 * M_PI * TONE_MARK / (double) SAMPLE_RATE)) {
				phase_inc = 2.0 * M_PI * TONE_SPACE / (double) SAMPLE_RATE;
				snprintf(uart_msg, sizeof(uart_msg),
						"Switch -> SPACE (%.0f Hz)\r\n", TONE_SPACE);
			} else {
				phase_inc = 2.0 * M_PI * TONE_MARK / (double) SAMPLE_RATE;
				snprintf(uart_msg, sizeof(uart_msg),
						"Switch -> MARK (%.0f Hz)\r\n", TONE_MARK);
			}
			HAL_UART_Transmit(&huart4, (uint8_t*) uart_msg, strlen(uart_msg),
			HAL_MAX_DELAY);
		}

		/* when Goertzel processed new block, print values (done in main loop to keep IRQ fast) */
		if (goertzel_ready_flag) {
			goertzel_ready_flag = 0;

			// detected_tone can be 1 (MARK) or 0 (SPACE) or 0xFF (noise)
			if (detected_tone == 1) {
				snprintf(uart_msg, sizeof(uart_msg),
						"DET: MARK  Pm=%.0f Ps=%.0f\r\n", last_power_mark,
						last_power_space);
			} else if (detected_tone == 0) {
				snprintf(uart_msg, sizeof(uart_msg),
						"DET: SPACE Pm=%.0f Ps=%.0f\r\n", last_power_mark,
						last_power_space);
			} else {
				snprintf(uart_msg, sizeof(uart_msg),
						"DET: NOISE Pm=%.0f Ps=%.0f\r\n", last_power_mark,
						last_power_space);
			}
			HAL_UART_Transmit(&huart4, (uint8_t*) uart_msg, strlen(uart_msg),
			HAL_MAX_DELAY);
		}

		/* small sleep to reduce CPU usage — DAC and ADC are driven by DMA/TIM */
		HAL_Delay(1);
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

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
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
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.GainCompensation = 0;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.DMAContinuousRequests = ENABLE;
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
	sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
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
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

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
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
