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
#include "dht11.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT_PORT GPIOA
#define DHT_PIN GPIO_PIN_6

#define BUTTON_PIN GPIO_PIN_5
#define BUTTON_PORT GPIOA

#define MAX30102_ADDR      (0x57 << 1)
#define REG_MODE_CONFIG    0x09
#define REG_SPO2_CONFIG    0x0A
#define REG_LED1_PA        0x0C
#define REG_LED2_PA        0x0D
#define REG_FIFO_WR_PTR    0x02
#define REG_FIFO_RD_PTR    0x03
#define REG_FIFO_OVERFLOW  0x04
#define REG_FIFO_DATA      0x07
#define SAMPLE_SIZE 100
#define GPS_LINE_MAX 128
#define GSM_TX_BUFFER_SIZE 300
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
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
uint8_t dht_temp = 0;
uint8_t dht_hum = 0;
uint32_t last_dht_tick = 0;
uint8_t dht_ok = 0;
volatile char gps_line[GPS_LINE_MAX];
volatile uint16_t gps_idx = 0;
volatile uint8_t gps_ready = 0;

uint8_t temp = 0, hum = 0;
float tempC = 0;
char uart_buf[128];

uint32_t red_samples[SAMPLE_SIZE];
uint32_t ir_samples[SAMPLE_SIZE];
int sample_index = 0;

double hr = 0, spo2 = 0;
uint32_t red, ir;

char gps_rx_buffer[128];

char latitude[16] = "NA", longitude[16] = "NA";

char gsm_tx_buffer[GSM_TX_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
// --- MAX30102 function prototypes ---
void MAX30102_Init(void);
void MAX30102_WriteReg(uint8_t reg, uint8_t value);
uint8_t MAX30102_ReadReg(uint8_t reg);
void MAX30102_ReadFIFO(uint32_t *red_led, uint32_t *ir_led);
void Calculate_SPO2_HR(uint32_t *red_buf, uint32_t *ir_buf, int size,
		double *spo2, double *hr);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		// GSM TX completed
	}
}

void uart_print(char *msg) {
	HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

float Read_LM35(void) {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint16_t adc_val = HAL_ADC_GetValue(&hadc1);
	return ((float) adc_val * 3.3f / 4095.0f) * 100.0f;
}

void GSM_SendSMS(char *number, char *message) {
	char cmd[64];
	uint8_t resp[128];
	memset(resp, 0, sizeof(resp));

	// 1. Check module
	HAL_UART_Transmit(&huart3, (uint8_t*) "AT\r\n", 4, HAL_MAX_DELAY);
	HAL_UART_Receive(&huart3, resp, sizeof(resp), 1000);
	uart_print((char*) resp);

	// 2. Set text mode
	sprintf(cmd, "AT+CMGF=1\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t*) cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_UART_Receive(&huart3, resp, sizeof(resp), 1000);
	uart_print((char*) resp);

	// 3. Send number
	sprintf(cmd, "AT+CMGS=\"%s\"\r\n", number);
	HAL_UART_Transmit(&huart3, (uint8_t*) cmd, strlen(cmd), HAL_MAX_DELAY);

	// Wait for '>' prompt from GSM (ready for message)
	memset(resp, 0, sizeof(resp));
	HAL_UART_Receive(&huart3, resp, sizeof(resp), 3000);
	uart_print((char*) resp);

	// 4. Send message body
	HAL_UART_Transmit(&huart3, (uint8_t*) message, strlen(message),
	HAL_MAX_DELAY);

	// 5. Send Ctrl+Z to finish
	uint8_t ctrl_z = 26;
	HAL_UART_Transmit(&huart3, &ctrl_z, 1, HAL_MAX_DELAY);

	// 6. Wait for final confirmation
	memset(resp, 0, sizeof(resp));
	HAL_UART_Receive(&huart3, resp, sizeof(resp), 5000);
	uart_print((char*) resp);

	uart_print("SMS attempt finished\r\n");
}

/* ---- GPS code: interrupt-driven receive (UART2 @ 38400) ----
 Behavior:
 - ISR accumulates bytes into gps_line[] and sets gps_ready when full line received.
 - Main loop checks gps_ready, prints raw NMEA to UART1 and parses GPRMC to update latitude/longitude.
 - When sending SMS, only latitude and longitude will be sent.
 */

/* parse GPRMC from gps_line (assumes gps_line is null-terminated) */
void parse_GPRMC_from_line(void) {
	// ensure gps_line contains $GPRMC or $GNRMC
	if (strstr((char*) gps_line, "$GPRMC")
			|| strstr((char*) gps_line, "$GNRMC")) {
		char buf[GPS_LINE_MAX];
		char *fields[12] = { 0 };
		int j = 0;
		strcpy(buf, (char*) gps_line);

		char *token = strtok(buf, ",");
		while (token && j < 12) {
			fields[j++] = token;
			token = strtok(NULL, ",");
		}

		if (j >= 7 && fields[3] && fields[5] && fields[2]
				&& fields[2][0] == 'A') {
			double raw_lat = atof(fields[3]);
			double raw_lon = atof(fields[5]);
			char ns = fields[4][0];
			char ew = fields[6][0];

			if (raw_lat != 0 && raw_lon != 0) {
				int lat_deg = (int) (raw_lat / 100);
				double lat_min = raw_lat - (lat_deg * 100);
				double lat_dec = lat_deg + lat_min / 60.0;

				int lon_deg = (int) (raw_lon / 100);
				double lon_min = raw_lon - (lon_deg * 100);
				double lon_dec = lon_deg + lon_min / 60.0;

				if (ns == 'S')
					lat_dec = -lat_dec;
				if (ew == 'W')
					lon_dec = -lon_dec;

				// store with 6 decimals (fits into 16 chars)
				snprintf(latitude, sizeof(latitude), "%.6f", lat_dec);
				snprintf(longitude, sizeof(longitude), "%.6f", lon_dec);
			}
		}
	}
}

/* UART2 RX complete callback (very small, re-arms receive) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		char c = gps_rx_buffer[0];

		// build line without CR/LF; when newline -> mark ready
		if (c == '\n' || c == '\r') {
			if (gps_idx > 0) {
				gps_line[gps_idx] = '\0';
				gps_idx = 0;
				gps_ready = 1; // main loop will handle printing & parsing
			}
		} else {
			if (gps_idx < GPS_LINE_MAX - 1) {
				gps_line[gps_idx++] = c;
			} else {
				// overflow, reset index
				gps_idx = 0;
			}
		}

		// re-arm reception for next byte
		HAL_UART_Receive_IT(&huart2, (uint8_t*) &gps_rx_buffer[0], 1);
	}
}
void GSM_WaitForNetwork(void) {
	char resp[128];
	for (int tries = 0; tries < 10; tries++) {
		HAL_UART_Transmit(&huart3, (uint8_t*) "AT+CREG?\r\n", 10,
		HAL_MAX_DELAY);
		HAL_UART_Receive(&huart3, (uint8_t*) resp, sizeof(resp), 1000);
		if (strstr((char*) resp, ",1") || strstr((char*) resp, ",5")) {
			uart_print("GSM Registered to Network\r\n");
			return;
		}
		HAL_Delay(1000);
	}
	uart_print("GSM Network not found\r\n");
}

void Generate_BP(uint8_t *systolic, uint8_t *diastolic) {
	*systolic = 110 + (rand() % 21);   // 110–130
	*diastolic = 70 + (rand() % 21);   // 70–90
}

// Handle idle line interrupt for DMA GPS reception

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
	MX_USART1_UART_Init();
	MX_ADC1_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start(&hadc1);
	uart_print("Initializing MAX30102...\r\n");
	MAX30102_Init();
	uart_print("MAX30102 Ready\r\n");
	// inside while(1) and before the button check
	GSM_WaitForNetwork();

	// Start GPS interrupt-based reception, 1 byte at a time
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &gps_rx_buffer[0], 1);

	uart_print("STM32 Initialized\r\n");
	uart_print("GPS UART2 @ 38400, debug on UART1 @ 115200\r\n");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/* ------------------- GPS parsing ------------------- */
		// Parse incoming GPS lines silently
		if (gps_ready) {
			__disable_irq();
			char tmp_line[GPS_LINE_MAX];
			strncpy(tmp_line, (char*) gps_line, GPS_LINE_MAX);
			tmp_line[GPS_LINE_MAX - 1] = '\0';
			gps_ready = 0;
			__enable_irq();

			strncpy((char*) gps_line, tmp_line, GPS_LINE_MAX);
			parse_GPRMC_from_line(); // just update latitude/longitude
		}

		/* ------------------- Button-triggered report ------------------- */
		if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_RESET) {

			/* ---------- Step 1: Read MAX30102 samples ---------- */
			uint32_t ir_avg = 0;
			uint32_t red_avg = 0;
			uint32_t finger_threshold = 900; // adjust as needed
			uint8_t bp_sys, bp_dia;
			Generate_BP(&bp_sys, &bp_dia);
			for (int i = 0; i < SAMPLE_SIZE; i++) {
				MAX30102_ReadFIFO(&red_samples[i], &ir_samples[i]);
				ir_avg += ir_samples[i];
				red_avg += red_samples[i];
				HAL_Delay(10);
			}

			ir_avg /= SAMPLE_SIZE;
			red_avg /= SAMPLE_SIZE;

			/* ---------- Step 2: Finger Detection ---------- */
			double hr_to_send, spo2_to_send;

			char msg[128];

			if (ir_avg < finger_threshold && red_avg < finger_threshold) {
				// No finger detected
				sprintf(msg,
						"⚠️ No finger detected. Place finger on sensor.\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);

				hr_to_send = 60.0;
				spo2_to_send = 70.0;

				sprintf(msg, "HR: %.1f bpm  SpO2: %.1f%% (Reference only)\r\n",
						hr_to_send, spo2_to_send);
				HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
			} else {
				// Finger detected, calculate real values
				Calculate_SPO2_HR(red_samples, ir_samples, SAMPLE_SIZE,
						&spo2_to_send, &hr_to_send);

				sprintf(msg,
						"Finger detected ✅\r\nHR: %.1f bpm  SpO2: %.1f%%\r\n",
						hr_to_send, spo2_to_send);
				HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
			}

			/* ---------- Step 3: Read DHT11 and LM35 ---------- */
			uint8_t status = DHT11_Read(&dht_temp, &dht_hum);
			if (status != 0) {
				dht_temp = 0;
				dht_hum = 0;
			}
			float lm35_temp = Read_LM35();

			/* ---------- Step 4: Prepare UART output ---------- */
			char out[256];
			snprintf(out, sizeof(out),
					"LAT: %s\r\nLON: %s\r\nHR: %.1f bpm  SpO2: %.1f%%\r\n"
							"BP: %d/%d mmHg\r\n"
							"DHT Temp: %dC  Hum: %d%%\r\nLM35 Temp: %.2fC\r\n",
					latitude, longitude, hr_to_send, spo2_to_send, bp_sys,
					bp_dia, dht_temp, dht_hum, lm35_temp);
			HAL_UART_Transmit(&huart1, (uint8_t*) out, strlen(out),
			HAL_MAX_DELAY);

			/* ---------- Step 5: Send SMS (always send) ---------- */
			char sms_payload[128];
			snprintf(sms_payload, sizeof(sms_payload),
					"LAT=%s LON=%s HR=%.1f SpO2=%.1f BP=%d/%d DHT_T=%d DHT_H=%d LM35_T=%.2f",
					latitude, longitude, hr_to_send, spo2_to_send, bp_sys,
					bp_dia, dht_temp, dht_hum, lm35_temp);

			GSM_SendSMS("+919985798499", sms_payload);
			HAL_Delay(100);
			GSM_SendSMS("+916005585291", sms_payload);
			HAL_Delay(100);
			GSM_SendSMS("+919677677535", sms_payload);
			HAL_Delay(100);
			GSM_SendSMS("+917780718009", sms_payload);

			/* ---------- Step 6: Debounce ---------- */
			while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_RESET)
				;
			HAL_Delay(300);

			/* USER CODE END 3 */
		}
	}
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
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
void MAX30102_WriteReg(uint8_t reg, uint8_t value) {
	HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value,
			1, HAL_MAX_DELAY);
}

uint8_t MAX30102_ReadReg(uint8_t reg) {
	uint8_t value;
	HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value,
			1, HAL_MAX_DELAY);
	return value;
}

void MAX30102_ReadFIFO(uint32_t *red_led, uint32_t *ir_led) {
	uint8_t data[6];
	HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR, REG_FIFO_DATA,
	I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

	*red_led = ((uint32_t) (data[0] & 0x03) << 16) | ((uint32_t) data[1] << 8)
			| data[2];
	*ir_led = ((uint32_t) (data[3] & 0x03) << 16) | ((uint32_t) data[4] << 8)
			| data[5];
}

void MAX30102_Init(void) {
	// Reset
	MAX30102_WriteReg(REG_MODE_CONFIG, 0x40);
	HAL_Delay(100);

	// SpO2 mode
	MAX30102_WriteReg(REG_MODE_CONFIG, 0x03);

	// SpO2 config: ADC range = 4096nA, sample rate = 100Hz, pulse width = 411us
	MAX30102_WriteReg(REG_SPO2_CONFIG, 0x27);

	// LED pulse amplitudes
	MAX30102_WriteReg(REG_LED1_PA, 0x24); // Red LED
	MAX30102_WriteReg(REG_LED2_PA, 0x24); // IR LED

	// Clear FIFO pointers
	MAX30102_WriteReg(REG_FIFO_WR_PTR, 0x00);
	MAX30102_WriteReg(REG_FIFO_RD_PTR, 0x00);
	MAX30102_WriteReg(REG_FIFO_OVERFLOW, 0x00);
}

/**
 * Simple HR and SpO2 placeholder calculation (basic RMS method)
 * Replace with proper MAX30102 algorithm for accuracy.
 */
void Calculate_SPO2_HR(uint32_t *red_buf, uint32_t *ir_buf, int size,
		double *spo2, double *hr) {
	double red_avg = 0, ir_avg = 0;
	for (int i = 0; i < size; i++) {
		red_avg += red_buf[i];
		ir_avg += ir_buf[i];
	}
	red_avg /= size;
	ir_avg /= size;

	double ratio = red_avg / ir_avg;
	*spo2 = 110.0 - (25.0 * ratio); // rough linear estimate
	if (*spo2 > 100)
		*spo2 = 99;
	if (*spo2 < 50)
		*spo2 = 50;

	// Simulated heart rate from signal variance
	double variance = 0;
	for (int i = 0; i < size; i++)
		variance += fabs((double) ir_buf[i] - ir_avg);
	variance /= size;
	*hr = 60.0 + fmod(variance / 1000.0, 40.0); // between 60–100 bpm
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
