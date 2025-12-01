/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (nRF24 + SPI + UART test)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char uart_buf[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

/* --------------------------------------------------------------------------- */
/* === nRF24 pin/command definitions (use your CubeMX labels) ===             */
/* NOTE: CubeMX generates macros like NRF_CSN_SPI1_Pin and NRF_CSN_SPI1_GPIO_Port */
#define NRF_CE_PIN_LABEL     NRF_CE_SPI1_Pin
#define NRF_CE_PORT_LABEL    NRF_CE_SPI1_GPIO_Port

#define NRF_CSN_PIN_LABEL    NRF_CSN_SPI1_Pin
#define NRF_CSN_PORT_LABEL   NRF_CSN_SPI1_GPIO_Port

#define NRF_SCK_PIN_LABEL    NRF_SCK_SPI1_Pin
#define NRF_SCK_PORT_LABEL   NRF_SCK_SPI1_GPIO_Port

#define NRF_MOSI_PIN_LABEL   NRF_MOSI_SPI1_Pin
#define NRF_MOSI_PORT_LABEL  NRF_MOSI_SPI1_GPIO_Port

#define NRF_MISO_PIN_LABEL   NRF_MISO_SPI1_Pin
#define NRF_MISO_PORT_LABEL  NRF_MISO_SPI1_GPIO_Port

#define UART_TX_PIN_LABEL    TTL_TX_UART2_Pin
#define UART_TX_PORT_LABEL   TTL_TX_UART2_GPIO_Port

#define UART_RX_PIN_LABEL    TTL_RX_UART2_Pin
#define UART_RX_PORT_LABEL   TTL_RX_UART2_GPIO_Port


/* nRF commands & registers */
#define NRF_CMD_R_REGISTER  0x00
#define NRF_CMD_W_REGISTER  0x20

#define NRF_REG_CONFIG      0x00
#define NRF_REG_RF_CH       0x05
#define NRF_REG_RF_SETUP    0x06
#define NRF_REG_STATUS      0x07

/* Helper macros to control CE/CSN */
#define CE_HIGH()  HAL_GPIO_WritePin(NRF_CE_PORT_LABEL, NRF_CE_PIN_LABEL, GPIO_PIN_SET)
#define CE_LOW()   HAL_GPIO_WritePin(NRF_CE_PORT_LABEL, NRF_CE_PIN_LABEL, GPIO_PIN_RESET)
#define CSN_HIGH() HAL_GPIO_WritePin(NRF_CSN_PORT_LABEL, NRF_CSN_PIN_LABEL, GPIO_PIN_SET)
#define CSN_LOW()  HAL_GPIO_WritePin(NRF_CSN_PORT_LABEL, NRF_CSN_PIN_LABEL, GPIO_PIN_RESET)

/* UART helper */
static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 300);
}

/* nRF helper prototypes */
uint8_t nrf24_read_reg(uint8_t reg);
void    nrf24_write_reg(uint8_t reg, uint8_t value);

/* --------------------------------------------------------------------------- */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* Ensure correct idle states after GPIO init */
  CSN_HIGH();  /* CSN must be HIGH when idle */
  CE_LOW();    /* CE idle LOW */

  HAL_Delay(50);

  uart_print("\r\n=== nRF24 SELF TEST (16 MHz SYSCLK) ===\r\n");

  /* Write CONFIG = PWR_UP + CRC (0x0A) */
  nrf24_write_reg(NRF_REG_CONFIG, 0x0A);
  HAL_Delay(5);

  uint8_t cfg  = nrf24_read_reg(NRF_REG_CONFIG);
  uint8_t rfch = nrf24_read_reg(NRF_REG_RF_CH);
  uint8_t rfst = nrf24_read_reg(NRF_REG_RF_SETUP);
  uint8_t stat = nrf24_read_reg(NRF_REG_STATUS);

  snprintf(uart_buf, sizeof(uart_buf), "CONFIG   = 0x%02X\r\n", cfg);  uart_print(uart_buf);
  snprintf(uart_buf, sizeof(uart_buf), "RF_CH    = 0x%02X\r\n", rfch); uart_print(uart_buf);
  snprintf(uart_buf, sizeof(uart_buf), "RF_SETUP = 0x%02X\r\n", rfst); uart_print(uart_buf);
  snprintf(uart_buf, sizeof(uart_buf), "STATUS   = 0x%02X\r\n", stat); uart_print(uart_buf);

  if (cfg == 0x0A) uart_print("nRF24 DETECTED OK\r\n");
  else             uart_print("nRF24 NOT RESPONDING\r\n");

  while (1)
  {
    /* idle loop - you can extend with TX/RX code later */
    HAL_Delay(1000);
  }
}

/* ----------------- nRF register read/write implementations ----------------- */
uint8_t nrf24_read_reg(uint8_t reg)
{
    uint8_t cmd = NRF_CMD_R_REGISTER | (reg & 0x1F);
    uint8_t val = 0xFF;

    CSN_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 200);
    HAL_SPI_Receive(&hspi1, &val, 1, 200);
    CSN_HIGH();

    return val;
}

void nrf24_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2];
    buf[0] = NRF_CMD_W_REGISTER | (reg & 0x1F);
    buf[1] = value;

    CSN_LOW();
    HAL_SPI_Transmit(&hspi1, buf, 2, 200);
    CSN_HIGH();
}

/* ----------------- System Clock (16 MHz) ---------------------------------- */
/* Your provided PLL config: HSI/2 * 4 = 16 MHz */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  /* use PLL: HSI/2 as input, PLLMUL = 4 -> (8/2)*4 = 16MHz */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;   /* HCLK = 16 MHz */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;    /* PCLK1 = 16 MHz (<=36MHz OK) */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    /* PCLK2 = 16 MHz */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }
}

/* ----------------- SPI1 Initialization ------------------------------------ */
static void MX_SPI1_Init(void)
{
  /* SPI1 uses APB2 clock (which is SYSCLK = 16 MHz) */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;   /* MODE0 */
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  /* choose prescaler so SCK <= ~1 MHz: 16 / 16 = 1 MHz */
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
}

/* ----------------- UART2 Initialization ----------------------------------- */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

/* ----------------- GPIO Initialization ------------------------------------ */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable GPIO clocks for ports used by labels (CubeMX generated defines port macros) */
  /* The generated macros (e.g. NRF_CSN_SPI1_GPIO_Port) must be present in main.h */
  /* Enable all ports that might be referenced */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* --- Configure SPI pins (use your CubeMX labels) --- */
  /* SCK & MOSI -> AF Push-Pull */
  GPIO_InitStruct.Pin = NRF_SCK_PIN_LABEL | NRF_MOSI_PIN_LABEL;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NRF_SCK_PORT_LABEL, &GPIO_InitStruct); /* NRF_SCK_PORT_LABEL and NRF_MOSI share same port (PA) */

  /* MISO -> Input floating */
  GPIO_InitStruct.Pin = NRF_MISO_PIN_LABEL;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF_MISO_PORT_LABEL, &GPIO_InitStruct);

  /* --- Configure UART pins (TX = AF_PP, RX = input) --- */
  GPIO_InitStruct.Pin = UART_TX_PIN_LABEL;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(UART_TX_PORT_LABEL, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = UART_RX_PIN_LABEL;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UART_RX_PORT_LABEL, &GPIO_InitStruct);

  /* --- Configure CE and CSN pins as outputs --- */
  GPIO_InitStruct.Pin = NRF_CSN_PIN_LABEL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NRF_CSN_PORT_LABEL, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = NRF_CE_PIN_LABEL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NRF_CE_PORT_LABEL, &GPIO_InitStruct);

  /* Set correct idle levels */
  HAL_GPIO_WritePin(NRF_CSN_PORT_LABEL, NRF_CSN_PIN_LABEL, GPIO_PIN_SET); /* CSN = HIGH (idle) */
  HAL_GPIO_WritePin(NRF_CE_PORT_LABEL, NRF_CE_PIN_LABEL, GPIO_PIN_RESET); /* CE = LOW (idle) */
}

/* ----------------- Error Handler ----------------------------------------- */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
