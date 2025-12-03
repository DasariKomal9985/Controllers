/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body (corrected)
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t txByte;
uint8_t rxByte;
/* single UART buffer used for prints */
char uart_buf[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* === nRF Pins (your wiring) === */
#define NRF_CE_PIN      GPIO_PIN_13    // PB13
#define NRF_CE_PORT     GPIOB

#define NRF_CSN_PIN     GPIO_PIN_12    // PB12
#define NRF_CSN_PORT    GPIOB

#define CE_HIGH()   HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_SET)
#define CE_LOW()    HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_RESET)

#define CSN_HIGH()  HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET)
#define CSN_LOW()   HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_RESET)

/* nRF commands / regs */
#define NRF_CMD_R_REGISTER  0x00
#define NRF_CMD_W_REGISTER  0x20

#define NRF_REG_CONFIG      0x00
#define NRF_REG_RF_CH       0x05
#define NRF_REG_RF_SETUP    0x06
#define NRF_REG_STATUS      0x07

/* helper */
void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 300);
}

/* prototypes for nRF helpers */
uint8_t nrf24_read_reg(uint8_t reg);
void nrf24_write_reg(uint8_t reg, uint8_t value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t nrf24_read_reg(uint8_t reg)
{
    uint8_t tx = NRF_CMD_R_REGISTER | (reg & 0x1F);
    uint8_t rx = 0xFF;

    CSN_LOW();
    /* send command byte */
    HAL_SPI_Transmit(&hspi1, &tx, 1, 100);
    /* read reply byte */
    HAL_SPI_Receive(&hspi1, &rx, 1, 100);
    CSN_HIGH();

    return rx;
}

void nrf24_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2];
    buf[0] = NRF_CMD_W_REGISTER | (reg & 0x1F);
    buf[1] = value;

    CSN_LOW();
    HAL_SPI_Transmit(&hspi1, buf, 2, 100);
    CSN_HIGH();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU init */
    HAL_Init();
    SystemClock_Config();   /* 36 MHz as you requested */

    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();

    /* Ensure proper CSN/CE idle levels (after GPIO init) */
    CSN_HIGH();   /* CSN idle HIGH */
    CE_LOW();     /* CE idle LOW */

    HAL_Delay(50);

    uart_print("\r\n=== nRF24 SELF TEST (36MHz Clock) ===\r\n");

    /* Write test value to CONFIG (PWR_UP + CRC enable) */
    nrf24_write_reg(NRF_REG_CONFIG, 0x0A);
    HAL_Delay(5);

    uint8_t cfg  = nrf24_read_reg(NRF_REG_CONFIG);
    uint8_t rfch = nrf24_read_reg(NRF_REG_RF_CH);
    uint8_t rfst = nrf24_read_reg(NRF_REG_RF_SETUP);
    uint8_t stat = nrf24_read_reg(NRF_REG_STATUS);

    snprintf(uart_buf, sizeof(uart_buf), "CONFIG   = 0x%02X\r\n", cfg);
    uart_print(uart_buf);
    snprintf(uart_buf, sizeof(uart_buf), "RF_CH    = 0x%02X\r\n", rfch);
    uart_print(uart_buf);
    snprintf(uart_buf, sizeof(uart_buf), "RF_SETUP = 0x%02X\r\n", rfst);
    uart_print(uart_buf);
    snprintf(uart_buf, sizeof(uart_buf), "STATUS   = 0x%02X\r\n", stat);
    uart_print(uart_buf);

    if (cfg == 0x0A)
        uart_print("nRF24 DETECTED OK\r\n");
    else
        uart_print("nRF24 NOT RESPONDING\r\n");

    /* loop: blink LED, slow if not detected */
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay((cfg == 0x0A) ? 200 : 1000);
    }
}

/* System Clock Configuration
   HSI (8 MHz) / 2 * 9 = 36 MHz
*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    /* HSI/2 as PLL input */
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; /* (8/2)*9 = 36 MHz */

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                                | RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; /* 36MHz */
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  /* 36MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;   /* 36MHz (APB1 max 36MHz OK) */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   /* 36MHz */

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* SPI1 init - SPI1 uses APB2 clock (36 MHz) */
static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE; /* SPI Mode 0 */
    hspi1.Init.NSS = SPI_NSS_SOFT;
    /* choose prescaler so SCK <= ~1MHz; 36/64 = 562.5kHz (safe with long wires) */
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

/* UART2 init (PA2 TX, PA3 RX) 9600 */
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

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* GPIO init - configure SPI pins, UART pins, CE/CSN, LED */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Clocks */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* -- SPI1 pins (PA5=SCK, PA6=MISO, PA7=MOSI) -- */
    /* SCK and MOSI need AF push-pull; MISO input floating */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7; /* SCK, MOSI */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6; /* MISO */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* -- UART2 pins (PA2=TX, PA3=RX) -- */
    GPIO_InitStruct.Pin = GPIO_PIN_2; /* TX */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3; /* RX */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* -- CE and CSN on PB13/PB12 -- */
    GPIO_InitStruct.Pin = NRF_CE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NRF_CE_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = NRF_CSN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NRF_CSN_PORT, &GPIO_InitStruct);

    /* -- LED on PC13 (push-pull) -- */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Set default levels: CSN = HIGH (idle), CE = LOW (idle), LED off */
    CSN_HIGH();
    CE_LOW();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); /* LED off on many boards (PC13 high is OFF) */
}

/* Error handler left as-is */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
