#include "dht11.h"
#include "stm32f1xx_hal.h"

#define DHT_PORT GPIOA
#define DHT_PIN GPIO_PIN_6

extern TIM_HandleTypeDef htim2;

void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

void DHT_Set_Pin_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);
}

void DHT_Set_Pin_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);
}

uint8_t DHT11_Read(uint8_t *temp, uint8_t *hum)
{
    uint8_t bits[5] = {0};
    uint8_t i, j;

    DHT_Set_Pin_Output();
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET);
    HAL_Delay(20); // 18+ ms
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);
    delay_us(30);
    DHT_Set_Pin_Input();

    // Wait for response
    uint32_t timeout = 0;
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET)
        if (++timeout > 10000) return 1;
    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_RESET)
        if (++timeout > 10000) return 1;
    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET)
        if (++timeout > 10000) return 1;

    // Read 5 bytes
    for (j = 0; j < 5; j++)
    {
        for (i = 0; i < 8; i++)
        {
            while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_RESET);
            delay_us(40);
            if (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET)
                bits[j] |= (1 << (7 - i));
            while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET);
        }
    }

    if ((uint8_t)(bits[0] + bits[1] + bits[2] + bits[3]) != bits[4])
        return 2;

    *hum = bits[0];
    *temp = bits[2];

    return 0;
}
