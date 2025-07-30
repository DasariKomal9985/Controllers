#include "max30102.h"

void MAX30102_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    HAL_I2C_Master_Transmit(hi2c, MAX30102_ADDRESS, data, 2, HAL_MAX_DELAY);
}

void MAX30102_ReadFIFO(I2C_HandleTypeDef *hi2c, uint32_t *red_led, uint32_t *ir_led) {
    uint8_t reg = REG_FIFO_DATA;
    uint8_t data[6];
    HAL_I2C_Master_Transmit(hi2c, MAX30102_ADDRESS, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c, MAX30102_ADDRESS, data, 6, HAL_MAX_DELAY);

    *red_led = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    *red_led &= 0x3FFFF;

    *ir_led = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
    *ir_led &= 0x3FFFF;
}

void MAX30102_Init(I2C_HandleTypeDef *hi2c) {
    MAX30102_WriteReg(hi2c, REG_INTR_ENABLE_1, 0xC0);
    MAX30102_WriteReg(hi2c, REG_FIFO_WR_PTR, 0x00);
    MAX30102_WriteReg(hi2c, REG_OVF_COUNTER, 0x00);
    MAX30102_WriteReg(hi2c, REG_FIFO_RD_PTR, 0x00);
    MAX30102_WriteReg(hi2c, REG_MODE_CONFIG, 0x03);
    MAX30102_WriteReg(hi2c, REG_SPO2_CONFIG, 0x27);
    MAX30102_WriteReg(hi2c, REG_LED1_PA, 0x24);
    MAX30102_WriteReg(hi2c, REG_LED2_PA, 0x24);
}
