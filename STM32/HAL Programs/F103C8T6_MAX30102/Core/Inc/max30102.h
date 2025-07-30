#ifndef __MAX30102_H__
#define __MAX30102_H__

#include "stm32f1xx_hal.h"

// MAX30102 I2C address
#define MAX30102_ADDRESS        0x57 << 1  // Shifted for HAL (8-bit address)

// Register map
#define REG_INTR_STATUS_1       0x00
#define REG_INTR_ENABLE_1       0x02
#define REG_FIFO_WR_PTR         0x04
#define REG_OVF_COUNTER         0x05
#define REG_FIFO_RD_PTR         0x06
#define REG_FIFO_DATA           0x07
#define REG_MODE_CONFIG         0x09
#define REG_SPO2_CONFIG         0x0A
#define REG_LED1_PA             0x0C
#define REG_LED2_PA             0x0D

// Function declarations
void MAX30102_Init(I2C_HandleTypeDef *hi2c);
void MAX30102_ReadFIFO(I2C_HandleTypeDef *hi2c, uint32_t *red_led, uint32_t *ir_led);

#endif
