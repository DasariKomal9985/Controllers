#ifndef SSD1306_H
#define SSD1306_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_ADDR (0x3C << 1)

extern I2C_HandleTypeDef hi2c1;

void SSD1306_Init(void);
void SSD1306_Update(void);
void SSD1306_Clear(void);
void SSD1306_SetCursor(uint8_t x, uint8_t y);
void SSD1306_WriteChar(char c);
void SSD1306_WriteString(const char *str);
void SSD1306_WriteStringXY(uint8_t x, uint8_t y, const char *s);
void SSD1306_DrawPixel(uint8_t x, uint8_t y);
void SSD1306_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);


#endif
