#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT   64
#define SSD1306_ADDR    (0x3C << 1)

extern I2C_HandleTypeDef hi2c1;

// ---------------- BASIC API ----------------
void SSD1306_Init(void);
void SSD1306_Update(void);
void SSD1306_Clear(void);
void SSD1306_SetCursor(uint8_t x, uint8_t y);
void SSD1306_WriteChar(char c);
void SSD1306_WriteString(const char *str);
void SSD1306_WriteStringXY(uint8_t x, uint8_t y, const char *s);

// ----------- PIXEL / DRAWING API -----------
void SSD1306_DrawPixel(uint8_t x, uint8_t y, bool color);
void SSD1306_ClearArea(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void SSD1306_FillCircle(int xc, int yc, int r);

// ----------------- ROBOT EYES --------------
void SSD1306_DrawEye(int xc, int yc, int radius,
                     int pupil_offset_x, int pupil_offset_y,
                     bool closed);

void SSD1306_ShowEyesExpression(uint8_t expr);
void SSD1306_ShowEyesAnimation(void);

#endif
