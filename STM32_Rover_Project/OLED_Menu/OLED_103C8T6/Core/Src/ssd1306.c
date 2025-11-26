/* ssd1306.c - merged, cleaned, and ready-to-build for STM32F103
 *
 * Includes:
 *  - Basic SSD1306 I2C driver (page write)
 *  - 5x7 font text functions
 *  - Pixel drawing, filled circle, outline
 *  - Rounded cartoon robot eyes (expressions + animation)
 *
 * Ensure Core/Inc/ssd1306.h matches the header provided earlier.
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "stm32f1xx_hal.h"
#include "ssd1306.h"

// display buffer & cursor state
static uint8_t buffer[1024];
static uint8_t posX = 0, posY = 0;

// 5x7 default font
static const uint8_t font5x7[][5] = {
{0x00,0x00,0x00,0x00,0x00}, //
{0x00,0x00,0x5F,0x00,0x00}, // !
{0x00,0x07,0x00,0x07,0x00}, // "
{0x14,0x7F,0x14,0x7F,0x14}, // #
{0x24,0x2A,0x7F,0x2A,0x12}, // $
{0x23,0x13,0x08,0x64,0x62}, // %
{0x36,0x49,0x55,0x22,0x50}, // &
{0x00,0x05,0x03,0x00,0x00}, // '
{0x00,0x1C,0x22,0x41,0x00}, // (
{0x00,0x41,0x22,0x1C,0x00}, // )
{0x14,0x08,0x3E,0x08,0x14}, // *
{0x08,0x08,0x3E,0x08,0x08}, // +
{0x00,0x50,0x30,0x00,0x00}, // ,
{0x08,0x08,0x08,0x08,0x08}, // -
{0x00,0x60,0x60,0x00,0x00}, // .
{0x20,0x10,0x08,0x04,0x02}, // /
{0x3E,0x51,0x49,0x45,0x3E}, // 0
{0x00,0x42,0x7F,0x40,0x00}, // 1
{0x42,0x61,0x51,0x49,0x46}, // 2
{0x21,0x41,0x45,0x4B,0x31}, // 3
{0x18,0x14,0x12,0x7F,0x10}, // 4
{0x27,0x45,0x45,0x45,0x39}, // 5
{0x3C,0x4A,0x49,0x49,0x30}, // 6
{0x01,0x71,0x09,0x05,0x03}, // 7
{0x36,0x49,0x49,0x49,0x36}, // 8
{0x06,0x49,0x49,0x29,0x1E}, // 9
{0x00,0x36,0x36,0x00,0x00}, // :
{0x00,0x56,0x36,0x00,0x00}, // ;
{0x08,0x14,0x22,0x41,0x00}, // <
{0x14,0x14,0x14,0x14,0x14}, // =
{0x00,0x41,0x22,0x14,0x08}, // >
{0x02,0x01,0x51,0x09,0x06}, // ?
{0x32,0x49,0x79,0x41,0x3E}, // @
{0x7E,0x11,0x11,0x11,0x7E}, // A
{0x7F,0x49,0x49,0x49,0x36}, // B
{0x3E,0x41,0x41,0x41,0x22}, // C
{0x7F,0x41,0x41,0x22,0x1C}, // D
{0x7F,0x49,0x49,0x49,0x41}, // E
{0x7F,0x09,0x09,0x09,0x01}, // F
{0x3E,0x41,0x49,0x49,0x7A}, // G
{0x7F,0x08,0x08,0x08,0x7F}, // H
{0x00,0x41,0x7F,0x41,0x00}, // I
{0x20,0x40,0x41,0x3F,0x01}, // J
{0x7F,0x08,0x14,0x22,0x41}, // K
{0x7F,0x40,0x40,0x40,0x40}, // L
{0x7F,0x02,0x04,0x02,0x7F}, // M
{0x7F,0x04,0x08,0x10,0x7F}, // N
{0x3E,0x41,0x41,0x41,0x3E}, // O
{0x7F,0x09,0x09,0x09,0x06}, // P
{0x3E,0x41,0x51,0x21,0x5E}, // Q
{0x7F,0x09,0x19,0x29,0x46}, // R
{0x46,0x49,0x49,0x49,0x31}, // S
{0x01,0x01,0x7F,0x01,0x01}, // T
{0x3F,0x40,0x40,0x40,0x3F}, // U
{0x1F,0x20,0x40,0x20,0x1F}, // V
{0x3F,0x40,0x38,0x40,0x3F}, // W
{0x63,0x14,0x08,0x14,0x63}, // X
{0x07,0x08,0x70,0x08,0x07}, // Y
{0x61,0x51,0x49,0x45,0x43}, // Z
{0x00,0x7F,0x41,0x41,0x00}, // [
{0x02,0x04,0x08,0x10,0x20}, // backslash
{0x00,0x41,0x41,0x7F,0x00}, // ]
{0x04,0x02,0x01,0x02,0x04}, // ^
{0x40,0x40,0x40,0x40,0x40}, // _
{0x00,0x01,0x02,0x04,0x00}, // `
{0x20,0x54,0x54,0x54,0x78}, // a
{0x7F,0x48,0x44,0x44,0x38}, // b
{0x38,0x44,0x44,0x44,0x20}, // c
{0x38,0x44,0x44,0x48,0x7F}, // d
{0x38,0x54,0x54,0x54,0x18}, // e
{0x08,0x7E,0x09,0x01,0x02}, // f
{0x0C,0x52,0x52,0x52,0x3E}, // g
{0x7F,0x08,0x04,0x04,0x78}, // h
{0x00,0x44,0x7D,0x40,0x00}, // i
{0x20,0x40,0x44,0x3D,0x00}, // j
{0x7F,0x10,0x28,0x44,0x00}, // k
{0x00,0x41,0x7F,0x40,0x00}, // l
{0x7C,0x04,0x18,0x04,0x7C}, // m
{0x7C,0x08,0x04,0x04,0x78}, // n
{0x38,0x44,0x44,0x44,0x38}, // o
{0x7C,0x14,0x14,0x14,0x08}, // p
{0x08,0x14,0x14,0x18,0x7C}, // q
{0x7C,0x08,0x04,0x04,0x08}, // r
{0x48,0x54,0x54,0x54,0x20}, // s
{0x04,0x3F,0x44,0x40,0x20}, // t
{0x3C,0x40,0x40,0x20,0x7C}, // u
{0x1C,0x20,0x40,0x20,0x1C}, // v
{0x3C,0x40,0x30,0x40,0x3C}, // w
{0x44,0x28,0x10,0x28,0x44}, // x
{0x0C,0x50,0x50,0x50,0x3C}, // y
{0x44,0x64,0x54,0x4C,0x44}, // z
{0x00,0x08,0x36,0x41,0x00}, // {
{0x00,0x00,0x7F,0x00,0x00}, // |
{0x00,0x41,0x36,0x08,0x00}, // }
{0x08,0x04,0x08,0x10,0x08}, // ~
};

// low-level command sender
static void cmd(uint8_t c) {
    uint8_t d[2] = {0x00, c};
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDR, d, 2, 50);
}

void SSD1306_Init(void)
{
    HAL_Delay(100);

    cmd(0xAE);
    cmd(0x20); cmd(0x00); // HORIZONTAL (Adafruit uses this)
    cmd(0x40);
    cmd(0xA1);
    cmd(0xC8);
    cmd(0x81); cmd(0x7F);
    cmd(0xA6);
    cmd(0xA8); cmd(0x3F);
    cmd(0xD3); cmd(0x00);
    cmd(0xD5); cmd(0x80);
    cmd(0xD9); cmd(0xF1);
    cmd(0xDA); cmd(0x12);
    cmd(0xDB); cmd(0x40);
    cmd(0x8D); cmd(0x14);
    cmd(0xA4);
    cmd(0xAF);

    SSD1306_Clear();
    SSD1306_Update();
}

void SSD1306_Update(void)
{
    for (uint8_t page = 0; page < 8; page++)
    {
        cmd(0xB0 + page);
        cmd(0x00);
        cmd(0x10);

        uint8_t data[129];
        data[0] = 0x40;
        memcpy(&data[1], &buffer[page * 128], 128);
        HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDR, data, sizeof(data), 100);
    }
}

void SSD1306_Clear(void) {
    memset(buffer, 0, sizeof(buffer));
}

void SSD1306_SetCursor(uint8_t x, uint8_t y) {
    posX = x;
    posY = y;
}

void SSD1306_WriteChar(char c)
{
    if (c < 32 || c > 126) c = ' ';

    uint16_t index = posX + (posY/8)*128;

    for (uint8_t i=0;i<5;i++)
        buffer[index+i] = font5x7[c-32][i];

    buffer[index+5] = 0x00;

    posX += 6;
}

void SSD1306_WriteString(const char *s)
{
    while(*s) SSD1306_WriteChar(*s++);
}
void SSD1306_WriteStringXY(uint8_t x, uint8_t y, const char *s)
{
    SSD1306_SetCursor(x, y);
    while (*s) SSD1306_WriteChar(*s++);
}

// --- drawing & eyes API

void SSD1306_DrawPixel(uint8_t x, uint8_t y, bool color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    uint16_t index = x + (y / 8) * SSD1306_WIDTH;
    if (color)
        buffer[index] |= (1 << (y & 7));
    else
        buffer[index] &= ~(1 << (y & 7));
}

void SSD1306_ClearArea(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    if (x + w > SSD1306_WIDTH) w = SSD1306_WIDTH - x;
    if (y + h > SSD1306_HEIGHT) h = SSD1306_HEIGHT - y;

    for (uint8_t yy = y; yy < y + h; yy++) {
        for (uint8_t xx = x; xx < x + w; xx++) {
            SSD1306_DrawPixel(xx, yy, 0);
        }
    }
}

// filled circle (simple integer rasterization)
void SSD1306_FillCircle(int xc, int yc, int r)
{
    if (r <= 0) return;
    for (int dy = -r; dy <= r; dy++) {
        int dx_limit = (int)sqrt((double)(r * r - dy * dy));
        for (int dx = -dx_limit; dx <= dx_limit; dx++) {
            int x = xc + dx;
            int y = yc + dy;
            if (x >= 0 && x < SSD1306_WIDTH && y >= 0 && y < SSD1306_HEIGHT)
                SSD1306_DrawPixel((uint8_t)x, (uint8_t)y, 1);
        }
    }
}

// thin hollow circle (outline)
static void SSD1306_DrawCircleOutline(int xc, int yc, int r)
{
    if (r <= 0) return;
    int x = r;
    int y = 0;
    int err = 0;

    while (x >= y) {
        // draw symmetric points (small thickness)
        for (int t = -1; t <= 1; t++) {
            if (xc + x >=0 && xc + x < SSD1306_WIDTH && yc + y + t >=0 && yc + y + t < SSD1306_HEIGHT)
                SSD1306_DrawPixel(xc + x, yc + y + t, 1);
            if (xc + y >=0 && xc + y < SSD1306_WIDTH && yc + x + t >=0 && yc + x + t < SSD1306_HEIGHT)
                SSD1306_DrawPixel(xc + y, yc + x + t, 1);
            if (xc - y >=0 && xc - y < SSD1306_WIDTH && yc + x + t >=0 && yc + x + t < SSD1306_HEIGHT)
                SSD1306_DrawPixel(xc - y, yc + x + t, 1);
            if (xc - x >=0 && xc - x < SSD1306_WIDTH && yc + y + t >=0 && yc + y + t < SSD1306_HEIGHT)
                SSD1306_DrawPixel(xc - x, yc + y + t, 1);
            if (xc - x >=0 && xc - x < SSD1306_WIDTH && yc - y + t >=0 && yc - y + t < SSD1306_HEIGHT)
                SSD1306_DrawPixel(xc - x, yc - y + t, 1);
            if (xc - y >=0 && xc - y < SSD1306_WIDTH && yc - x + t >=0 && yc - x + t < SSD1306_HEIGHT)
                SSD1306_DrawPixel(xc - y, yc - x + t, 1);
            if (xc + y >=0 && xc + y < SSD1306_WIDTH && yc - x + t >=0 && yc - x + t < SSD1306_HEIGHT)
                SSD1306_DrawPixel(xc + y, yc - x + t, 1);
            if (xc + x >=0 && xc + x < SSD1306_WIDTH && yc - y + t >=0 && yc - y + t < SSD1306_HEIGHT)
                SSD1306_DrawPixel(xc + x, yc - y + t, 1);
        }

        y += 1;
        err += 1 + 2*y;
        if (2*(err - x) + 1 > 0) {
            x -= 1;
            err += 1 - 2*x;
        }
    }
}

// Draw one eye at center (xc,yc), "radius" for outer size
// pupil_offset_x, pupil_offset_y in pixels; closed=true draws a blink
void SSD1306_DrawEye(int xc, int yc, int radius, int pupil_offset_x, int pupil_offset_y, bool closed)
{
    // clear eye bounding box a bit larger than radius
    int bb_x = xc - radius - 2;
    int bb_y = yc - radius - 2;
    int bb_w = 2 * (radius + 2);
    int bb_h = 2 * (radius + 2);
    SSD1306_ClearArea((bb_x>0)?bb_x:0, (bb_y>0)?bb_y:0, bb_w, bb_h);

    if (closed) {
        // closed eye: draw a small horizontal line
        for (int dx = -radius; dx <= radius; dx++) SSD1306_DrawPixel(xc + dx, yc, 1);
        // slight hint of eyelid (thin)
        for (int dx = -radius/2; dx <= radius/2; dx++) SSD1306_DrawPixel(xc + dx, yc - 2, 1);
        return;
    }

    // outer white (filled) for the eyeball
    SSD1306_FillCircle(xc, yc, radius);

    // cut out inner to make it look like white eyeball with pupil (we'll "erase" with 0)
    // erase an inner circle to create a ring effect (gives a glossy look)
    int inner = radius - 3;
    if (inner > 1) {
        for (int dy = -inner; dy <= inner; dy++) {
            int dx_limit = (int)sqrt((double)(inner * inner - dy * dy));
            for (int dx = -dx_limit; dx <= dx_limit; dx++) {
                int x = xc + dx;
                int y = yc + dy;
                if (x >= 0 && x < SSD1306_WIDTH && y >= 0 && y < SSD1306_HEIGHT)
                    SSD1306_DrawPixel((uint8_t)x, (uint8_t)y, 0); // clear
            }
        }
    }

    // draw pupil as filled smaller circle (black)
    int pupil_r = radius / 3;
    int px = xc + pupil_offset_x;
    int py = yc + pupil_offset_y;
    // draw pupil by setting pixels (we already have background 0 inside inner circle)
    for (int dy = -pupil_r; dy <= pupil_r; dy++) {
        int dx_limit = (int)sqrt((double)(pupil_r * pupil_r - dy * dy));
        for (int dx = -dx_limit; dx <= dx_limit; dx++) {
            int x = px + dx;
            int y = py + dy;
            if (x >= 0 && x < SSD1306_WIDTH && y >= 0 && y < SSD1306_HEIGHT)
                SSD1306_DrawPixel((uint8_t)x, (uint8_t)y, 1);
        }
    }

    // draw a thin outline to improve readability
    SSD1306_DrawCircleOutline(xc, yc, radius);
}

// expressions:
// 0 neutral,1 blink,2 left,3 right,4 happy,5 angry
void SSD1306_ShowEyesExpression(uint8_t expr)
{
    SSD1306_Clear();   // <<< IMPORTANT FIX

    const int left_x = 38;
    const int right_x = 89;
    const int yc = 28;
    const int radius = 12;

    switch (expr) {
        case 0: // neutral
            SSD1306_DrawEye(left_x, yc, radius, 0, 0, false);
            SSD1306_DrawEye(right_x, yc, radius, 0, 0, false);
            break;

        case 1: // blink
            SSD1306_DrawEye(left_x, yc, radius, 0, 0, true);
            SSD1306_DrawEye(right_x, yc, radius, 0, 0, true);
            break;

        case 2: // left
            SSD1306_DrawEye(left_x, yc, radius, -4, 0, false);
            SSD1306_DrawEye(right_x, yc, radius, -4, 0, false);
            break;

        case 3: // right
            SSD1306_DrawEye(left_x, yc, radius, 4, 0, false);
            SSD1306_DrawEye(right_x, yc, radius, 4, 0, false);
            break;

        case 4: // happy
            SSD1306_DrawEye(left_x, yc, radius, 0, -3, false);
            SSD1306_DrawEye(right_x, yc, radius, 0, -3, false);

            // smile
            for (int dx = -8; dx <= 8; dx++) {
                int px = (left_x + right_x) / 2 + dx;
                int py = yc + 12 + (dx*dx)/40;
                SSD1306_DrawPixel(px, py, 1);
            }
            break;

        case 5: // angry
            SSD1306_DrawEye(left_x, yc, radius, 0, 3, false);
            SSD1306_DrawEye(right_x, yc, radius, 0, 3, false);

            // eyebrows
            for (int i = 0; i < 10; i++)
            {
                SSD1306_DrawPixel(left_x  - 8 + i, yc - 12 + (i/3), 1);
                SSD1306_DrawPixel(right_x - 8 + i, yc - 12 + ((9-i)/3), 1);
            }
            break;
    }

    SSD1306_Update();
}


// cycles through expressions once; you can call repeatedly for looped animation
void SSD1306_ShowEyesAnimation(void)
{
    // neutral -> blink -> left -> neutral -> right -> neutral -> happy -> angry -> blink
    SSD1306_ShowEyesExpression(0);
    HAL_Delay(800);

    SSD1306_ShowEyesExpression(1); // blink
    HAL_Delay(220);

    SSD1306_ShowEyesExpression(0);
    HAL_Delay(300);

    SSD1306_ShowEyesExpression(2); // look left
    HAL_Delay(700);

    SSD1306_ShowEyesExpression(0);
    HAL_Delay(300);

    SSD1306_ShowEyesExpression(3); // look right
    HAL_Delay(700);

    SSD1306_ShowEyesExpression(0);
    HAL_Delay(300);

    SSD1306_ShowEyesExpression(4); // happy
    HAL_Delay(900);

    SSD1306_ShowEyesExpression(5); // angry
    HAL_Delay(900);

    SSD1306_ShowEyesExpression(1); // blink
    HAL_Delay(220);

    SSD1306_ShowEyesExpression(0);
    HAL_Delay(400);
}
