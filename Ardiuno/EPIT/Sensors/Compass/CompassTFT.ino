#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "bmm150.h"
#include "bmm150_defs.h"

// TFT pins
#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// BMM150
bmm150_dev dev;
bmm150_settings settings;
bmm150_mag_data mag_data;

uint8_t bmm_addr = 0x13;

// Compass animation variables
int compass_center_x = 200;      // Center X for compass circle
int compass_center_y = 160;      // Center Y for compass circle
int compass_radius = 60;         // Radius of compass circle
float prev_needle_angle = -999;  // Previous needle angle for clearing
float displayed_needle_angle = -999; // For smooth animation

// I2C communication
int8_t i2c_read(uint8_t reg_addr, uint8_t* data, uint32_t len, void* intf_ptr) {
  Wire.beginTransmission(bmm_addr);
  Wire.write(reg_addr);
  Wire.endTransmission(false);
  Wire.requestFrom(bmm_addr, (uint8_t)len);
  for (uint32_t i = 0; i < len && Wire.available(); i++) {
    data[i] = Wire.read();
  }
  return 0;
}

int8_t i2c_write(uint8_t reg_addr, const uint8_t* data, uint32_t len, void* intf_ptr) {
  Wire.beginTransmission(bmm_addr);
  Wire.write(reg_addr);
  for (uint32_t i = 0; i < len; i++) {
    Wire.write(data[i]);
  }
  Wire.endTransmission();
  return 0;
}

void delay_us(uint32_t period, void* intf_ptr) {
  delayMicroseconds(period);
}

void drawCompassBase() {
  // Outer circle
  tft.drawCircle(compass_center_x, compass_center_y, compass_radius, ST77XX_WHITE);
  tft.drawCircle(compass_center_x, compass_center_y, compass_radius - 1, ST77XX_WHITE);

  // Inner circle
  tft.drawCircle(compass_center_x, compass_center_y, compass_radius - 20, ST77XX_CYAN);

  // Decorative dots around the outer circle
  for (int i = 0; i < 360; i += 15) {
    float rad = i * PI / 180.0;
    int dot_x = compass_center_x + (compass_radius - 5) * cos(rad);
    int dot_y = compass_center_y + (compass_radius - 5) * sin(rad);
    tft.fillCircle(dot_x, dot_y, 2, ST77XX_YELLOW);
  }

  // Draw long compass points (N, S, E, W)
  for (int i = 0; i < 360; i += 90) {
    float rad = i * PI / 180.0;
    int x1 = compass_center_x + (compass_radius - 20) * cos(rad);
    int y1 = compass_center_y + (compass_radius - 20) * sin(rad);
    int x2 = compass_center_x + (compass_radius) * cos(rad);
    int y2 = compass_center_y + (compass_radius) * sin(rad);
    tft.drawLine(x1, y1, x2, y2, ST77XX_WHITE);
  }

  // Draw short compass points (NE, SE, SW, NW)
  for (int i = 45; i < 360; i += 90) {
    float rad = i * PI / 180.0;
    int x1 = compass_center_x + (compass_radius - 15) * cos(rad);
    int y1 = compass_center_y + (compass_radius - 15) * sin(rad);
    int x2 = compass_center_x + (compass_radius - 5) * cos(rad);
    int y2 = compass_center_y + (compass_radius - 5) * sin(rad);
    tft.drawLine(x1, y1, x2, y2, ST77XX_CYAN);
  }

  // Draw direction labels
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(compass_center_x - 10, compass_center_y - compass_radius - 20);
  tft.print("N");
  tft.setCursor(compass_center_x - 10, compass_center_y + compass_radius + 5);
  tft.print("S");
  tft.setCursor(compass_center_x + compass_radius + 5, compass_center_y - 10);
  tft.print("E");
  tft.setCursor(compass_center_x - compass_radius - 20, compass_center_y - 10);
  tft.print("W");

  // Diagonal labels (NE, SE, SW, NW)
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(compass_center_x + compass_radius / 1.4, compass_center_y - compass_radius / 1.4);
  tft.print("NE");
  tft.setCursor(compass_center_x + compass_radius / 1.4, compass_center_y + compass_radius / 1.4);
  tft.print("SE");
  tft.setCursor(compass_center_x - compass_radius / 1.4 - 15, compass_center_y + compass_radius / 1.4);
  tft.print("SW");
  tft.setCursor(compass_center_x - compass_radius / 1.4 - 15, compass_center_y - compass_radius / 1.4);
  tft.print("NW");

  // Center circle
  tft.fillCircle(compass_center_x, compass_center_y, 8, ST77XX_YELLOW);
  tft.drawCircle(compass_center_x, compass_center_y, 8, ST77XX_WHITE);
}

void drawCompassNeedle(float heading) {
  // Convert heading to radians (0° = North, clockwise)
  float angle_rad = (heading - 90) * PI / 180.0;  // -90 to make 0° point north

  // Clear previous needle if it exists
  if (prev_needle_angle != -999) {
    float prev_angle_rad = (prev_needle_angle - 90) * PI / 180.0;

    // Clear previous needle (draw in black)
    int prev_end_x = compass_center_x + (compass_radius - 10) * cos(prev_angle_rad);
    int prev_end_y = compass_center_y + (compass_radius - 10) * sin(prev_angle_rad);
    int prev_tail_x = compass_center_x - 15 * cos(prev_angle_rad);
    int prev_tail_y = compass_center_y - 15 * sin(prev_angle_rad);

    // Thicker erase
    for (int i = -1; i <= 1; i++) {
      tft.drawLine(compass_center_x + i, compass_center_y, prev_end_x + i, prev_end_y, ST77XX_BLACK);
    }
    tft.drawLine(compass_center_x, compass_center_y, prev_tail_x, prev_tail_y, ST77XX_BLACK);
  }

  // Calculate needle endpoints
  int needle_end_x = compass_center_x + (compass_radius - 10) * cos(angle_rad);
  int needle_end_y = compass_center_y + (compass_radius - 10) * sin(angle_rad);
  int needle_tail_x = compass_center_x - 15 * cos(angle_rad);
  int needle_tail_y = compass_center_y - 15 * sin(angle_rad);

  // Draw new needle
  // North pointing end (red, thicker)
  for (int i = -1; i <= 1; i++) {
    tft.drawLine(compass_center_x + i, compass_center_y, needle_end_x + i, needle_end_y, ST77XX_RED);
  }

  // South pointing end (white)
  tft.drawLine(compass_center_x, compass_center_y, needle_tail_x, needle_tail_y, ST77XX_WHITE);

  // Redraw center dot to cover needle intersection
  tft.fillCircle(compass_center_x, compass_center_y, 8, ST77XX_YELLOW);
  tft.drawCircle(compass_center_x, compass_center_y, 8, ST77XX_WHITE);

  prev_needle_angle = heading;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // TFT setup
  tft.init(240, 320);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  // BMM150 setup
  dev.intf = BMM150_I2C_INTF;
  dev.intf_ptr = &bmm_addr;
  dev.read = i2c_read;
  dev.write = i2c_write;
  dev.delay_us = delay_us;

  if (bmm150_init(&dev) != BMM150_OK) {
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.print("BMM150 Init Failed!");
    while (1);
  }

  settings.pwr_mode = BMM150_POWERMODE_NORMAL;
  bmm150_set_op_mode(&settings, &dev);

  // Draw compass base only
  drawCompassBase();
}

void loop() {
  if (bmm150_read_mag_data(&mag_data, &dev) == BMM150_OK) {
    int16_t x = mag_data.x;
    int16_t y = mag_data.y;

    float heading = atan2((float)y, (float)x) * 180.0 / PI;
    if (heading < 0) heading += 360.0;

    // Smoothly move the needle
    if (displayed_needle_angle == -999) {
      displayed_needle_angle = heading;
    } else {
      float diff = heading - displayed_needle_angle;
      // Handle wrap-around
      if (diff > 180) diff -= 360;
      if (diff < -180) diff += 360;
      displayed_needle_angle += diff * 0.2;  // Adjust 0.2 for smoothness
      if (displayed_needle_angle < 0) displayed_needle_angle += 360;
      if (displayed_needle_angle >= 360) displayed_needle_angle -= 360;
    }

    // Draw compass needle with smooth animation
    drawCompassNeedle(displayed_needle_angle);

    // No heading, direction, or X/Y/Z display
  }

  delay(100);  // Reduced delay for smoother animation
}