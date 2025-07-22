#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "bmm150.h"
#include "bmm150_defs.h"

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9
#define ST77XX_GRAY 0x8410
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

bmm150_dev dev;
bmm150_settings settings;
bmm150_mag_data mag_data;

uint8_t bmm_addr = 0x13;
int compass_center_x = 230;
int compass_center_y = 140;
int compass_radius = 60;
float prev_needle_angle = -999;
float displayed_needle_angle = -999;

int16_t prev_x = 0, prev_y = 0, prev_z = 0;
int prev_heading = -999;
char prev_dir[4] = "";

float lastAngle = -999;

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
  int box_size = compass_radius + 25;
  tft.fillRect(compass_center_x - box_size, compass_center_y - box_size,
               2 * box_size, 100 + box_size, ST77XX_GRAY);
  tft.drawCircle(compass_center_x, compass_center_y, compass_radius, ST77XX_WHITE);
  tft.fillCircle(compass_center_x, compass_center_y, compass_radius, ST77XX_BLACK);
  tft.drawCircle(compass_center_x, compass_center_y, compass_radius - 1, ST77XX_WHITE);
  tft.drawCircle(compass_center_x, compass_center_y, compass_radius - 20, ST77XX_CYAN);

  for (int i = 0; i < 360; i += 15) {
    float rad = i * PI / 180.0;
    int dot_x = compass_center_x + (compass_radius - 5) * cos(rad);
    int dot_y = compass_center_y + (compass_radius - 5) * sin(rad);
    tft.fillCircle(dot_x, dot_y, 2, ST77XX_YELLOW);
  }

  for (int i = 0; i < 360; i += 90) {
    float rad = i * PI / 180.0;
    int x1 = compass_center_x + (compass_radius - 20) * cos(rad);
    int y1 = compass_center_y + (compass_radius - 20) * sin(rad);
    int x2 = compass_center_x + (compass_radius)*cos(rad);
    int y2 = compass_center_y + (compass_radius)*sin(rad);
    tft.drawLine(x1, y1, x2, y2, ST77XX_WHITE);
  }

  for (int i = 45; i < 360; i += 90) {
    float rad = i * PI / 180.0;
    int x1 = compass_center_x + (compass_radius - 15) * cos(rad);
    int y1 = compass_center_y + (compass_radius - 15) * sin(rad);
    int x2 = compass_center_x + (compass_radius - 5) * cos(rad);
    int y2 = compass_center_y + (compass_radius - 5) * sin(rad);
    tft.drawLine(x1, y1, x2, y2, ST77XX_CYAN);
  }

  tft.setTextSize(3);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(compass_center_x - 10, compass_center_y - compass_radius - 20);
  tft.print("N");
  tft.setCursor(compass_center_x - 10, compass_center_y + compass_radius + 5);
  tft.print("S");
  tft.setCursor(compass_center_x + compass_radius + 5, compass_center_y - 10);
  tft.print("E");
  tft.setCursor(compass_center_x - compass_radius - 20, compass_center_y - 10);
  tft.print("W");

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(compass_center_x + compass_radius / 1.1, compass_center_y - compass_radius / 1.4);
  tft.print("NE");
  tft.setCursor(compass_center_x + compass_radius / 1.2, compass_center_y + compass_radius / 1.4);
  tft.print("SE");
  tft.setCursor(compass_center_x - compass_radius / 1.4 - 15, compass_center_y + compass_radius / 1.2);
  tft.print("SW");
  tft.setCursor(compass_center_x - compass_radius / 1.4 - 15, compass_center_y - compass_radius / 1.1);
  tft.print("NW");

  tft.fillCircle(compass_center_x, compass_center_y, 6, ST77XX_YELLOW);
}

void drawCompassNeedle(float heading) {
  float angle_rad = (heading - 90) * PI / 180.0;

  if (prev_needle_angle != -999) {
    float prev_angle_rad = (prev_needle_angle - 90) * PI / 180.0;

    int prev_north_x = compass_center_x + (compass_radius - 25) * cos(prev_angle_rad);
    int prev_north_y = compass_center_y + (compass_radius - 25) * sin(prev_angle_rad);
    int prev_south_x = compass_center_x - (compass_radius - 25) * cos(prev_angle_rad);
    int prev_south_y = compass_center_y - (compass_radius - 25) * sin(prev_angle_rad);
    int left_x = compass_center_x + 6 * cos(prev_angle_rad + PI / 2);
    int left_y = compass_center_y + 6 * sin(prev_angle_rad + PI / 2);
    int right_x = compass_center_x + 6 * cos(prev_angle_rad - PI / 2);
    int right_y = compass_center_y + 6 * sin(prev_angle_rad - PI / 2);
    tft.fillTriangle(prev_north_x, prev_north_y, left_x, left_y, right_x, right_y, ST77XX_BLACK);
    tft.fillTriangle(prev_south_x, prev_south_y, left_x, left_y, right_x, right_y, ST77XX_BLACK);
    tft.fillCircle(compass_center_x, compass_center_y, 5, ST77XX_YELLOW);
    tft.drawCircle(compass_center_x, compass_center_y, 5, ST77XX_WHITE);
    tft.drawCircle(compass_center_x, compass_center_y, compass_radius - 20, ST77XX_CYAN);
  }

  int north_x = compass_center_x + (compass_radius - 25) * cos(angle_rad);
  int north_y = compass_center_y + (compass_radius - 25) * sin(angle_rad);
  int south_x = compass_center_x - (compass_radius - 25) * cos(angle_rad);
  int south_y = compass_center_y - (compass_radius - 25) * sin(angle_rad);
  int left_x = compass_center_x + 6 * cos(angle_rad + PI / 2);
  int left_y = compass_center_y + 6 * sin(angle_rad + PI / 2);
  int right_x = compass_center_x + 6 * cos(angle_rad - PI / 2);
  int right_y = compass_center_y + 6 * sin(angle_rad - PI / 2);
  tft.fillTriangle(north_x, north_y, left_x, left_y, right_x, right_y, ST77XX_RED);
  tft.fillTriangle(south_x, south_y, left_x, left_y, right_x, right_y, ST77XX_GRAY);
  tft.fillCircle(compass_center_x, compass_center_y, 5, ST77XX_YELLOW);
  tft.drawCircle(compass_center_x, compass_center_y, 5, ST77XX_WHITE);

  prev_needle_angle = heading;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  tft.init(240, 320);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

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
    while (1)
      ;
  }

  settings.pwr_mode = BMM150_POWERMODE_NORMAL;
  bmm150_set_op_mode(&settings, &dev);

  drawCompassBase();
}

void loop() {
  if (bmm150_read_mag_data(&mag_data, &dev) == BMM150_OK) {
    int16_t x = mag_data.x;
    int16_t y = mag_data.y;
    int16_t z = mag_data.z;

    float heading = atan2((float)y, (float)x) * 180.0 / PI;
    heading = -heading;  // Invert heading
    if (heading < 0) heading += 360.0;

    char dir[4] = "";
    float angle = heading;
    if (angle >= 337.5 || angle < 22.5) strcpy(dir, "North");
    else if (angle >= 22.5 && angle < 67.5) strcpy(dir, "Nprth-East");
    else if (angle >= 67.5 && angle < 112.5) strcpy(dir, "East");
    else if (angle >= 112.5 && angle < 157.5) strcpy(dir, "S0uth-East");
    else if (angle >= 157.5 && angle < 202.5) strcpy(dir, "South");
    else if (angle >= 202.5 && angle < 247.5) strcpy(dir, "South-West");
    else if (angle >= 247.5 && angle < 292.5) strcpy(dir, "West");
    else if (angle >= 292.5 && angle < 337.5) strcpy(dir, "North-West");

    if (lastAngle != -999 && abs(angle - lastAngle) < 10.0) {
      delay(100);
      return;
    }
    lastAngle = angle;

    if (displayed_needle_angle == -999) {
      displayed_needle_angle = heading;
    } else {
      float diff = heading - displayed_needle_angle;
      if (diff > 180) diff -= 360;
      if (diff < -180) diff += 360;
      displayed_needle_angle += diff * 0.2;
      if (displayed_needle_angle < 0) displayed_needle_angle += 360;
      if (displayed_needle_angle >= 360) displayed_needle_angle -= 360;
    }

    drawCompassNeedle(displayed_needle_angle);

    if (x != prev_x) {
      tft.fillRect(5, 80, 30, 10, ST77XX_BLACK);
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(10, 80);
      tft.print(x);
      prev_x = x;
    }

    if (y != prev_y) {
      tft.fillRect(55, 80, 30, 10, ST77XX_BLACK);
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(65, 80);
      tft.print(y);
      prev_y = y;
    }

    if (z != prev_z) {
      tft.fillRect(105, 80, 30, 10, ST77XX_BLACK);
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(110, 80);
      tft.print(z);
      prev_z = z;
    }

    if (heading_int != prev_heading) {
      tft.drawRect(0, 150, 140, 20, ST77XX_WHITE);
      tft.fillRect(10, 155, 80, 10, ST77XX_BLACK);
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(20, 155);
      tft.print(heading_int);
      tft.print((char)176);
      prev_heading = heading_int;
    }

    if (strcmp(dir, prev_dir) != 0) {
      tft.drawRect(0, 220, 140, 20, ST77XX_WHITE);
      tft.fillRect(10, 225, 100, 10, ST77XX_BLACK);
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(20, 225);
      tft.print(dir);
      strcpy(prev_dir, dir);
    }

    static bool labels_drawn = false;
    int boxW = 40;
    int boxH = 40;
    int baseY = 60;
    int startX = 0;
    if (!labels_drawn) {
      tft.fillRect(0, 0, 320, 50, ST77XX_WHITE);
      tft.setTextSize(3);
      tft.setTextColor(ST77XX_BLACK);
      tft.setCursor(30, 15);
      tft.print("3 Axis Compass");
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_WHITE);
      tft.drawRect(startX, baseY, boxW, boxH, ST77XX_WHITE);
      tft.setCursor(startX + 12, baseY + 5);
      tft.print("X");

      tft.drawRect(startX + boxW + 10, baseY, boxW, boxH, ST77XX_WHITE);
      tft.setCursor(startX + boxW + 10 + 12, baseY + 5);
      tft.print("Y");

      tft.drawRect(startX + 2 * (boxW + 10), baseY, boxW, boxH, ST77XX_WHITE);
      tft.setCursor(startX + 2 * (boxW + 10) + 12, baseY + 5);
      tft.print("Z");

      tft.drawRect(0, 105, 140, 40, ST77XX_WHITE);
      tft.setTextSize(2);
      tft.setTextColor(ST77XX_YELLOW);
      tft.setCursor(10, 115);
      tft.print("Heading");

      tft.drawRect(0, 175, 140, 40, ST77XX_WHITE);
      tft.setTextSize(2);
      tft.setTextColor(ST77XX_YELLOW);
      tft.setCursor(10, 185);
      tft.print("Direction");

      labels_drawn = true;
    }
  }
  delay(100);
}
