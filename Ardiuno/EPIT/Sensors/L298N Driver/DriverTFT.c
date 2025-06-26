#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS     53
#define TFT_RST    8
#define TFT_DC     7
#define TFT_BL     9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

int speedValue = 180;
String direction = "Forward";

void setup() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init(240, 320);
  tft.setRotation(1);
  Serial.begin(9600);
  drawStaticLayout();
  updateDisplay();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'f': direction = "Forward"; break;
      case 'b': direction = "Backward"; break;
      case 'l': direction = "Left"; break;
      case 'r': direction = "Right"; break;
      case 's': direction = "Stopped"; break;
      case '+':
      case '=':
        speedValue += 25;
        if (speedValue > 255) speedValue = 255;
        break;
      case '-':
        speedValue -= 25;
        if (speedValue < 0) speedValue = 0;
        break;
    }
    updateDisplay();
  }
}

void drawStaticLayout() {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(10, 10, 300, 35, ST77XX_YELLOW);
  tft.setCursor(60, 20);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.print("L298N MOTOR DRIVER");

  tft.fillRect(10, 50, 300, 35, ST77XX_GREEN);
  tft.setCursor(20, 60);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print("Direction:");
  tft.drawRect(140, 50, 160, 35, ST77XX_BLUE);

  tft.fillRect(10, 90, 130, 35, ST77XX_GREEN);
  tft.setCursor(20, 100);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print("Speed:");

  tft.drawRect(150, 300, 80, 35, ST77XX_BLUE);
}

void updateDisplay() {
  tft.fillRect(145, 55, 150, 25, ST77XX_BLACK);
  tft.setCursor(160, 62);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print(direction);

  int bars = map(speedValue, 0, 255, 0, 20);
  int percent = map(speedValue, 0, 255, 0, 100);
  int xStart = 20;
  int yBase = 160;
  int barWidth = 10;
  int barSpacing = 2;

  for (int i = 0; i < 20; i++) {
    uint16_t color = ST77XX_RED;
    if (i < bars) {
      if (i < 5) color = ST77XX_GREEN;
      else if (i < 10) color = ST77XX_YELLOW;
      else if (i < 15) color = ST77XX_ORANGE;
      else color = ST77XX_RED;
      tft.fillRect(xStart + i * (barWidth + barSpacing), yBase - i, barWidth, 10 + i, color);
    } else {
      tft.fillRect(xStart + i * (barWidth + barSpacing), yBase - i, barWidth, 10 + i, ST77XX_BLACK);
      tft.drawRect(xStart + i * (barWidth + barSpacing), yBase - i, barWidth, 10 + i, ST77XX_RED);
    }
  }

  tft.fillRect(151, 111, 78, 28, ST77XX_BLACK);
  tft.setCursor(160, 100);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print(percent);
  tft.print("%");

  drawDirectionArrow(direction);
}

void drawDirectionArrow(String dir) {
  tft.fillRect(110, 190, 100, 40, ST77XX_BLACK);

  if (dir == "Forward") {
    tft.fillTriangle(160, 190, 150, 210, 170, 210, ST77XX_CYAN);
  } else if (dir == "Backward") {
    tft.fillTriangle(160, 230, 150, 210, 170, 210, ST77XX_CYAN);
  } else if (dir == "Left") {
    tft.fillTriangle(130, 210, 150, 200, 150, 220, ST77XX_CYAN);
  } else if (dir == "Right") {
    tft.fillTriangle(190, 210, 170, 200, 170, 220, ST77XX_CYAN);
  } else if (dir == "Stopped") {
    tft.setCursor(130, 200);
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(2);
    tft.print("STOP");
  }
}
