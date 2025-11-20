#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS 53
#define TFT_DC 7
#define TFT_RST 8
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

const int floatSensorPin = 34;

const int tankX = 240;
const int tankY = 80;
const int tankW = 80;
const int tankH = 150;

int currentWaterLevel = 0;
int targetWaterLevel = 0;
int lastWaterLevel = -1;
int waveOffset = 0;
unsigned long lastUpdate = 0;
bool isPouring = false;
int lastSensorState = -1;

void setup() {
  pinMode(floatSensorPin, INPUT_PULLUP);
  Serial.begin(9600);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  tft.fillRect(10, 8, 230, 50, ST77XX_WHITE);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(3);
  tft.setCursor(20, 20);
  tft.println("Float Sensor");

  tft.drawRect(10, 85, 220, 60, ST77XX_WHITE);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 105);
  tft.print("Status:");

  tft.drawRect(10, 160, 220, 60, ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 180);
  tft.print("Water Level:");

  drawTankOutline();
  drawTap();

  currentWaterLevel = tankH / 3;
  drawWaterFill();
}

void loop() {
  int sensorState = digitalRead(floatSensorPin);

  if (sensorState != lastSensorState) {
    lastSensorState = sensorState;

    tft.fillRect(120, 105, 70, 20, ST77XX_BLACK);
    tft.fillRect(160, 180, 60, 20, ST77XX_BLACK);

    tft.setCursor(120, 105);
    tft.setTextSize(2);
    if (sensorState == HIGH) {
      tft.setTextColor(ST77XX_GREEN);
      tft.print("HIGH");
    } else {
      tft.setTextColor(ST77XX_RED);
      tft.print("OFF ");
    }

    tft.setCursor(170, 180);
    tft.setTextSize(2);
    if (sensorState == HIGH) {
      tft.setTextColor(ST77XX_GREEN);
      tft.print("High");
      targetWaterLevel = tankH - 10;
      isPouring = true;
    } else {
      tft.setTextColor(ST77XX_RED);
      tft.print("OFF ");
      targetWaterLevel = tankH / 3;
      isPouring = false;
    }
  }

  if (millis() - lastUpdate > 50) {
    animateWaterLevel();
    isPouring ? animateWaterPour() : clearWaterPour();
    waveOffset = (waveOffset + 5) % 360;
    lastUpdate = millis();
  }

  delay(20);
}

void drawTankOutline() {
  tft.drawRoundRect(tankX, tankY, tankW, tankH, 10, ST77XX_BLUE);
  tft.drawLine(tankX + 10, tankY + tankH, tankX + 5, tankY + tankH + 10, ST77XX_BLUE);
  tft.drawLine(tankX + tankW - 10, tankY + tankH, tankX + tankW - 5, tankY + tankH + 10, ST77XX_BLUE);

  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.drawLine(tankX - 10, tankY + 20, tankX - 5, tankY + 20, ST77XX_GREEN);
  tft.setCursor(tankX - 25, tankY + 16);
  tft.print("H");

  tft.drawLine(tankX - 10, tankY + tankH - 30, tankX - 5, tankY + tankH - 30, ST77XX_RED);
  tft.setCursor(tankX - 25, tankY + tankH - 34);
  tft.print("L");
}

void drawTap() {
  int tapX = tankX + tankW / 2;
  int tapY = tankY - 40;

  tft.fillRect(tapX - 10, tapY - 15, 25, 8, ST77XX_BLACK);
  tft.drawRect(tapX - 10, tapY - 15, 25, 8, ST77XX_WHITE);
  tft.fillRect(tapX - 8, tapY - 15, 16, 20, ST77XX_BLACK);
  tft.drawRect(tapX - 8, tapY - 15, 16, 20, ST77XX_WHITE);
  tft.fillRect(tapX - 6, tapY + 5, 12, 8, ST77XX_BLACK);
  tft.drawRect(tapX - 6, tapY + 5, 12, 8, ST77XX_WHITE);
  tft.fillCircle(tapX - 15, tapY - 10, 4, ST77XX_YELLOW);
  tft.drawCircle(tapX - 15, tapY - 10, 4, ST77XX_ORANGE);
  tft.fillRect(tapX - 3, tapY + 13, 6, 4, ST77XX_BLACK);
  tft.drawRect(tapX - 3, tapY + 13, 6, 4, ST77XX_WHITE);
}

void animateWaterPour() {
  int tapX = tankX + tankW / 2;
  int nozzleY = tankY - 40 + 17;

  tft.fillRect(tapX - 8, nozzleY, 16, tankY - nozzleY, ST77XX_BLACK);

  for (int i = 0; i < (tankY - nozzleY - 5); i += 3) {
    int width = 4 + sin((i + waveOffset) * 0.1) * 1;
    int streamX = tapX - width / 2;
    tft.fillRect(streamX, nozzleY + i, width, 2, ST77XX_CYAN);

    if (i % 6 == 0) {
      int dropX = streamX + random(-2, 3);
      int dropY = nozzleY + i + random(0, 3);
      tft.drawPixel(dropX, dropY, ST77XX_BLUE);
    }
  }

  for (int i = 0; i < 4; i++) {
    int sx = tapX - 10 + i * 5 + sin(waveOffset * 0.1 + i) * 2;
    tft.drawPixel(sx, tankY - 2, ST77XX_WHITE);
  }
}

void clearWaterPour() {
  int tapX = tankX + tankW / 2;
  int nozzleY = tankY - 40 + 17;
  tft.fillRect(tapX - 8, nozzleY, 16, tankY - nozzleY, ST77XX_BLACK);
}

void animateWaterLevel() {
  if (currentWaterLevel != targetWaterLevel) {
    int step = abs(currentWaterLevel - targetWaterLevel) > 10 ? 3 : 1;
    currentWaterLevel += (currentWaterLevel < targetWaterLevel) ? step : -step;
    if (abs(currentWaterLevel - targetWaterLevel) < step)
      currentWaterLevel = targetWaterLevel;
    drawWaterFill();
  }
}

void drawWaterFill() {
  if (currentWaterLevel <= 0) return;

  int waterY = tankY + tankH - currentWaterLevel;

  if (lastWaterLevel > currentWaterLevel) {
    int clearY = tankY + tankH - lastWaterLevel;
    int clearHeight = lastWaterLevel - currentWaterLevel;
    tft.fillRect(tankX + 4, clearY, tankW - 8, clearHeight, ST77XX_BLACK);
  }

  for (int y = 0; y < currentWaterLevel; y++) {
    int levelY = waterY + y;
    uint16_t color;

    if (levelY >= tankY + tankH - (tankH / 3)) {
      color = ST77XX_RED;
    } else if (levelY >= tankY + tankH - (2 * tankH / 3)) {
      color = ST77XX_ORANGE;
    } else {
      color = ST77XX_GREEN;
    }

    tft.drawFastHLine(tankX + 4, levelY, tankW - 8, color);
  }

  for (int x = 0; x < tankW - 8; x++) {
    int h = 2 + sin((x * 0.2) + waveOffset * 0.1) * 2;
    tft.drawPixel(tankX + 4 + x, waterY + h, ST77XX_WHITE);
    tft.drawPixel(tankX + 4 + x, waterY + h + 1, ST77XX_CYAN);
  }

  drawFloat(waterY);
  lastWaterLevel = currentWaterLevel;
}

void drawFloat(int waterY) {
  if (currentWaterLevel < 20) return;

  int fx = tankX + tankW - 20;
  int fy = waterY - 5 + sin(waveOffset * 0.1) * 2;

  tft.fillRect(fx, fy, 12, 6, ST77XX_YELLOW);
  tft.drawRect(fx, fy, 12, 6, ST77XX_ORANGE);
  tft.drawLine(fx + 6, fy, fx + 6, tankY - 20, ST77XX_BLACK);
  tft.fillCircle(fx + 6, fy + 3, 2, lastSensorState == HIGH ? ST77XX_GREEN : ST77XX_RED);
}
