#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9

const int mq2Pin = A2;

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

const int totalBars = 30;
const int barWidth = 8;
const int barSpacing = 2;
const int barHeight = 60;
const int barBaseX = 10;
const int barBaseY = 240;

int lastLevel = -1;
bool lastDetection = false;
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 500;

void setup() {
  Serial.begin(9600);
  pinMode(mq2Pin, INPUT);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.drawRect(5, 5, 310, 60, ST77XX_WHITE);
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(25, 20);
  tft.println("MQ2 Gas Sensor");

  tft.drawRect(5, 85, 150, 40, ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 92);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("Gas Level:");

  tft.drawRect(160, 85, 150, 40, ST77XX_WHITE);

  tft.drawRect(5, 130, 150, 40, ST77XX_WHITE);
  tft.setCursor(10, 138);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("Status:");

  tft.drawRect(160, 130, 150, 40, ST77XX_WHITE);

  for (int i = 0; i < totalBars; i++) {
    int x = barBaseX + i * (barWidth + barSpacing);
    tft.drawRect(x, barBaseY, barWidth, -barHeight, ST77XX_WHITE);
  }
}

void loop() {
  if (millis() - lastUpdateTime >= updateInterval) {
    lastUpdateTime = millis();

    int rawValue = analogRead(mq2Pin);
    int gasLevel = map(rawValue, 0, 1023, 0, 100);
    gasLevel = constrain(gasLevel, 0, 100);
    bool isDetected = gasLevel > 20;

    if (gasLevel != lastLevel) {
      tft.fillRect(165, 90, 60, 20, ST77XX_BLACK);
      tft.setCursor(170, 93);
      tft.setTextColor(ST77XX_CYAN);
      tft.print(gasLevel);
      tft.print("%");
      lastLevel = gasLevel;

      int activeBars = map(gasLevel, 0, 100, 0, totalBars);
      for (int i = 0; i < totalBars; i++) {
        int x = barBaseX + i * (barWidth + barSpacing);

        if (i < activeBars) {
          float percent = (i + 1) * (100.0 / totalBars);
          uint16_t color;
          if (percent <= 30) {
            color = ST77XX_GREEN;
          } else if (percent <= 50) {
            color = ST77XX_YELLOW;
          } else if (percent <= 70) {
            color = 0xFD20;
          } else {
            color = ST77XX_RED;
          }

          tft.fillRect(x + 1, barBaseY - barHeight + 1, barWidth - 2, barHeight - 2, color);
        } else {
          tft.fillRect(x + 1, barBaseY - barHeight + 1, barWidth - 2, barHeight - 2, ST77XX_BLACK);
        }
      }

      Serial.print("Gas Level: ");
      Serial.print(gasLevel);
      Serial.println(" %");
    }

    if (isDetected != lastDetection) {
      tft.fillRect(165, 131, 110, 36, ST77XX_BLACK);
      tft.setCursor(180, 140);
      tft.setTextColor(isDetected ? ST77XX_RED : ST77XX_GREEN);
      tft.print(isDetected ? "Detected" : "Safe");
      lastDetection = isDetected;

      Serial.print("Status: ");
      Serial.println(isDetected ? "Detected" : "Safe");
      Serial.println("---------------------");
    }
  }
}
