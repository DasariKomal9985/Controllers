#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS   53
#define TFT_RST  8
#define TFT_DC   7
#define TFT_BL   9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

const int voltagePin = A5;
const float referenceVoltage = 5.0;
const float voltageDividerRatio = 5.0;
const float minVoltage = 10.5;
const float maxVoltage = 12.37;

float lastBatteryVoltage = -1.0;
int lastBarLevel = -1;

void setup() {
  Serial.begin(9600);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);

  drawUI();
}

void loop() {
  int sensorValue = analogRead(voltagePin);
  float voltage = sensorValue * (referenceVoltage / 1023.0);
  float batteryVoltage = voltage * voltageDividerRatio;
  float roundedVoltage = round(batteryVoltage * 100.0) / 100.0;

  if (roundedVoltage != lastBatteryVoltage) {
    lastBatteryVoltage = roundedVoltage;

    tft.fillRect(20, 95, 320, 30, ST77XX_GREEN);

    tft.setCursor(60, 100);
    tft.setTextColor(ST77XX_BLACK);
    tft.setTextSize(3);
    tft.print(roundedVoltage, 2);
    tft.print(" V");
  }

  float percentage = (batteryVoltage - minVoltage) / (maxVoltage - minVoltage);
  percentage = constrain(percentage, 0.0, 1.0);
  int level = round(percentage * 5);

  if (level != lastBarLevel) {
    lastBarLevel = level;

    drawBatteryOutline();

    for (int i = 0; i < 5; i++) {
      int segmentX = 12 + i * 48;
      int segmentY = 145;
      int segmentW = 44;
      int segmentH = 80;

      uint16_t color;
      if (i < level) {
        if (level <= 2) color = ST77XX_RED;
        else if (level == 3) color = ST77XX_ORANGE;
        else color = ST77XX_GREEN;
        tft.fillRect(segmentX, segmentY, segmentW, segmentH, color);
      } else {
        tft.fillRect(segmentX, segmentY, segmentW, segmentH, ST77XX_BLACK);
      }
    }
  }

  delay(500);
}

void drawUI() {
  tft.fillRect(10, 10, 320, 40, ST77XX_YELLOW);
  tft.setCursor(25, 20);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(3);
  tft.print("Voltage Sensor");

  tft.fillRect(10, 52, 320, 40, ST77XX_BLUE);
  tft.setCursor(30, 65);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  tft.print("Battery Voltage");

  tft.fillRect(10, 95, 320, 30, ST77XX_GREEN);

  drawBatteryOutline();
}

void drawBatteryOutline() {
  int batteryX = 10;
  int batteryY = 140;
  int batteryWidth = 240;
  int batteryHeight = 90;
  int tipWidth = 10;
  int tipHeight = 40;
  int tipX = batteryX + batteryWidth;
  int tipY = batteryY + (batteryHeight - tipHeight) / 2;

  tft.fillRect(batteryX, batteryY, batteryWidth + tipWidth + 5, batteryHeight, ST77XX_BLACK);

  tft.drawRect(batteryX, batteryY, batteryWidth, batteryHeight, ST77XX_WHITE);

  tft.fillRect(tipX, tipY, tipWidth, tipHeight, ST77XX_WHITE);

  for (int i = 1; i < 5; i++) {
    int x = batteryX + i * (batteryWidth / 5);
    tft.drawLine(x, batteryY, x, batteryY + batteryHeight, ST77XX_WHITE);
  }
}
