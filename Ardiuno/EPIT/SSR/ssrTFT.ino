#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS   53
#define TFT_RST  8
#define TFT_DC   7
#define TFT_BL   9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
#define relayPin 40

void drawThickLine(int x1, int y1, int x2, int y2, uint16_t color) {
  tft.drawLine(x1, y1, x2, y2, color);
  tft.drawLine(x1 + 1, y1, x2 + 1, y2, color);
  tft.drawLine(x1 - 1, y1, x2 - 1, y2, color);
}

void drawStaticUI() {
  tft.fillRect(0, 0, 320, 40, ST77XX_BLUE);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  int16_t tx, ty;
  uint16_t tw, th;
  tft.getTextBounds("Solid State Relay", 0, 0, &tx, &ty, &tw, &th);
  tft.setCursor((320 - tw) / 2, 10);
  tft.print("Solid State Relay");

  tft.fillRect(0, 60, 320, 40, ST77XX_CYAN);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 70); tft.print("Relay State");
  tft.drawRect(200, 60, 50, 30, ST77XX_WHITE); tft.setCursor(210, 70); tft.print("ON");
  tft.drawRect(260, 60, 50, 30, ST77XX_WHITE); tft.setCursor(268, 70); tft.print("OFF");
}

void highlightState(bool isOn) {
  if (isOn) {
    tft.fillRect(200, 60, 50, 40, ST77XX_GREEN);
    tft.setTextColor(ST77XX_BLACK); tft.setCursor(210, 70); tft.print("ON");
    tft.fillRect(260, 60, 50, 40, ST77XX_CYAN);
    tft.drawRect(260, 60, 50, 40, ST77XX_WHITE);
    tft.setCursor(268, 80); tft.setTextColor(ST77XX_BLACK); tft.print("OFF");
  } else {
    tft.fillRect(260, 60, 50, 40, ST77XX_RED);
    tft.setTextColor(ST77XX_WHITE); tft.setCursor(268, 70); tft.print("OFF");
    tft.fillRect(200, 60, 50, 40, ST77XX_CYAN);
    tft.drawRect(200, 60, 50, 40, ST77XX_WHITE);
    tft.setTextColor(ST77XX_BLACK); tft.setCursor(210, 70); tft.print("ON");
  }
}

void drawRelayDiagram(bool isOn) {
  tft.fillRect(0, 120, 320, 120, ST77XX_BLACK);

  int centerX = 130;
  int centerY = 180;
  int coilX = centerX;
  int coilY = centerY + 20;

  tft.fillRect(coilX - 25, coilY - 15, 50, 30, isOn ? ST77XX_YELLOW : ST77XX_WHITE);
  tft.drawRect(coilX - 25, coilY - 15, 50, 30, ST77XX_WHITE);

  for (int i = 0; i < 6; i++) {
    int y = coilY - 10 + i * 3;
    tft.drawLine(coilX - 20, y, coilX + 20, y, ST77XX_BLACK);
  }

  tft.fillRect(coilX - 30, coilY - 3, 10, 6, ST77XX_WHITE);
  tft.fillRect(coilX + 20, coilY - 3, 10, 6, ST77XX_WHITE);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(coilX - 40, coilY - 5); tft.print("A1");
  tft.setCursor(coilX + 35, coilY - 5); tft.print("A2");

  int contactX = centerX;
  int contactY = centerY - 30;

  tft.fillCircle(contactX, contactY, 5, ST77XX_WHITE);

  int noX = contactX - 40;
  int noY = contactY - 25;
  tft.fillCircle(noX, noY, 4, ST77XX_WHITE);

  int ncX = contactX - 40;
  int ncY = contactY + 25;
  tft.fillCircle(ncX, ncY, 4, ST77XX_WHITE);

  if (isOn) {
    drawThickLine(contactX, contactY, noX, noY, ST77XX_GREEN);
    tft.fillCircle(noX, noY, 2, ST77XX_YELLOW);
  } else {
    drawThickLine(contactX, contactY, ncX, ncY, ST77XX_RED);
    tft.fillCircle(ncX, ncY, 2, ST77XX_RED);
  }

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(noX - 15, noY - 10); tft.print("NC");
  tft.setCursor(ncX - 15, ncY + 15); tft.print("NO");
  tft.setCursor(contactX + 10, contactY - 5); tft.print("COM");

  int ledX = centerX + 80;
  int ledY = centerY - 30;

  tft.drawCircle(ledX, ledY, 10, ST77XX_WHITE);
  tft.drawCircle(ledX, ledY, 9, ST77XX_WHITE);

  if (isOn) {
    tft.fillCircle(ledX, ledY, 8, ST77XX_GREEN);
    tft.drawCircle(ledX, ledY, 12, ST77XX_GREEN);
  } else {
    tft.fillCircle(ledX, ledY, 8, ST77XX_BLACK);
  }

  drawThickLine(noX, noY, ledX - 10, ledY, isOn ? ST77XX_GREEN : ST77XX_WHITE);
  drawThickLine(ledX + 10, ledY, ledX + 40, ledY, isOn ? ST77XX_GREEN : ST77XX_WHITE);
  drawThickLine(ledX + 40, ledY, ledX + 40, centerY + 40, isOn ? ST77XX_GREEN : ST77XX_WHITE);
  drawThickLine(ledX + 40, centerY + 40, coilX - 30, centerY + 40, isOn ? ST77XX_GREEN : ST77XX_WHITE);
  drawThickLine(coilX - 30, centerY + 40, coilX - 30, coilY, isOn ? ST77XX_GREEN : ST77XX_WHITE);

  tft.fillRect(coilX + 30, centerY + 35, 30, 15, ST77XX_RED);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(coilX + 32, centerY + 37); tft.print("+12V");
  drawThickLine(coilX + 30, coilY, coilX + 30, centerY + 35, ST77XX_WHITE);
}

void setup() {
  pinMode(relayPin, OUTPUT);
  pinMode(TFT_BL, OUTPUT); digitalWrite(TFT_BL, HIGH);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  drawStaticUI();
}

void loop() {
  digitalWrite(relayPin, HIGH);
  highlightState(true);
  drawRelayDiagram(true);
  delay(3000);

  digitalWrite(relayPin, LOW);
  highlightState(false);
  drawRelayDiagram(false);
  delay(3000);
}
