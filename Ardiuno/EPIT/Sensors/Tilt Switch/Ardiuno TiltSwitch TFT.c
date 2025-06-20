#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#define TFT_CS     53
#define TFT_RST    8
#define TFT_DC     7
#define TFT_BL     9
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
const int tiltPin = 24;
int barAngle = 0;
bool isTilted = false;
bool lastTiltedState = false;
unsigned long lastTiltTime = 0;
const unsigned long tiltDuration = 300;
const int barWidth = 20;
const int barLength = 80;
const int standWidth = 25;
const int standHeight = 15;
const int marginLeft = 10;
const int baseX = marginLeft;
const int baseY = 230;
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 30;
int prevBarAngle = -1;
void setup() {
  pinMode(tiltPin, INPUT_PULLUP);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  Serial.begin(9600);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(4);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(marginLeft, 10);
  tft.print("Tilt Switch");
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(marginLeft, 50);
  tft.print("Status:");
  tft.setCursor(marginLeft, 90);
  tft.setTextSize(4);
  tft.setTextColor(ST77XX_RED);
  tft.print("Not Tilted");
  drawStand();
  drawBarAtAngle(barAngle);
  prevBarAngle = barAngle;
}
void drawStand() {
  tft.fillRect(baseX, baseY, standWidth, standHeight, ST77XX_WHITE);
}
void clearBarAtAngle(int angle) {
  int x0 = baseX;
  int y0 = baseY - standHeight;
  float rad = radians(angle);
  float cosA = cos(rad);
  float sinA = sin(rad);
  int x1 = x0 + (int)(0 * cosA - 0 * sinA);
  int y1 = y0 + (int)(0 * sinA + 0 * cosA);
  int x2 = x0 + (int)(barWidth * cosA - 0 * sinA);
  int y2 = y0 + (int)(barWidth * sinA + 0 * cosA);
  int x3 = x0 + (int)(barWidth * cosA - (-barLength) * sinA);
  int y3 = y0 + (int)(barWidth * sinA + (-barLength) * cosA);
  int x4 = x0 + (int)(0 * cosA - (-barLength) * sinA);
  int y4 = y0 + (int)(0 * sinA + (-barLength) * cosA);
  fillPolygon(x1, y1, x2, y2, x3, y3, x4, y4, ST77XX_BLACK);
}


void drawBarAtAngle(int angle) {
  int x0 = baseX;
  int y0 = baseY - standHeight;
  float rad = radians(angle);
  float cosA = cos(rad);
  float sinA = sin(rad);
  int x1 = x0 + (int)(0 * cosA - 0 * sinA);
  int y1 = y0 + (int)(0 * sinA + 0 * cosA);
  int x2 = x0 + (int)(barWidth * cosA - 0 * sinA);
  int y2 = y0 + (int)(barWidth * sinA + 0 * cosA);
  int x3 = x0 + (int)(barWidth * cosA - (-barLength) * sinA);
  int y3 = y0 + (int)(barWidth * sinA + (-barLength) * cosA);
  int x4 = x0 + (int)(0 * cosA - (-barLength) * sinA);
  int y4 = y0 + (int)(0 * sinA + (-barLength) * cosA);
  fillPolygon(x1, y1, x2, y2, x3, y3, x4, y4, ST77XX_CYAN);
}
void fillPolygon(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4, uint16_t color) {
  tft.fillTriangle(x1, y1, x2, y2, x3, y3, color);
  tft.fillTriangle(x3, y3, x4, y4, x1, y1, color);
}
void loop() {
  unsigned long currentMillis = millis();
  int state = digitalRead(tiltPin);
  if (state == LOW) {
    lastTiltTime = currentMillis;
    isTilted = true;
  }
  if (isTilted && (currentMillis - lastTiltTime > tiltDuration)) {
    isTilted = false;
  }
  if (isTilted != lastTiltedState) {
    lastTiltedState = isTilted;
    Serial.println(isTilted ? "Tilted" : "Not Tilted");
    tft.fillRect(marginLeft, 90, 240, 30, ST77XX_BLACK);
    tft.setCursor(marginLeft, 90);
    tft.setTextSize(4);
    tft.setTextColor(isTilted ? ST77XX_GREEN : ST77XX_RED);
    tft.print(isTilted ? "Tilted" : "Not Tilted");
  }
  if (currentMillis - lastUpdate >= updateInterval) {
    lastUpdate = currentMillis;
    int targetAngle = isTilted ? 90 : 0;
    if (barAngle != targetAngle) {
      clearBarAtAngle(barAngle);
      if (barAngle < targetAngle) {
        barAngle += 5;
        if (barAngle > targetAngle) barAngle = targetAngle;
      } else if (barAngle > targetAngle) {
        barAngle -= 5;
        if (barAngle < targetAngle) barAngle = targetAngle;
      }
      drawBarAtAngle(barAngle);
      prevBarAngle = barAngle;
    }
  }
}
