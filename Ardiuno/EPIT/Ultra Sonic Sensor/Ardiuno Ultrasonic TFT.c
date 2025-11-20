#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9
#define TRIG_PIN 48
#define ECHO_PIN 49
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
const int baseY = 150;
const int startX = 20;
const int echoX = 20;
const int triggerY = baseY - 60;
int previousObjectX = -1;
float previousDistance = -1;
int previousLabelX = -1;
void setup() {
  Serial.begin(9600);
  initTFT();
  initPins();
  drawStaticUI();
}
void loop() {
  float distance = getDistanceCM();
  float displayDistance = constrain(distance, 0, 100);
  if (abs(displayDistance - previousDistance) >= 0.5) {
    updateUI(displayDistance);
    previousDistance = displayDistance;
  }
  delay(500);
}
void initTFT() {
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  digitalWrite(TFT_BL, HIGH);
}
void initPins() {
  pinMode(TFT_BL, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}
float getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.0343 / 2;
}
void drawStaticUI() {
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(5, 5);
  tft.println("Ultrasonic Sensor");
  int boxSize = 30;
  int verticalBarWidth = 12;
  int barX = startX - verticalBarWidth - 5;
  int barTop = triggerY;
  int barHeight = (baseY + 30 + boxSize) - triggerY;
  tft.fillRect(barX, barTop, verticalBarWidth, barHeight, ST77XX_GREEN);
  tft.fillRect(startX - boxSize / 2, triggerY, boxSize, boxSize, ST77XX_GREEN);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(startX - 20, triggerY - 15);
  tft.print("Trig");
  tft.fillRect(echoX - boxSize / 2, baseY + 30, boxSize, boxSize, ST77XX_GREEN);
  tft.setCursor(echoX - 15, baseY + 65);
  tft.print("Echo");
}
void updateUI(float distance) {
  updateDistanceText(distance);
  updateObject(distance);
}
void updateDistanceText(float distance) {
  tft.fillRect(5, 25, 230, 30, ST77XX_BLACK);
  tft.setCursor(5, 35);
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(2);
  tft.print("Distance: ");
  tft.print(distance, 1);
  tft.print(" cm");
}
void updateObject(float distance) {
  if (previousObjectX != -1) {
    int prevX = previousObjectX + 100;
    tft.fillCircle(prevX, baseY, 13, ST77XX_BLACK);
    drawThickLine(startX, triggerY + 10, prevX, baseY, 4, ST77XX_BLACK);
    drawThickLine(prevX, baseY, echoX, baseY + 40, 4, ST77XX_BLACK);
    tft.fillRect(previousLabelX - 10, baseY + 12, 100, 20, ST77XX_BLACK);
  }
  float scale = distance / 100.0;
  int objectX = startX + scale * 180;
  if (objectX > 220) objectX = 220;
  int shiftedX = objectX + 100;
  int labelX = shiftedX - 60;
  drawThickLine(startX, triggerY + 10, shiftedX, baseY, 4, ST77XX_WHITE);
  tft.fillCircle(shiftedX, baseY, 12, ST77XX_RED);
  tft.fillRect(labelX - 10, baseY + 12, 100, 20, ST77XX_BLACK);
  tft.setCursor(labelX, baseY + 12);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print("Object");
  drawThickLine(shiftedX, baseY, echoX, baseY + 40, 4, ST77XX_CYAN);
  previousObjectX = objectX;
  previousLabelX = labelX;
}
void drawThickLine(int x0, int y0, int x1, int y1, int thickness, uint16_t color) {
  float dx = x1 - x0;
  float dy = y1 - y0;
  float distance = sqrt(dx * dx + dy * dy);
  float stepX = dx / distance;
  float stepY = dy / distance;
  for (int i = -thickness / 2; i < thickness / 2; i++) {
    int offsetX = i;
    int offsetY = 0;
    for (int j = 0; j < distance; j++) {
      int x = x0 + stepX * j + offsetX;
      int y = y0 + stepY * j + offsetY;
      tft.fillRect(x, y, 2, 2, color);
    }
  }
}


