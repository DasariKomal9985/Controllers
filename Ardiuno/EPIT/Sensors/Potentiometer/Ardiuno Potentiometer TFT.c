#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <math.h>

#define TFT_CS     53
#define TFT_RST    8
#define TFT_DC     7
#define TFT_BL     9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

const int potPin = A8;
int potValue = 0;
int mappedValue = 0;
int lastValue = -1;
String currentStatus = "";
String lastStatus = "";

void setup() {
  Serial.begin(9600);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  drawStaticUI();
}

void loop() {
  static int smoothedValue = 0;
  potValue = analogRead(potPin);
  smoothedValue = (smoothedValue * 7 + potValue) / 8;
  mappedValue = map(smoothedValue, 0, 1023, 0, 100);

  if (mappedValue < 30)
    currentStatus = "Low";
  else if (mappedValue < 70)
    currentStatus = "Medium";
  else
    currentStatus = "High";

  if (abs(mappedValue - lastValue) >= 2) {
    Serial.print("Raw: ");
    Serial.print(potValue);
    Serial.print(" | Mapped: ");
    Serial.println(mappedValue);
    updateValue(mappedValue);
    drawCircularBar(mappedValue);
    lastValue = mappedValue;
  }

  if (currentStatus != lastStatus) {
    updateStatus(currentStatus);
    lastStatus = currentStatus;
  }

  delay(100);
}

void drawStaticUI() {
  tft.fillRoundRect(10, 10, 300, 30, 8, ST77XX_YELLOW);
  tft.setCursor(50, 18);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.print("Potentiometer");

  tft.fillRoundRect(10, 50, 140, 30, 6, ST77XX_BLUE);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(30, 58);
  tft.print("Status");

  tft.drawRoundRect(152, 50, 155, 30, 4, ST77XX_WHITE);

  tft.fillRoundRect(10, 90, 140, 30, 6, ST77XX_GREEN);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(30, 98);
  tft.print("Value");

  tft.drawRoundRect(152, 90, 155, 30, 6, ST77XX_WHITE);
}

void updateStatus(String status) {
  tft.fillRoundRect(160, 52, 96, 26, 4, ST77XX_BLACK);
  tft.setCursor(180, 60);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print(status);
}

void updateValue(int val) {
  tft.fillRoundRect(160, 92, 96, 26, 4, ST77XX_BLACK);
  tft.setCursor(180, 100);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print(val);
}

void drawCircularBar(int value) {
  static int lastAngle = -1;
  static int lastNeedleX = -1;
  static int lastNeedleY = -1;

  int angle = map(value, 0, 100, 0, 270);
  if (angle == lastAngle) return;
  lastAngle = angle;

  int cx = 60;
  int cy = 200;
  int outerR = 35;
  int innerR = 28;

  for (int a = 0; a <= 270; a += 4) {
    float rad = radians(a - 135);
    int x1 = cx + cos(rad) * innerR;
    int y1 = cy + sin(rad) * innerR;
    int x2 = cx + cos(rad) * outerR;
    int y2 = cy + sin(rad) * outerR;
    tft.drawLine(x1, y1, x2, y2, ST77XX_BLACK);
  }

  for (int a = 0; a <= angle; a += 4) {
    float rad = radians(a - 135);
    int x1 = cx + cos(rad) * innerR;
    int y1 = cy + sin(rad) * innerR;
    int x2 = cx + cos(rad) * outerR;
    int y2 = cy + sin(rad) * outerR;
    tft.drawLine(x1, y1, x2, y2, ST77XX_GREEN);
  }

  tft.drawCircle(cx, cy, outerR, ST77XX_WHITE);

  if (lastNeedleX != -1 && lastNeedleY != -1)
    tft.drawLine(cx, cy, lastNeedleX, lastNeedleY, ST77XX_BLACK);

  float needleRad = radians(angle - 135);
  int nx = cx + cos(needleRad) * (innerR - 5);
  int ny = cy + sin(needleRad) * (innerR - 5);
  tft.drawLine(cx, cy, nx, ny, ST77XX_RED);

  lastNeedleX = nx;
  lastNeedleY = ny;

  tft.fillCircle(cx, cy, 3, ST77XX_WHITE);
}
