#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <math.h>
#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9
const int detectPin = 25;
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
// Magnifying glass animation variables
int magnifyingGlassX = 50;
int magnifyingGlassY = 170;
int direction = 1;  // 1 for right, -1 for left
const int minX = 50;
const int maxX = 270;
const int moveSpeed = 3;
bool detectedLastState = false;
void drawTitleBoxes() {
  int boxHeight = 40;
  int boxWidth = 160;
  int yPos = 10;
  tft.fillRect(0, yPos, boxWidth, boxHeight, ST77XX_GREEN);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(2);
  int16_t x1, y1;
  uint16_t w, h;
  const char* text1 = "METAL";
  tft.getTextBounds(text1, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((boxWidth - w) / 2, yPos + (boxHeight - h) / 2);
  tft.print(text1);
  tft.fillRect(boxWidth, yPos, boxWidth, boxHeight, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_BLACK);
  const char* text2 = "DETECTOR";
  tft.getTextBounds(text2, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(boxWidth + (boxWidth - w) / 2, yPos + (boxHeight - h) / 2);
  tft.print(text2);
}
void drawStatusBox() {
  int boxHeight = 40;
  int boxWidth = 160;
  int yPos = 60;
  tft.fillRect(0, yPos, boxWidth, boxHeight, ST77XX_MAGENTA);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(15, yPos + 7);
  tft.print("  STATUS ");
  tft.drawRect(boxWidth, yPos, boxWidth, boxHeight, ST77XX_WHITE);
}


void printStatus(const char* statusText, uint16_t color) {
  int boxHeight = 40;
  int boxWidth = 160;
  int yPos = 60;
  tft.fillRect(boxWidth + 2, yPos + 2, boxWidth - 4, boxHeight - 4, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(color);
  int cursorX = boxWidth + 10;
  int cursorY = yPos + (boxHeight - 16) / 2;
  tft.setCursor(cursorX, cursorY);
  tft.print(statusText);
}
void setup() {
  Serial.begin(9600);
  pinMode(detectPin, INPUT);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  drawTitleBoxes();
  drawStatusBox();
}
unsigned long lastMoveTime = 0;
const int moveInterval = 1000;  // ms between moves (~20 FPS)
const int moveStep = 10;       // pixels per step

void drawMagnifyingGlassSmooth() {
  static int prevX = -999;
  static int prevY = -999;
  int glassRadius = 25;
  int handleLength = 30;

  // Erase old position only when needed
  if (prevX != -999 && (magnifyingGlassX != prevX || magnifyingGlassY != prevY)) {
    int extra = handleLength;
    int eraseX = prevX - glassRadius - extra;
    int eraseY = prevY - glassRadius - extra;
    int eraseW = (glassRadius + extra) * 2;
    int eraseH = (glassRadius + extra) * 2;
    tft.fillRect(eraseX, eraseY, eraseW, eraseH, ST77XX_BLACK);
  }

  // Draw lens rim
  tft.drawCircle(magnifyingGlassX, magnifyingGlassY, glassRadius, ST77XX_WHITE);
  tft.drawCircle(magnifyingGlassX, magnifyingGlassY, glassRadius - 1, ST77XX_WHITE);
  tft.drawCircle(magnifyingGlassX, magnifyingGlassY, glassRadius - 2, 0x8410);

  // Lens fill
  tft.fillCircle(magnifyingGlassX, magnifyingGlassY, glassRadius - 3, 0x3D7F);
  tft.fillCircle(magnifyingGlassX - 8, magnifyingGlassY - 8, 6, ST77XX_WHITE);
  tft.fillCircle(magnifyingGlassX - 12, magnifyingGlassY - 12, 3, ST77XX_WHITE);

  // Handle
  float angle = 2.356;
  int handleStartX = magnifyingGlassX + cos(angle) * (glassRadius - 2);
  int handleStartY = magnifyingGlassY + sin(angle) * (glassRadius - 2);
  int handleEndX = handleStartX + cos(angle) * handleLength;
  int handleEndY = handleStartY + sin(angle) * handleLength;

  for (int i = 0; i < 4; i++) {
    tft.drawLine(handleStartX + i, handleStartY, handleEndX + i, handleEndY, 0x8400);
    tft.drawLine(handleStartX, handleStartY + i, handleEndX, handleEndY + i, 0x8400);
  }
  tft.drawLine(handleStartX, handleStartY, handleEndX, handleEndY, 0xA800);

  for (int i = 5; i < handleLength - 5; i += 4) {
    int gripX = handleStartX + cos(angle) * i;
    int gripY = handleStartY + sin(angle) * i;
    tft.drawLine(gripX - 1, gripY + 1, gripX + 1, gripY - 1, 0x6000);
  }

  prevX = magnifyingGlassX;
  prevY = magnifyingGlassY;
}

void updateMagnifyingGlassPosition() {
  if (millis() - lastMoveTime >= moveInterval) {
    magnifyingGlassX += direction * moveStep;
    if (magnifyingGlassX >= maxX) direction = -1;
    else if (magnifyingGlassX <= minX) direction = 1;
    lastMoveTime = millis();
  }
}


void clearMagnifyingGlassArea() {
  tft.fillRect(0, 120, 320, 100, ST77XX_BLACK);
}
void showDetectedBox() {
  int boxWidth = 220;
  int boxHeight = 80;
  int boxX = (320 - boxWidth) / 2;
  int boxY = 160;
  tft.fillRect(boxX, boxY, boxWidth, boxHeight, ST77XX_RED);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  int textX = boxX + (boxWidth - (8 * 3 * 6)) / 2;
  int textY = boxY + (boxHeight - 24) / 2;
  tft.setCursor(textX, textY);
  tft.print("DETECTED");
}
void clearDetectedBox() {
  tft.fillRect(0, 160, 320, 80, ST77XX_BLACK);
}
void loop() {
  int state = digitalRead(detectPin);
  if (state == HIGH) {
    if (!detectedLastState) {
      Serial.println("Metal Detected");
      clearMagnifyingGlassArea();
      clearDetectedBox();
      showDetectedBox();
      printStatus("DETECTED", ST77XX_GREEN);
      detectedLastState = true;
    }
  } else {
    if (detectedLastState) {
      Serial.println("No Metal");
      clearDetectedBox();
      printStatus("NOT FOUND", ST77XX_RED);
      detectedLastState = false;
    }
    updateMagnifyingGlassPosition();
    drawMagnifyingGlassSmooth();
  }
  delay(100);
}
