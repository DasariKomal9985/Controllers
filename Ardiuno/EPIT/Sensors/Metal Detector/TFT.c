#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <math.h>

#define TFT_CS     53
#define TFT_RST    8
#define TFT_DC     7
#define TFT_BL     9

const int detectPin = 22;

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

int frame = 0;
const int totalDots = 24;
const int activeDots = 6;

int centerX = 120;
int centerY = 170;
int radius = 60;

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

void drawLoadingWheel() {
  for (int i = 0; i < totalDots; i++) {
    float angle = 2 * PI * i / totalDots;
    int x = centerX + cos(angle) * radius;
    int y = centerY + sin(angle) * radius;

    if ((i + frame) % totalDots < activeDots) {
      uint8_t brightness = 255 - ((activeDots - ((i + frame) % totalDots)) * 30);
      uint16_t color = tft.color565(brightness, brightness, brightness);
      tft.fillCircle(x, y, 3, color);
    } else {
      tft.fillCircle(x, y, 3, ST77XX_BLACK);
    }
  }
  frame = (frame + 1) % totalDots;
}

void clearLoadingWheelArea() {
  tft.fillCircle(centerX, centerY, radius + 5, ST77XX_BLACK);
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
      clearLoadingWheelArea();
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

    drawLoadingWheel();
  }

  delay(100);
}
