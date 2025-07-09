#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS    53
#define TFT_RST   8
#define TFT_DC    7
#define TFT_BL    9
#define BUTTON_PIN 46

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

bool buttonPressed = false;
unsigned long pressStartTime = 0;
float holdTime = 0;
float lastPressDuration = 0;
String statusText = "RELEASED";

void setup() {
  pinMode(BUTTON_PIN, INPUT);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  drawLayout();
  drawStatus(statusText);
  drawHoldTime(0);
  drawTotalPressTime(0);
  drawAnimation(false);
}

void loop() {
  bool currentState = digitalRead(BUTTON_PIN);

  if (currentState == HIGH && !buttonPressed) {
    buttonPressed = true;
    pressStartTime = millis();
    statusText = "PUSHED";
    drawStatus(statusText);
    drawAnimation(true);
  }

  if (buttonPressed && currentState == HIGH) {
    holdTime = (millis() - pressStartTime) / 1000.0;
    drawHoldTime(holdTime);
  }

  if (buttonPressed && currentState == LOW) {
    buttonPressed = false;
    holdTime = (millis() - pressStartTime) / 1000.0;
    lastPressDuration = holdTime;

    drawTotalPressTime(lastPressDuration);
    statusText = "RELEASED";
    drawStatus(statusText);
    drawHoldTime(0);
    drawAnimation(false);
  }

  delay(50);
}

void drawLayout() {
  tft.setTextSize(4);
  tft.setTextColor(ST77XX_WHITE);
  tft.drawRect(10, 8, 310, 60, ST77XX_WHITE);
  tft.setCursor(20, 18);
  tft.print("Push Button");
  tft.setTextSize(2);
  tft.drawRect(10, 90, 100, 40, ST77XX_WHITE);
  tft.setCursor(15, 98);
  tft.print("Status");
  tft.drawRect(115, 90, 130, 40, ST77XX_WHITE);
  tft.drawRect(10, 140, 100, 40, ST77XX_WHITE);
  tft.setCursor(15, 150);
  tft.print("Hold");
  tft.drawRect(115, 140, 130, 40, ST77XX_WHITE);
  tft.drawRect(10, 190, 100, 40, ST77XX_WHITE);
  tft.setCursor(15, 200);
  tft.print("Total");
  tft.drawRect(115, 190, 130, 40, ST77XX_WHITE);
  tft.drawRect(250, 90, 60, 140, ST77XX_WHITE);
}

void drawStatus(String status) {
  tft.fillRect(120, 100, 120, 26, ST77XX_BLACK);
  tft.setCursor(122, 100);
  tft.print(status);
}

void drawHoldTime(float seconds) {
  tft.fillRect(120, 150, 120, 26, ST77XX_BLACK);
  tft.setCursor(122, 150);
  tft.print(seconds, 2);
  tft.print("s");
}

void drawTotalPressTime(float seconds) {
  tft.fillRect(120, 200, 120, 26, ST77XX_BLACK);
  tft.setCursor(122, 200);
  tft.print(seconds, 2);
  tft.print("s");
}

void drawAnimation(bool isPressed) {
  tft.fillRect(251, 91, 58, 138, ST77XX_BLACK);
  if (isPressed) {
    tft.fillRect(253, 92, 55, 135, ST77XX_RED);
  } else {
    tft.fillRect(253, 92, 55, 135, ST77XX_GREEN);
  }
}
