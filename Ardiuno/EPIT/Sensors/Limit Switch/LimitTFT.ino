#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS    53
#define TFT_RST   8
#define TFT_DC    7
#define TFT_BL    9

#define SWITCH_PIN 47
#define LED_PIN    33

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

bool switchPressed = false;
bool ledState = false;
unsigned long pressStartTime = 0;
float holdTime = 0;

void setup() {
  pinMode(SWITCH_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  drawLayout();
  drawStatus("RELEASED");
  drawHoldTime(0);
  drawLEDLabel("LED: OFF");
  drawAnimation(false);
}

void loop() {
  bool currentState = !digitalRead(SWITCH_PIN);

  if (currentState && !switchPressed) {
    switchPressed = true;
    pressStartTime = millis();
    drawStatus("PUSHED");
  }

  if (switchPressed && currentState) {
    holdTime = (millis() - pressStartTime) / 1000.0;
    drawHoldTime(holdTime);
  }

  if (switchPressed && !currentState) {
    switchPressed = false;
    holdTime = (millis() - pressStartTime) / 1000.0;

    drawStatus("RELEASED");
    drawHoldTime(0);

    if (holdTime >= 0.1) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      drawLEDLabel(ledState ? "LED: ON" : "LED: OFF");
      drawAnimation(ledState);
    }
  }

  delay(10);
}

void drawLayout() {
  tft.setTextSize(4);
  tft.setTextColor(ST77XX_WHITE);
  tft.drawRect(10, 8, 310, 60, ST77XX_WHITE);
  tft.setCursor(20, 18);
  tft.print("Limit Switch");

  tft.setTextSize(2);
  tft.drawRect(10, 90, 100, 40, ST77XX_WHITE);
  tft.setCursor(15, 98);
  tft.print("Status");
  tft.drawRect(115, 90, 130, 40, ST77XX_WHITE);

  tft.drawRect(10, 140, 100, 40, ST77XX_WHITE);
  tft.setCursor(15, 150);
  tft.print("Hold");
  tft.drawRect(115, 140, 130, 40, ST77XX_WHITE);

  tft.drawRect(10, 190, 70, 40, ST77XX_WHITE);
  tft.setCursor(15, 200);
  tft.print("LED");
  tft.drawRect(90, 190, 150, 40, ST77XX_WHITE);

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

void drawLEDLabel(String label) {
  tft.fillRect(100, 200, 130, 26, ST77XX_BLACK);
  tft.setCursor(100, 200);
  tft.print(label);
}

void drawAnimation(bool isOn) {
  tft.fillRect(251, 91, 58, 138, ST77XX_BLACK);
  if (isOn) {
    tft.fillRect(253, 92, 55, 135, ST77XX_RED);
  } else {
    tft.drawRect(253, 92, 55, 135, ST77XX_WHITE);
  }
}
