#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS    53
#define TFT_RST   8
#define TFT_DC    7
#define TFT_BL    9
#define BUTTON_PIN 46
#define LED_PIN    33

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

enum LedMode { OFF, ON, BLINK_100, BLINK_500, BLINK_1000 };
LedMode currentMode = OFF;

bool buttonPressed = false;
unsigned long pressStartTime = 0;
float holdTime = 0;
String statusText = "RELEASED";

bool ledState = false;
bool tftState = false;

unsigned long ledTimer = 0;
unsigned long tftTimer = 0;
unsigned int blinkInterval = 0;

bool lastTFTDrawState = false;
LedMode lastTFTMode = OFF;

void setup() {
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  drawLayout();
  drawStatus(statusText);
  drawHoldTime(0);
  drawLEDLabel("LED OFF");
  drawAnimation(false);
}

void loop() {
  handleButton();
  handleLED();
  handleTFT();
  delay(20);
}

void handleButton() {
  bool currentState = digitalRead(BUTTON_PIN);

  if (currentState == HIGH && !buttonPressed) {
    buttonPressed = true;
    pressStartTime = millis();
    statusText = "PUSHED";
    drawStatus(statusText);
  }

  if (buttonPressed && currentState == HIGH) {
    holdTime = (millis() - pressStartTime) / 1000.0;
    drawHoldTime(holdTime);
  }

  if (buttonPressed && currentState == LOW) {
    buttonPressed = false;
    holdTime = (millis() - pressStartTime) / 1000.0;
    handlePressAction(holdTime);
    statusText = "RELEASED";
    drawStatus(statusText);
    drawHoldTime(0);
  }
}

void handlePressAction(float duration) {
  if (duration >= 4.0) {
    currentMode = BLINK_1000;
    blinkInterval = 1000;
    drawLEDLabel("LED ON/1Sec");
  } else if (duration >= 3.0) {
    currentMode = BLINK_500;
    blinkInterval = 500;
    drawLEDLabel("LED 500mSec");
  } else if (duration >= 2.0) {
    currentMode = BLINK_100;
    blinkInterval = 100;
    drawLEDLabel("LED 100mSec");
  } else if (duration >= 1.0) {
    currentMode = ON;
    blinkInterval = 0;
    drawLEDLabel("LED ON");
  } else {
    currentMode = OFF;
    blinkInterval = 0;
    drawLEDLabel("LED OFF");
  }

  ledTimer = millis();
  tftTimer = millis();
  ledState = false;
  tftState = false;
  lastTFTMode = LedMode(-1);
}

void handleLED() {
  if (blinkInterval == 0) {
    digitalWrite(LED_PIN, currentMode == ON ? HIGH : LOW);
  } else {
    if (millis() - ledTimer >= blinkInterval) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      ledTimer = millis();
    }
  }
}

void handleTFT() {
  if (blinkInterval == 0) {
    if (currentMode != lastTFTMode || lastTFTDrawState != (currentMode == ON)) {
      drawAnimation(currentMode == ON);
      lastTFTDrawState = (currentMode == ON);
      lastTFTMode = currentMode;
    }
  } else {
    if (millis() - tftTimer >= blinkInterval) {
      tftState = !tftState;
      drawAnimation(tftState);
      lastTFTDrawState = tftState;
      tftTimer = millis();
    }
  }
}

void drawAnimation(bool isOn) {
  tft.fillRect(251, 91, 58, 138, ST77XX_BLACK);

  if (isOn) {
    tft.fillRect(253, 92, 55, 135, ST77XX_RED);
  } else {
    tft.drawRect(253, 92, 55, 135, ST77XX_WHITE);
  }
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
  tft.fillRect(120, 200, 110, 26, ST77XX_BLACK);
  tft.setCursor(100, 200);
  tft.print(label);
}
