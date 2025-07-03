#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Wire.h>
#include <SPI.h>

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9
#define POT_PIN A8
#define BUTTON_PIN 46
#define IN1 44
#define IN2 45
#define ENA 3

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

int speedValue = 0;
int lastSpeedValue = -1;
bool motorRunning = true;
String direction = "Clockwise";

bool buttonState = HIGH;
bool lastButtonState = HIGH;
unsigned long buttonPressTime = 0;

int lastBarCount = -1;
String lastDirection = "";
bool lastMotorRunning = true;

unsigned long warningStartTime = 0;
bool warningActive = false;

void updateDisplay(bool full = false) {
  static int lastBarsDrawn = -1;
  static bool lastStoppedShown = false;

  if (full || direction != lastDirection) {
    tft.fillRect(150, 90, 160, 30, ST77XX_BLACK);
    tft.setCursor(155, 98);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.print(direction);
  }

  if (full || speedValue != lastSpeedValue || motorRunning != lastMotorRunning) {
    tft.fillRect(150, 130, 160, 30, ST77XX_BLACK);
    tft.setCursor(165, 138);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    char speedStr[4];
    sprintf(speedStr, "%03d", motorRunning ? speedValue : 0);
    tft.print(speedStr);
  }

  int bars = (motorRunning && speedValue <= 240) ? map(speedValue, 0, 250, 0, 27) : 0;

  if (bars != lastBarsDrawn || full || motorRunning != lastMotorRunning) {
    if (!motorRunning || speedValue > 240) {
      if (!lastStoppedShown || full) {
        tft.fillRect(11, 176, 298, 58, ST77XX_BLACK);
        tft.setCursor(20, 195);
        tft.setTextColor(ST77XX_RED);
        tft.setTextSize(3);
        tft.print("STOPPED MOTORS");
        lastStoppedShown = true;
      }
    } else {
      if (lastStoppedShown || full) {
        tft.fillRect(11, 176, 298, 58, ST77XX_BLACK);
      }
      int xStart = 10, yBase = 234, barWidth = 9, barSpacing = 2, height = 58;
      for (int i = 0; i < 27; i++) {
        int x = xStart + i * (barWidth + barSpacing);
        if (i < bars) {
          uint16_t color = ST77XX_GREEN;
          if (i >= 5)  color = ST77XX_YELLOW;
          if (i >= 15) color = ST77XX_ORANGE;
          if (i >= 22) color = ST77XX_RED;
          tft.fillRect(x, yBase - height, barWidth, height, color);
        } else {
          tft.fillRect(x, yBase - height, barWidth, height, ST77XX_BLACK);
          tft.drawRect(x, yBase - height, barWidth, height, ST77XX_WHITE);
        }
      }
      lastStoppedShown = false;
    }

    lastBarsDrawn = bars;
  }
}

void setup() {
  Wire.begin();
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init(240, 320);
  tft.setRotation(1);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  drawStaticLayout();
  moveClockwise();
  updateDisplay(true);
}

void loop() {
  handleButton();

  int potValue = analogRead(POT_PIN);
  speedValue = map(potValue, 0, 1023, 0, 250);

  if (motorRunning && speedValue > 240) {
    if (!warningActive) {
      warningStartTime = millis();
      warningActive = true;
    } else if (millis() - warningStartTime >= 5000) {
      motorRunning = false;
      analogWrite(ENA, 0);
      updateDisplay(true);
      warningActive = false;
    }
  } else {
    warningActive = false;
  }

  if (motorRunning && speedValue <= 240) {
    analogWrite(ENA, speedValue);
  } else if (!motorRunning) {
    analogWrite(ENA, 0);
  }

  if (speedValue != lastSpeedValue || motorRunning != lastMotorRunning || direction != lastDirection) {
    updateDisplay();
    lastSpeedValue = speedValue;
    lastMotorRunning = motorRunning;
    lastDirection = direction;
  }

  delay(50);
}

void handleButton() {
  buttonState = digitalRead(BUTTON_PIN);
  static bool waitingForRelease = false;

  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonPressTime = millis();
    waitingForRelease = true;
  }

  if (buttonState == HIGH && lastButtonState == LOW && waitingForRelease) {
    unsigned long pressDuration = millis() - buttonPressTime;
    if (pressDuration >= 1000) {
      motorRunning = !motorRunning;
    } else {
      if (direction == "Clockwise") moveAntiClockwise();
      else moveClockwise();
    }
    updateDisplay(true);
    waitingForRelease = false;
  }

  lastButtonState = buttonState;
}

void moveClockwise() {
  direction = "Clockwise";
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void moveAntiClockwise() {
  direction = "AntiClockwise";
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void drawStaticLayout() {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(10, 10, 310, 35, ST77XX_YELLOW);
  tft.setCursor(20, 18);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(3);
  tft.print("L298N Driver");
  tft.drawRect(10, 50, 310, 30, ST77XX_WHITE);
  tft.setCursor(14, 58);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.print("Short: CW/ACW | Long: ON/OFF");
  tft.fillRect(10, 90, 220, 30, ST77XX_GREEN);
  tft.setCursor(15, 98);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print("Direction:");
  tft.fillRect(10, 130, 220, 30, ST77XX_GREEN);
  tft.setCursor(15, 138);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print("Speed:");
  tft.drawRect(10, 175, 300, 60, ST77XX_WHITE);
}
