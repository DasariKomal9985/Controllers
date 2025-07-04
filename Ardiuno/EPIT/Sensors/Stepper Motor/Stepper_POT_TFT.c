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
#define DIR_PIN 11
#define STEP_PIN 12
#define EN_PIN 10

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

bool direction = true;
bool motorEnabled = true;
bool lastButtonState = HIGH;
unsigned long buttonPressTime = 0;
bool buttonHandled = false;

int potValue = 0;
unsigned long stepDelay = 1000;
unsigned long lastStepTime = 0;
bool stepState = LOW;

unsigned long lastDisplayUpdate = 0;

String lastDirectionText = "";
int lastSpeedShown = -1;
int lastBars = -1;
bool lastMotorStatus = true;

void setup() {
  Wire.begin();
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init(240, 320);
  tft.setRotation(1);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIR_PIN, direction);

  drawLayout();
  updateDisplay(true);
}

void loop() {
  potValue = analogRead(POT_PIN);
  stepDelay = map(potValue, 0, 1023, 2000, 200);
  unsigned long now = micros();

  bool buttonState = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && buttonState == LOW) {
    buttonPressTime = millis();
    buttonHandled = false;
  }
  if (lastButtonState == LOW && buttonState == HIGH) {
    unsigned long pressDuration = millis() - buttonPressTime;
    if (!buttonHandled) {
      if (pressDuration > 700) {
        motorEnabled = !motorEnabled;
        digitalWrite(EN_PIN, motorEnabled ? LOW : HIGH);
      } else {
        direction = !direction;
        digitalWrite(DIR_PIN, direction);
      }
      buttonHandled = true;
      updateDisplay(true);
    }
  }
  lastButtonState = buttonState;

  if (motorEnabled && potValue >= 10 && potValue <= 1020) {
    if (now - lastStepTime >= stepDelay) {
      lastStepTime = now;
      stepState = !stepState;
      digitalWrite(STEP_PIN, stepState);
    }
  } else {
    digitalWrite(STEP_PIN, LOW);
  }

  if (millis() - lastDisplayUpdate > 150) {
    lastDisplayUpdate = millis();
    updateDisplay(false);
  }
}

void drawLayout() {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(10, 10, 310, 35, ST77XX_YELLOW);
  tft.setCursor(20, 18);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(3);
  tft.print("Stepper Driver");

  tft.drawRect(10, 50, 310, 30, ST77XX_WHITE);
  tft.setCursor(14, 58);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.print("Short: CW/ACW | Long: ON/OFF");

  tft.setCursor(15, 98);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print("Direction:");

  tft.setCursor(15, 138);
  tft.print("Speed:");
}

void updateDisplay(bool full) {
  String currentDirection = direction ? "Clockwise" : "AntiClock";
  int currentSpeed = motorEnabled ? map(potValue, 0, 1023, 0, 100) : 0;
  int bars = (motorEnabled && potValue <= 1020) ? map(potValue, 0, 1023, 0, 27) : 0;

  if (full || currentDirection != lastDirectionText) {
    tft.fillRect(150, 90, 160, 30, ST77XX_BLACK);
    tft.setCursor(155, 98);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.print(currentDirection);
    lastDirectionText = currentDirection;
  }

  if (full || currentSpeed != lastSpeedShown || motorEnabled != lastMotorStatus) {
    tft.fillRect(150, 130, 160, 30, ST77XX_BLACK);
    tft.setCursor(165, 138);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    if (!motorEnabled)
      tft.print("OFF");
    else
      tft.print(currentSpeed);
    lastSpeedShown = currentSpeed;
  }

  if (full || bars != lastBars || motorEnabled != lastMotorStatus) {
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
    lastBars = bars;
  }

  lastMotorStatus = motorEnabled;
}
