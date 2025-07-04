#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define DIR_PIN 11
#define STEP_PIN 12
#define EN_PIN 10
#define POT_PIN A8
#define BUTTON_PIN 46

LiquidCrystal_I2C lcd(0x27, 20, 4);

bool direction = true;
bool motorEnabled = true;
bool lastButtonState = HIGH;
unsigned long buttonPressTime = 0;
bool buttonHandled = false;

long stepCount = 0;
unsigned long stepDelay = 1000;
unsigned long lastStepTime = 0;
unsigned long lastLCDUpdate = 0;

bool stepPinState = LOW;

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIR_PIN, direction);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Driver & Stepper");
}

void loop() {
  int potValue = analogRead(POT_PIN);
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
    }
  }
  lastButtonState = buttonState;

  if (motorEnabled && potValue >= 10 && potValue <= 920) {
    if (potValue > 900) {
      stepDelay = 200;
    } else {
      stepDelay = map(potValue, 10, 900, 2000, 400);
    }

    if (now - lastStepTime >= stepDelay) {
      lastStepTime = now;
      stepPinState = !stepPinState;
      digitalWrite(STEP_PIN, stepPinState);

      if (stepPinState && potValue <= 900) {
        stepCount++;
      }
    }
  } else {
    digitalWrite(STEP_PIN, LOW);
  }

  if (millis() - lastLCDUpdate > 200) {
    lastLCDUpdate = millis();
    updateLCD(potValue);
  }
}

void updateLCD(int potValue) {
  lcd.setCursor(0, 1);
  lcd.print("Steps:           ");
  lcd.setCursor(7, 1);
  if (potValue > 900 && potValue <= 920) {
    lcd.print("Inf");
  } else if (potValue < 10 || potValue > 920) {
    lcd.print("---");
  } else {
    lcd.print(stepCount);
  }

  lcd.setCursor(0, 2);
  lcd.print("Direction: ");
  lcd.print(direction ? "Clockwise   " : "Anticlock   ");

  lcd.setCursor(0, 3);
  if (!motorEnabled) {
    lcd.print("     MOTOR OFF       ");
  } else if (potValue < 10) {
    lcd.print("     POT TOO LOW     ");
  } else if (potValue > 920) {
    lcd.print("     POT TOO HIGH    ");
  } else {
    int bars = map(potValue, 10, 920, 1, 20);
    for (int i = 0; i < 20; i++) {
      lcd.print(i < bars ? char(255) : ' ');
    }
  }
}
