#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>

#define DIR_PIN 11
#define STEP_PIN 12
#define EN_PIN 10
#define POT_PIN A8
#define BUTTON_PIN 46

LiquidCrystal_I2C lcd(0x27, 20, 4);

volatile bool stepPinState = LOW;
volatile unsigned long stepDelay = 1000;
volatile unsigned long lastToggleMicros = 0;

unsigned long lastLCDUpdate = 0;
unsigned long stepCounter = 0;
bool motorEnabled = true;
bool direction = true;

bool lastButtonState = HIGH;
unsigned long buttonPressTime = 0;
bool buttonHandled = false;

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(DIR_PIN, direction);
  digitalWrite(EN_PIN, LOW);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(" Stepper & Driver");

  Timer1.initialize(100);
  Timer1.attachInterrupt(stepperISR);
}

void loop() {
  int potValue = analogRead(POT_PIN);
  unsigned long now = millis();

  bool buttonState = digitalRead(BUTTON_PIN);

  if (lastButtonState == HIGH && buttonState == LOW) {
    buttonPressTime = now;
    buttonHandled = false;
  }

  if (lastButtonState == LOW && buttonState == HIGH) {
    unsigned long pressDuration = now - buttonPressTime;

    if (!buttonHandled) {
      if (pressDuration >= 800) {
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

  if (motorEnabled && potValue > 10 && potValue < 1000) {
    stepDelay = map(potValue, 10, 1000, 2000, 300);
  }

  if (millis() - lastLCDUpdate > 200) {
    lastLCDUpdate = millis();
    updateLCD(potValue);
  }
}

void updateLCD(int potValue) {
  lcd.setCursor(0, 1);
  lcd.print("   Steps: ");
  lcd.print(stepCounter);
  lcd.print("     ");

  lcd.setCursor(0, 2);
  lcd.print("Direction: ");
  lcd.print(direction ? "Clockwise   " : "Anticlock   ");

  lcd.setCursor(0, 3);
  if (!motorEnabled) {
    lcd.print("     MOTOR OFF       ");
  } else if (potValue <= 10 || potValue >= 1000) {
    lcd.print("    POT LIMIT REACHED");
  } else {
    int bars = map(potValue, 10, 1000, 0, 20);
    for (int i = 0; i < 20; i++) {
      lcd.setCursor(i, 3);
      lcd.write(i < bars ? byte(255) : ' ');
    }
  }
}

void stepperISR() {
  if (!motorEnabled) return;

  unsigned long now = micros();
  if (now - lastToggleMicros >= stepDelay) {
    lastToggleMicros = now;
    stepPinState = !stepPinState;
    digitalWrite(STEP_PIN, stepPinState);
    if (stepPinState) stepCounter++;
  }
}
