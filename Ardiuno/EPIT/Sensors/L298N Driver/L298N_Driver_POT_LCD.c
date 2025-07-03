#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define IN1 44
#define IN2 45
#define ENA 3
#define POT_PIN A8
#define BUTTON_PIN 46

LiquidCrystal_I2C lcd(0x27, 20, 4);

int speedValue = 0;
int lastSpeedValue = -1;
String direction = "Clockwise";
bool motorRunning = true;

unsigned long buttonPressTime = 0;
bool buttonState = HIGH;
bool lastButtonState = HIGH;

int lastBarCount = -1;

byte fullBlock[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

void setup() {
  Wire.begin();
  lcd.init();
  lcd.backlight();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  lcd.createChar(0, fullBlock);
  lcd.setCursor(0, 0);
  lcd.print("L298N Potentiometer");

  moveClockwise();
  updateLCD();
}

void loop() {
  handleButton();

  int potValue = analogRead(POT_PIN);
  speedValue = map(potValue, 0, 1023, 0, 255);

  if (motorRunning) {
    analogWrite(ENA, speedValue);
  } else {
    analogWrite(ENA, 0);
  }

  if (speedValue != lastSpeedValue) {
    updateLCD();
    lastSpeedValue = speedValue;
  }

  delay(100);
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
      if (direction == "Clockwise") {
        moveAntiClockwise();
      } else {
        moveClockwise();
      }
    }
    updateLCD();
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
  direction = "AntiClock";
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void updateLCD() {
  lcd.setCursor(0, 1);
  lcd.print("Direction: ");
  lcd.print(direction);
  lcd.print("     ");

  lcd.setCursor(0, 2);
  lcd.print("Speed: ");
  lcd.print(motorRunning ? speedValue : 0);
  lcd.print("     ");

  int barCount = map(motorRunning ? speedValue : 0, 0, 255, 0, 20);

  for (int i = 0; i < 20; i++) {
    lcd.setCursor(i, 3);
    if (i < barCount) {
      lcd.write(byte(0));
    } else {
      lcd.print(" ");
    }
  }
}
