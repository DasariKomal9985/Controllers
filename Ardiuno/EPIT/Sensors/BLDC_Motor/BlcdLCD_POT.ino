#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define POT_PIN A8
#define ESC_PIN 2
#define BUTTON_PIN 46

Servo esc;
LiquidCrystal_I2C lcd(0x27, 20, 4);

bool buttonState = false;
bool lastButtonReading = HIGH;
bool toggledState = false;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

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
  Serial.begin(9600);
  esc.attach(ESC_PIN);
  esc.writeMicroseconds(1000);
  delay(2000);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, fullBlock);
  lcd.setCursor(0, 0);
  lcd.print("BLDC Motor");
}

void loop() {
  bool currentReading = digitalRead(BUTTON_PIN);

  if (currentReading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (currentReading == LOW && !buttonState) {
      toggledState = !toggledState;
      Serial.print("Button Pressed → ");
      Serial.println(toggledState ? "ON" : "OFF");
      buttonState = true;
    } else if (currentReading == HIGH) {
      buttonState = false;
    }
  }

  lastButtonReading = currentReading;

  int potVal = analogRead(POT_PIN);
  int mappedVal = map(potVal, 0, 840, 0, 100);
  int escSpeed = map(potVal, 0, 840, 1000, 2000);

  if (toggledState) {
    esc.writeMicroseconds(escSpeed);
  } else {
    esc.writeMicroseconds(1000);
  }

  Serial.print("Raw: ");
  Serial.print(potVal);
  Serial.print("  |  Mapped (0–100): ");
  Serial.print(mappedVal);
  Serial.print("  |  ESC PWM: ");
  Serial.println(toggledState ? escSpeed : 1000);

  lcd.setCursor(0, 1);
  lcd.print("Button State: ");
  lcd.print(toggledState ? "ON " : "OFF");

  lcd.setCursor(0, 2);
  lcd.print("Speed: ");
  lcd.print(mappedVal);
  lcd.print("%     ");

  int blocks = map(mappedVal, 0, 100, 0, 20);
  lcd.setCursor(0, 3);
  for (int i = 0; i < 20; i++) {
    if (i < blocks) {
      lcd.write(byte(0));
    } else {
      lcd.print(" ");
    }
  }

  delay(100);
}
