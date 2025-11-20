#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define SWITCH_PIN 47      
#define LED_PIN    33

LiquidCrystal_I2C lcd(0x27, 20, 4);

bool buttonPressed = false;
bool ledState = false;
unsigned long pressStartTime = 0;
float holdTime = 0;

void setup() {
  pinMode(SWITCH_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Push Button");
  lcd.setCursor(0, 1);
  lcd.print("Status: RELEASED");
  lcd.setCursor(0, 2);
  lcd.print("Hold: 0.00s");
  lcd.setCursor(0, 3);
  lcd.print("LED: OFF");
}

void loop() {
  bool currentState = !digitalRead(SWITCH_PIN);  

  if (currentState && !buttonPressed) {
    buttonPressed = true;
    pressStartTime = millis();
    lcd.setCursor(0, 1);
    lcd.print("Status: PUSHED   ");
  }

  if (buttonPressed && currentState) {
    holdTime = (millis() - pressStartTime) / 1000.0;
    lcd.setCursor(6, 2);
    lcd.print(holdTime, 2);
    lcd.print("s ");
  }

  if (buttonPressed && !currentState) {
    buttonPressed = false;
    holdTime = (millis() - pressStartTime) / 1000.0;

    lcd.setCursor(0, 1);
    lcd.print("Status: RELEASED ");

    lcd.setCursor(6, 2);
    lcd.print("0.00s   ");

    if (holdTime >= 0.1) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      lcd.setCursor(5, 3);
      lcd.print(ledState ? "ON " : "OFF");
    }
  }
}
