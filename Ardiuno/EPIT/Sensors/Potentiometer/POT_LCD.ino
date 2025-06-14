#include <Wire.h>
#include <LiquidCrystal_I2C.h>
const int potPin = A8;
int potValue = 0;
int mappedValue = 0;
int prevMappedValue = -1;
String currentStatus = "";
String lastStatus = "";
int lastBarLength = -1;
LiquidCrystal_I2C lcd(0x27, 20, 4);
void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Potentiometer");
}
void loop() {
  potValue = analogRead(potPin);
  mappedValue = map(potValue, 0, 1023, 0, 100);
  if (mappedValue < 30) {
    currentStatus = "Low   ";
  } else if (mappedValue < 70) {
    currentStatus = "Medium";
  } else {
    currentStatus = "High  ";
  }
  if (mappedValue != prevMappedValue) {
    Serial.print("Raw Value: ");
    Serial.print(potValue);
    Serial.print("  |  Mapped Value (0–100): ");
    Serial.println(mappedValue);
  }
  if (currentStatus != lastStatus) {
    lcd.setCursor(0, 1);
    lcd.print("Status:        ");
    lcd.setCursor(8, 1);
    lcd.print(currentStatus);
    lastStatus = currentStatus;
  }
  if (mappedValue != prevMappedValue) {
    lcd.setCursor(0, 2);
    lcd.print("Value:        ");
    lcd.setCursor(7, 2);
    lcd.print(mappedValue);
    lcd.print("   ");
    prevMappedValue = mappedValue;
  }
  int barLength = map(mappedValue, 0, 100, 0, 20);
  if (barLength != lastBarLength) {
    lcd.setCursor(0, 3);
    for (int i = 0; i < 20; i++) {
      if (i < barLength) {
        lcd.write(byte(255));
      } else {
        lcd.print(" ");
      }
    }
    lastBarLength = barLength;
  }
  delay(100);
}
