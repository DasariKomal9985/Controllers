#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int potPin = A8;
int potValue = 0;
int mappedValue = 0;

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

  Serial.print("Raw Value: ");
  Serial.print(potValue);
  Serial.print("  |  Mapped Value (0–100): ");
  Serial.println(mappedValue);

  lcd.setCursor(0, 1);
  lcd.print("Status:            ");
  lcd.setCursor(8, 1);
  if (mappedValue < 30) {
    lcd.print("Low   ");
  } else if (mappedValue < 70) {
    lcd.print("Medium");
  } else {
    lcd.print("High  ");
  }

  lcd.setCursor(0, 2);
  lcd.print("Value:            ");
  lcd.setCursor(7, 2);
  lcd.print(mappedValue);
  lcd.print("   ");

  lcd.setCursor(0, 3);
  int barLength = map(mappedValue, 0, 100, 0, 20);
  for (int i = 0; i < 20; i++) {
    if (i < barLength) {
      lcd.write(byte(255));
    } else {
      lcd.print(" ");
    }
  }

  delay(200);
}
