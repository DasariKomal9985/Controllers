#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);
int relayPin = 47;

void setup() {
  pinMode(relayPin, OUTPUT);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Solid State Relay");
  lcd.setCursor(0, 1);
  lcd.print("Relay State:");
}

void loop() {
  digitalWrite(relayPin, HIGH);
  lcd.setCursor(0, 2);
  lcd.print("Relay ON     ");
  delay(2000);
  lcd.setCursor(0, 2);
  lcd.print("             ");

  digitalWrite(relayPin, LOW);
  lcd.setCursor(0, 3);
  lcd.print("Relay OFF    ");
  delay(2000);
  lcd.setCursor(0, 3);
  lcd.print("             ");
}
