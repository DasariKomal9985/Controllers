#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
Servo myServo;
LiquidCrystal_I2C lcd(0x27, 20, 4);
void setup() {
  myServo.attach(2);
  myServo.write(180);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Servo Control");
  lcd.setCursor(0, 1);
  lcd.print("Status");
  lcd.setCursor(0, 2);
  lcd.print("180 deg");
}
void loop() {
}
