#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int detectPin = 23;
LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
  Serial.begin(9600);
  pinMode(detectPin, INPUT);
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Metal Detector");

  lcd.setCursor(0, 1);
  lcd.print("Status:");
}

void loop() {
  int state = digitalRead(detectPin);

  lcd.setCursor(0, 2);
  if (state == HIGH) {
    Serial.println("✅ Metal Detected!");
    lcd.print("Detected         ");

    lcd.setCursor(0, 3);
    for (int i = 0; i < 20; i++) {
      lcd.write(byte(255));
    }
  } else {
    Serial.println("❌ No Metal");
    lcd.print("Not Detected     ");

    lcd.setCursor(0, 3);
    lcd.print("                    ");
  }

  delay(300);
}
