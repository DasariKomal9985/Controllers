#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
const int irSensorPin = 32;
byte fullBox[8] = {
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
  pinMode(irSensorPin, INPUT);
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, fullBox);
  lcd.setCursor(0, 0);
  lcd.print("Infrared Ray Sensor");


  lcd.setCursor(0, 1);
  lcd.print("Status:");
}
void loop() {
  int sensorState = digitalRead(irSensorPin);


  lcd.setCursor(0, 2);
  if (sensorState == LOW) {
    Serial.println("Object Detected!");
    lcd.print("Object Detected   ");
  } else {
    Serial.println("No Object.");
    lcd.print("No Object         ");
  }
  lcd.setCursor(0, 3);
  if (sensorState == LOW) {
    lcd.write(byte(0));
  } else {
    lcd.print(" ");
  }
  delay(500);
}
}
