#include <Wire.h>
#include <LiquidCrystal_I2C.h>
const int analogPin = A3;

LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  
  lcd.setCursor(5, 0);
  lcd.print("Dust Sensor");
}

void loop() {
  int adcValue = analogRead(analogPin);
  float voltage = adcValue * (5.0 / 1023.0);

  float dustDensity = 0;
  if (voltage > 0.9) {
    dustDensity = (voltage - 0.9) * 1000.0 / 0.5;
  }

  String level;
  if (dustDensity == 0) level = "Clear Air";
  else if (dustDensity < 50) level = "Low";
  else if (dustDensity < 150) level = "Moderate";
  else level = "High";

  Serial.print("ADC: ");
  Serial.print(adcValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V | Dust: ");
  Serial.print(dustDensity);
  Serial.print(" ug/m3 | Level: ");
  Serial.println(level);

  lcd.setCursor(0, 1);
  lcd.print("Dust ug/m3:     ");
  lcd.setCursor(12, 1);
  lcd.print(dustDensity, 1);

  lcd.setCursor(0, 2);
  lcd.print("Voltage:        ");
  lcd.setCursor(9, 2);
  lcd.print(voltage, 2);
  lcd.print(" V");

  lcd.setCursor(0, 3);
  lcd.print("Status:         ");
  lcd.setCursor(8, 3);
  lcd.print(level);
  for (int i = level.length() + 8; i < 20; i++) {
    lcd.print(" ");
  }

  delay(1000);
}
