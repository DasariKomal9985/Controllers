#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

const int sensorPin = A10;
const float vRef = 5.0;
const float fullScale = 30.0;
const float zeroWindVoltage = 1.21;

byte fullBlock[8] = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, fullBlock);
}

void loop() {
  int adcValue = analogRead(sensorPin);
  float rawVoltage = adcValue * (vRef / 1023.0);
  rawVoltage = constrain(rawVoltage, 1.0, 5.0);

  float effectiveVoltage = rawVoltage - zeroWindVoltage;
  if (effectiveVoltage < 0) effectiveVoltage = 0;

  float windSpeed = (effectiveVoltage / (4.0 - (zeroWindVoltage - 1.0))) * fullScale;
  if (windSpeed < 0.3) windSpeed = 0;

  lcd.setCursor(5, 0);
  lcd.print("Anemometer");

  lcd.setCursor(0, 1);
  lcd.print("Wind Speed :     ");
  lcd.setCursor(12, 1);
  lcd.print(windSpeed, 1);
  lcd.setCursor(17, 1);
  lcd.print("m/s");

  lcd.setCursor(0, 2);
  lcd.print("Voltage :       ");
  lcd.setCursor(12, 2);
  lcd.print(effectiveVoltage, 2);
  lcd.setCursor(17, 2);
  lcd.print("V");

  lcd.setCursor(0, 3);
  int bars = map(windSpeed, 0, fullScale, 0, 20);
  for (int i = 0; i < 20; i++) {
    if (i < bars) {
      lcd.write(byte(0));
    } else {
      lcd.print(" ");
    }
  }

  delay(700);
}
