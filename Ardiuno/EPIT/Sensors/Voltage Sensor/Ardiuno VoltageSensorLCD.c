#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

const int voltagePin = A5;
const float referenceVoltage = 5.0;
const float voltageDividerRatio = 5.0;
const float minVoltage = 10.5;
const float maxVoltage = 12.37;

float lastBatteryVoltage = -1.0;
int lastBarLength = -1;

void setup() {
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);
  lcd.setCursor(0, 0);
  lcd.print("Voltage Sensor");
  lcd.setCursor(0, 1);
  lcd.print("Battery Voltage:");
}

void loop() {
  int sensorValue = analogRead(voltagePin);
  float voltage = sensorValue * (referenceVoltage / 1023.0);
  float batteryVoltage = voltage * voltageDividerRatio;
  float roundedVoltage = round(batteryVoltage * 100.0) / 100.0;

  if (roundedVoltage != lastBatteryVoltage) {
    lastBatteryVoltage = roundedVoltage;
    lcd.setCursor(0, 2);
    lcd.print("                ");
    lcd.setCursor(0, 2);
    lcd.print(roundedVoltage, 2);
    lcd.print(" V");
  }

  float percentage = (batteryVoltage - minVoltage) / (maxVoltage - minVoltage);
  percentage = constrain(percentage, 0.0, 1.0);
  int barLength = round(percentage * 20);

  if (barLength != lastBarLength) {
    lastBarLength = barLength;
    lcd.setCursor(0, 3);
    for (int i = 0; i < 20; i++) {
      if (i < barLength) {
        lcd.write(byte(255));
      } else {
        lcd.print(" ");
      }
    }
  }

  delay(500);
}
