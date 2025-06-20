#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

const int currentSensorPin = A10;
const float sensitivity = 0.185;
float vRef = 2.5;

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Current Sensor");

  lcd.setCursor(0, 1);
  lcd.print("Measured AC Current");

  delay(1000);
  vRef = calibrateZeroCurrent(currentSensorPin);
}

void loop() {
  const int sampleCount = 1000;
  float voltage, total = 0;

  for (int i = 0; i < sampleCount; i++) {
    int adc = analogRead(currentSensorPin);
    voltage = (adc * 5.0) / 1023.0;
    float centered = voltage - vRef;
    total += centered * centered;
    delayMicroseconds(200);
  }

  float rms = sqrt(total / sampleCount);
  float current = rms / sensitivity;

  Serial.print("Measured AC Current: ");
  Serial.print(current, 3);
  Serial.println(" A");

  lcd.setCursor(0, 2);
  lcd.print("                    ");
  lcd.setCursor(0, 2);
  lcd.print(current, 3);
  lcd.print(" A");

  delay(1000);
}

float calibrateZeroCurrent(int pin) {
  long sum = 0;
  const int readings = 500;
  for (int i = 0; i < readings; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  float avg = sum / (float)readings;
  float voltage = avg * (5.0 / 1023.0);
  return voltage;
}
