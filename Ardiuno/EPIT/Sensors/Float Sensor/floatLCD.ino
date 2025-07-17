#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

const int floatSensorPin = 34;

byte fullBar[8] = {
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
  pinMode(floatSensorPin, INPUT_PULLUP);
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.createChar(0, fullBar);

  lcd.setCursor(0, 0);
  lcd.print(" Float Sensor");
  lcd.setCursor(0, 1);
  lcd.print(" Status: ");
  lcd.setCursor(0, 2);
  lcd.print(" Water Level: ");
}

void loop() {
  int sensorState = digitalRead(floatSensorPin);

  Serial.print("Float Sensor State: ");
  Serial.println(sensorState);

  lcd.setCursor(9, 1);
  if (sensorState == HIGH) {
    lcd.print("HIGH ");
  } else {
    lcd.print("LOW  ");
  }

  lcd.setCursor(14, 2);
  if (sensorState == HIGH) {
    lcd.print("High ");
  } else {
    lcd.print("Low  ");
  }

  lcd.setCursor(0, 3);
  if (sensorState == HIGH) {
    for (int i = 0; i < 20; i++) {
      lcd.write(byte(0));
    }
  } else {
    lcd.print("                    ");
  }

  delay(500);
}
