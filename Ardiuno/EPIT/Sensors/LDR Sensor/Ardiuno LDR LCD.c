#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define LDR_PIN A7

LiquidCrystal_I2C lcd(0x27, 20, 4);

int lastPercent = -1;
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 500;

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  drawStaticLayout();
}

void loop() {
  unsigned long now = millis();

  if (now - lastUpdate >= updateInterval) {
    lastUpdate = now;

    int ldrValue = analogRead(LDR_PIN);
    int percent = map(ldrValue, 0, 1023, 100, 0);

    if (abs(percent - lastPercent) >= 3) {
      lastPercent = percent;

      Serial.print("LDR: ");
      Serial.print(ldrValue);
      Serial.print(" | Brightness: ");
      Serial.print(percent);
      Serial.println("%");

      drawBar(percent);
    }
  }
}

void drawStaticLayout() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LDR Sensor");
  lcd.setCursor(0, 1);
  lcd.print("Light Intensity");
}

void drawBar(int percent) {
  int numBlocks = map(percent, 0, 100, 0, 20);
  lcd.setCursor(0, 2);

  for (int i = 0; i < 20; i++) {
    if (i < numBlocks)
      lcd.write(byte(255));
    else
      lcd.print(" ");
  }

  lcd.setCursor(0, 3);
  lcd.print("Brightness: ");
  lcd.print(percent);
  lcd.print("%   ");
}
