#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); 

#define SoilMoisture A5

unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 1000;

void setup() {
  Serial.begin(9600);
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Soil Moisture Meter");
  pinMode(SoilMoisture, INPUT);
}

void loop() {
  unsigned long currentMillis = millis();

 
  if (currentMillis - lastUpdateTime >= updateInterval) {
    lastUpdateTime = currentMillis;

    int sensorValue = analogRead(SoilMoisture);
    int moisturePercent = map(sensorValue, 0, 1023, 0, 100);

    
    lcd.setCursor(0, 2);
    lcd.print("                    "); 
    lcd.setCursor(0, 2);
    lcd.print("Moisture: ");
    lcd.print(moisturePercent);
    lcd.print("%");


    String status = "";
    if (moisturePercent < 10) {
      status = "DRY";
    } else if (moisturePercent < 50) {
      status = "MOIST";
    } else {
      status = "WET";
    }

    lcd.setCursor(0, 3);
    lcd.print("                    "); 
    lcd.setCursor(0, 3);
    lcd.print("Soil Level: ");
    lcd.print(status);

   
    Serial.print("Moisture: ");
    Serial.print(moisturePercent);
    Serial.print("% - ");
    Serial.println(status);
  }
}
