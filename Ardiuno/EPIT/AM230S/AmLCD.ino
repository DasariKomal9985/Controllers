#include "DHT.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
#define DHTPIN 35
#define DHTTYPE DHT22 
DHT dht(DHTPIN, DHTTYPE);
void setup() {
  Serial.begin(9600);
  dht.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
}
void loop() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature(); 
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from AM2305 sensor!");
    lcd.setCursor(0, 0);
    lcd.print("Sensor read error!");
    lcd.clear();
    return;
  }
  lcd.setCursor(3, 0);
  lcd.print("AM2305 Sensor");
  lcd.setCursor(0, 1);
  lcd.print("Temperature: ");
  lcd.print(temperature, 1);
  lcd.print((char)223);   
  lcd.print("C");
  lcd.setCursor(0, 2);
  lcd.print("Humidity: ");
  lcd.print(humidity, 1);     
  lcd.print(" %");
  delay(700);
}
