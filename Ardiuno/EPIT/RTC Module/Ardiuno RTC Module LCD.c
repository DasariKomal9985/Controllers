#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);
RTC_DS3231 rtc;

void setup() {
  Wire.begin();
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Real Time Clock");

  if (!rtc.begin()) {
    lcd.setCursor(0, 1);
    lcd.print("RTC Not Found!");
    while (1);
  }

  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void loop() {
  DateTime now = rtc.now();
  float tempC = rtc.getTemperature();

  lcd.setCursor(0, 1);
  lcd.print("Date & Time:      ");

  lcd.setCursor(0, 2);
  lcd.print(now.year()); lcd.print('/');
  if (now.month() < 10) lcd.print('0');
  lcd.print(now.month()); lcd.print('/');
  if (now.day() < 10) lcd.print('0');
  lcd.print(now.day()); lcd.print("  ");
  if (now.hour() < 10) lcd.print('0');
  lcd.print(now.hour()); lcd.print(':');
  if (now.minute() < 10) lcd.print('0');
  lcd.print(now.minute()); lcd.print(':');
  if (now.second() < 10) lcd.print('0');
  lcd.print(now.second());

  lcd.setCursor(0, 3);
  lcd.print("Temp : ");
  lcd.print(tempC, 1);
  lcd.print(" C     ");

  delay(1000);
}
