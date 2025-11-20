#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>

// LCD at I2C address 0x27, 20 chars, 4 lines
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
    // Set RTC to compile time if it's the first use or battery disconnected
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void loop() {
  DateTime now = rtc.now();
  float tempC = rtc.getTemperature();

  // Print heading on line 2
  lcd.setCursor(0, 1);
  lcd.print("Date & Time:      ");

  // Print date and time on line 3
  lcd.setCursor(0, 2);
  lcd.print(now.year());
  lcd.print('/');
  if (now.month() < 10) lcd.print('0');
  lcd.print(now.month());
  lcd.print('/');
  if (now.day() < 10) lcd.print('0');
  lcd.print(now.day());
  lcd.print("  ");
  if (now.hour() < 10) lcd.print('0');
  lcd.print(now.hour());
  lcd.print(':');
  if (now.minute() < 10) lcd.print('0');
  lcd.print(now.minute());
  lcd.print(':');
  if (now.second() < 10) lcd.print('0');
  lcd.print(now.second());

  // Print temperature on line 4
  lcd.setCursor(0, 3);
  lcd.print("Temp : ");
  lcd.print(tempC, 1); // One decimal place
  lcd.print(" C     ");

  delay(1000);
}
