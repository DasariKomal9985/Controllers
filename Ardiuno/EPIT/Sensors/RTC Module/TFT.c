#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <RTClib.h>
#include <Wire.h>

#define TFT_CS  53
#define TFT_RST 8
#define TFT_DC  7
#define TFT_BL  9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
RTC_DS3231 rtc;

#define DARKGREY 0x7BEF

const char* daysOfWeek[] = { "SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT" };

char prevDateStr[20] = "";
char prevTimeStr[20] = "";
int prevDayOfWeek = -1;

void setup() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  Serial.begin(9600);
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  // Title
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  tft.setCursor(20, 5);
  tft.print("Real Time Clock");

  // Date labels
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
  tft.setCursor(20, 45);
  tft.print("DAY");

  tft.setCursor(100, 45);
  tft.print("MONTH");

  tft.setCursor(190, 45);
  tft.print("YEAR");

  // Time labels
  tft.setCursor(20, 105);
  tft.print("HOUR");

  tft.setCursor(100, 105);
  tft.print("MIN");

  tft.setCursor(190, 105);
  tft.print("SEC");

  // Day-of-week labels (fixed spacing)
  int labelWidth = 320 / 7;
  tft.setTextSize(2);  // Important: same size used in loop
  for (int i = 0; i < 7; i++) {
    int x = i * labelWidth + (labelWidth - 3 * 12) / 2;  // 3 letters * ~12 pixels per char
    tft.setCursor(x, 180);
    tft.setTextColor(DARKGREY, ST77XX_BLACK);
    tft.print(daysOfWeek[i]);
  }
}

void loop() {
  DateTime now = rtc.now();

  int day = now.day();
  int month = now.month();
  int year = now.year() % 100;
  int hour12 = now.hour() % 12;
  if (hour12 == 0) hour12 = 12;
  int minute = now.minute();
  int second = now.second();

  char dateStr[20];
  sprintf(dateStr, "%02d%02d%02d", day, month, year);

  char timeStr[20];
  sprintf(timeStr, "%02d%02d%02d", hour12, minute, second);

  // Update date
  if (strcmp(dateStr, prevDateStr) != 0) {
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);

    tft.drawRect(10, 65, 70, 30, ST77XX_WHITE);
    tft.setCursor(20, 70);
    tft.print(day);

    tft.drawRect(90, 65, 70, 30, ST77XX_WHITE);
    tft.setCursor(100, 70);
    tft.print(month);

    tft.drawRect(170, 65, 70, 30, ST77XX_WHITE);
    tft.setCursor(180, 70);
    tft.print(year);

    strcpy(prevDateStr, dateStr);
  }

  // Update time
  if (strcmp(timeStr, prevTimeStr) != 0) {
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);

    tft.drawRect(10, 125, 70, 30, ST77XX_WHITE);
    tft.setCursor(20, 130);
    tft.print(hour12);

    tft.drawRect(90, 125, 70, 30, ST77XX_WHITE);
    tft.setCursor(100, 130);
    tft.print(minute);

    tft.drawRect(170, 125, 70, 30, ST77XX_WHITE);
    tft.setCursor(180, 130);
    tft.print(second);

    strcpy(prevTimeStr, timeStr);
  }

  // Highlight day of week
  int labelWidth = 320 / 7;
  int currentDay = now.dayOfTheWeek();

  if (prevDayOfWeek >= 0 && prevDayOfWeek != currentDay) {
    int xPrev = prevDayOfWeek * labelWidth + (labelWidth - 3 * 12) / 2;
    tft.setTextSize(2);
    tft.setTextColor(DARKGREY, ST77XX_BLACK);
    tft.setCursor(xPrev, 180);
    tft.print(daysOfWeek[prevDayOfWeek]);
  }

  if (prevDayOfWeek != currentDay) {
    int xCurr = currentDay * labelWidth + (labelWidth - 3 * 12) / 2;
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    tft.setCursor(xCurr, 180);
    tft.print(daysOfWeek[currentDay]);

    prevDayOfWeek = currentDay;
  }

  delay(500);
}
