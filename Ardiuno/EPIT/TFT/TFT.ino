#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS   53
#define TFT_RST  8
#define TFT_DC   7
#define TFT_BL   9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);  // Turn on backlight

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  tft.fillRect(10, 5, 320, 30, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_BLACK, ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(20, 12);
  tft.print("Infrared Ray Sensor");

  tft.fillRect(10, 50, 160, 40, ST77XX_BLUE);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(20, 60);
  tft.print("Status");
}

void loop() {
  // No updates in loop
}
