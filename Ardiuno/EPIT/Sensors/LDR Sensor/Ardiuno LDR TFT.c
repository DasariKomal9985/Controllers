#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS   53
#define TFT_RST  8
#define TFT_DC   7
#define TFT_BL   9
#define LDR_PIN A7

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

int lastPercent = -1;
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 500;

void setup() {
  Serial.begin(9600);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
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

      drawVolumeBar(percent);
    }
  }
}

void drawStaticLayout() {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(0, 10, 320, 40, ST77XX_YELLOW);
  tft.setCursor(80, 20);
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_BLACK);
  tft.print("LDR Sensor");
  tft.fillRect(0, 55, 320, 40, ST77XX_BLUE);
  tft.setCursor(80, 65);
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("Light Level");
  tft.drawRect(20, 110, 280, 60, ST77XX_WHITE);
}

void drawVolumeBar(int percent) {
  int barWidth = map(percent, 0, 100, 0, 278);
  tft.fillRect(21, 111, 278, 58, ST77XX_BLACK);

  for (int x = 0; x < barWidth; x++) {
    float ratio = (float)x / 278.0;
    uint8_t r, g;
    if (ratio < 0.5) {
      r = ratio * 2 * 255;
      g = 255;
    } else {
      r = 255;
      g = (1.0 - (ratio - 0.5) * 2) * 255;
    }
    uint16_t color = tft.color565(r, g, 0);
    tft.drawFastVLine(21 + x, 111, 58, color);
  }

  tft.fillRect(0, 190, 320, 30, ST77XX_BLACK);
  tft.setCursor(100, 200);
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(percent);
  tft.print(" %");
}
