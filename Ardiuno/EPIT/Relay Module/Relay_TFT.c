#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS   53
#define TFT_RST  8
#define TFT_DC   7
#define TFT_BL   9
#define RELAY_PIN 43

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

bool relayState = false;
unsigned long lastToggleTime = 0;
const unsigned long toggleInterval = 2000;

void setup() {
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  drawStaticLayout();
  updateRelayDisplay();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastToggleTime >= toggleInterval) {
    relayState = !relayState;
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
    lastToggleTime = currentMillis;
    updateRelayDisplay();
  }

  delay(100);
}

void drawStaticLayout() {
  tft.fillRect(10, 10, 320, 40, ST77XX_YELLOW);
  tft.setCursor(20, 20);
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_RED);
  tft.print("Relay Module");

  tft.fillRect(10, 60, 320, 40, ST77XX_BLUE);
  tft.setCursor(20, 70);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("Status");

  tft.drawRect(10, 110, 310, 40, ST77XX_WHITE);

  tft.drawRect(10, 170, 100, 60, ST77XX_WHITE);
  tft.drawRect(130, 170, 100, 60, ST77XX_WHITE);
}

void updateRelayDisplay() {
  tft.fillRect(12, 112, 216, 36, ST77XX_BLACK);
  tft.setCursor(90, 120);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_GREEN);
  tft.print(relayState ? "ON" : "OFF");

  tft.fillRect(12, 172, 96, 56, ST77XX_BLACK);
  tft.fillRect(132, 172, 96, 56, ST77XX_BLACK);

  if (relayState) {
    tft.fillRect(132, 172, 96, 56, ST77XX_GREEN);
  } else {
    tft.fillRect(12, 172, 96, 56, ST77XX_RED);
  }

  Serial.println(relayState ? "Relay ON" : "Relay OFF");
}
