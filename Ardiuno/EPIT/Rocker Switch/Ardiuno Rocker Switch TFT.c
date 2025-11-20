#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9
#define SWITCH_PIN 41
#define LED_PIN 33

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

bool lastSwitchState = HIGH;

void setup() {
  Serial.begin(9600);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(TFT_BL, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  digitalWrite(LED_PIN, LOW);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  drawStaticLayout();

  Serial.println("System Initialized. Waiting for switch input...");
}

void loop() {
  bool switchState = digitalRead(SWITCH_PIN);
  if (switchState != lastSwitchState) {
    lastSwitchState = switchState;
    String status = switchState ? "OFF" : "ON";
    Serial.print("Switch State Changed: ");
    Serial.println(status);
    updateStatus(status);
    digitalWrite(LED_PIN, switchState ? LOW : HIGH);
  }
  delay(100);
}

void drawStaticLayout() {
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.fillRect(0, 10, 320, 40, ST77XX_YELLOW);
  tft.setCursor(30, 20);
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_RED);
  tft.print("Rocker Switch");

  tft.fillRect(0, 55, 320, 40, ST77XX_BLUE);
  tft.setCursor(30, 65);
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("Switch Status");

  tft.fillRect(0, 100, 320, 40, ST77XX_WHITE);
  tft.drawRect(0, 145, 320, 80, ST77XX_WHITE);
}

void updateStatus(String statusText) {
  tft.fillRect(0, 100, 320, 40, ST77XX_WHITE);
  tft.setCursor(120, 110);
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_BLACK);
  tft.print(statusText);


  tft.fillRect(10, 155, 100, 60, ST77XX_BLACK);
  tft.fillRect(210, 155, 100, 60, ST77XX_BLACK);


  if (statusText == "ON") {
    tft.fillRect(210, 155, 100, 60, ST77XX_RED);
    tft.setCursor(220, 175);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("LED ON");
  } else {
    tft.fillRect(10, 155, 100, 60, ST77XX_GREEN);
    tft.setCursor(20, 175);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_BLACK);
    tft.print("LED OFF");
  }
}
