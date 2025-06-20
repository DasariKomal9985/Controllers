#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <math.h>

#define TFT_CS   53
#define TFT_RST  8
#define TFT_DC   7
#define TFT_BL   9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

const int currentSensorPin = A10;
const float sensitivity = 0.185;
float vRef = 2.5;

void setup() {
  Serial.begin(9600);

  tft.init(240, 320);
  tft.setRotation(1);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.fillScreen(ST77XX_BLACK);

  tft.fillRect(10, 10, 320, 40, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(3);
  tft.setCursor(40, 22);
  tft.print("Current Sensor");

  tft.fillRect(10, 60, 320, 40, ST77XX_BLUE);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 72);
  tft.print("Measured AC Current");

  tft.drawRect(10, 110, 220, 60, ST77XX_WHITE);

  delay(1000);
  vRef = calibrateZeroCurrent(currentSensorPin);
}

void loop() {
  const int sampleCount = 1000;
  float voltage, total = 0;

  for (int i = 0; i < sampleCount; i++) {
    int adc = analogRead(currentSensorPin);
    voltage = (adc * 5.0) / 1023.0;
    float centered = voltage - vRef;
    total += centered * centered;
    delayMicroseconds(200);
  }

  float rms = sqrt(total / sampleCount);
  float current = rms / sensitivity;

  Serial.print("Measured AC Current: ");
  Serial.print(current, 3);
  Serial.println(" A");

  tft.fillRect(12, 112, 216, 56, ST77XX_BLACK);
  tft.setCursor(50, 130);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(3);
  tft.print(current, 3);
  tft.print(" A");

  delay(1000);
}

float calibrateZeroCurrent(int pin) {
  long sum = 0;
  const int readings = 500;
  for (int i = 0; i < readings; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  float avg = sum / (float)readings;
  float voltage = avg * (5.0 / 1023.0);
  return voltage;
}
