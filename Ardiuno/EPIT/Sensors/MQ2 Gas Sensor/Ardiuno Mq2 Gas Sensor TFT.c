#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>


#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9


const int mq2Pin = A2;
const int threshold = 5;

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

int rawValue = 0;
int gasLevel = 0;
bool isDetected = false;
int lastLevel = -1;
bool lastDetection = false;

void setup() {
  Serial.begin(9600);
  pinMode(mq2Pin, INPUT);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 10);
  tft.println("MQ2 Gas Sensor");
}

void loop() {
  rawValue = analogRead(mq2Pin);
  gasLevel = map(rawValue, 0, 1023, 0, 100);
  gasLevel = constrain(gasLevel, 0, 100);
  isDetected = gasLevel > threshold;

  logToSerial();
  updateGasDisplay();
  delay(1000);
}

void updateGasDisplay() {
  if (gasLevel != lastLevel || isDetected != lastDetection) {

    tft.fillRect(10, 50, 240, 30, ST77XX_BLACK);
    tft.setCursor(10, 50);
    tft.setTextColor(ST77XX_CYAN);
    tft.print("Gas Level: ");
    tft.print(gasLevel);
    tft.print("%");


    tft.fillRect(10, 90, 240, 30, ST77XX_BLACK);
    tft.setCursor(10, 90);
    tft.setTextColor(isDetected ? ST77XX_RED : ST77XX_GREEN);
    tft.print("Status: ");
    tft.print(isDetected ? "Detected   " : "Not Detected");

    lastLevel = gasLevel;
    lastDetection = isDetected;
  }
}

void logToSerial() {
  Serial.print("Gas Level: ");
  Serial.print(gasLevel);
  Serial.println(" %");
}
