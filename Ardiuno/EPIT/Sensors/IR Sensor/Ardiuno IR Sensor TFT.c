#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>


#define TFT_CS   53
#define TFT_RST  8
#define TFT_DC   7
#define TFT_BL   9


const int irSensorPin = 32;
int lastSensorState = -1;


Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);


void setup() {
  pinMode(irSensorPin, INPUT);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);


  Serial.begin(9600);
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
  int sensorState = digitalRead(irSensorPin);


  if (sensorState != lastSensorState) {
    lastSensorState = sensorState;


    tft.fillRect(160, 50, 160, 40, ST77XX_BLACK);


    if (sensorState == LOW) {
      Serial.println("Object Detected!");


      tft.fillRect(160, 50, 160, 40, ST77XX_RED);
      tft.setTextColor(ST77XX_WHITE, ST77XX_RED);
      tft.setCursor(170, 60);
      tft.print("Detected");


      tft.fillRect(10, 150, 320, 100, ST77XX_RED);
      tft.setTextColor(ST77XX_WHITE, ST77XX_RED);
      tft.setCursor(60, 180);
      tft.print("Object Detected");


    } else {
      Serial.println("No Object.");
      tft.fillRect(160, 50, 160, 40, ST77XX_GREEN);
      tft.setTextColor(ST77XX_BLACK, ST77XX_GREEN);
      tft.setCursor(170, 60);
      tft.print("No Object");


      tft.fillRect(10, 150, 320, 100, ST77XX_BLACK);
    }
  }


  delay(100);
}
