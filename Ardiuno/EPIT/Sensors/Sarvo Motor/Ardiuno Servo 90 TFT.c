#include <Servo.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <math.h>


#define TFT_CS     53
#define TFT_RST    8
#define TFT_DC     7
#define TFT_BL     9


Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Servo myServo;


void setup() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);


  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(true);
  tft.setTextSize(3);


  tft.fillRect(5, 5, 320, 35, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(20, 10);
  tft.print("Servo Control");


  tft.fillRect(5, 45, 150, 40, ST77XX_MAGENTA);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(15, 50);
  tft.print("Status");


  tft.drawRect(160, 45, 160, 40, ST77XX_WHITE);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(170, 50);
  tft.print("90 deg");


  myServo.attach(2);
  myServo.write(90);


  drawProtractor(90);
}


void loop() {
}


void drawProtractor(int angleDeg) {
  int centerX = 160;
  int centerY = 220;
  int radius = 100;


  for (int r = radius - 1; r <= radius + 1; r++) {
    for (int angle = 0; angle <= 180; angle++) {
      float rad = radians(angle);
      int x = centerX + cos(rad) * r;
      int y = centerY - sin(rad) * r;
      tft.drawPixel(x, y, ST77XX_WHITE);
    }
  }


  uint16_t colors[] = {
    ST77XX_RED, ST77XX_ORANGE, ST77XX_YELLOW, ST77XX_GREEN,
    ST77XX_CYAN, ST77XX_BLUE, ST77XX_MAGENTA
  };


  int index = 0;


  for (int angle = 0; angle <= 180; angle += 30) {
    float rad = radians(angle);
    int labelX = centerX + cos(rad) * (radius + 30);
    int labelY = centerY - sin(rad) * (radius + 30);


    tft.setTextColor(colors[index % 7]);
    tft.setTextSize(2);
    tft.setCursor(labelX - 10, labelY - 8);
    tft.print(angle);


    int tickStartX = centerX + cos(rad) * (radius - 5);
    int tickStartY = centerY - sin(rad) * (radius - 5);
    int tickEndX = centerX + cos(rad) * (radius + 5);
    int tickEndY = centerY - sin(rad) * (radius + 5);
    tft.drawLine(tickStartX, tickStartY, tickEndX, tickEndY, ST77XX_WHITE);


    index++;
  }


  float pointerRad = radians(angleDeg);
  int endX = centerX + cos(pointerRad) * (radius - 20);
  int endY = centerY - sin(pointerRad) * (radius - 20);


  for (int i = -1; i <= 1; i++) {
    tft.drawLine(centerX + i, centerY, endX + i, endY, ST77XX_RED);
  }


  float arrowSize = 6;
  float leftRad = pointerRad + radians(150);
  float rightRad = pointerRad - radians(150);


  int arrowLeftX = endX + cos(leftRad) * arrowSize;
  int arrowLeftY = endY - sin(leftRad) * arrowSize;
  int arrowRightX = endX + cos(rightRad) * arrowSize;
  int arrowRightY = endY - sin(rightRad) * arrowSize;


  tft.drawLine(arrowLeftX, arrowLeftY, endX, endY, ST77XX_RED);
  tft.drawLine(arrowRightX, arrowRightY, endX, endY, ST77XX_RED);


  tft.fillCircle(centerX, centerY, 5, ST77XX_WHITE);
}


