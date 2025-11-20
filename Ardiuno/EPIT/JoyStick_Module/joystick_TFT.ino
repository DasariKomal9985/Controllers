#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <math.h>

#define JOYSTICK_X A0
#define JOYSTICK_Y A1

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

#define X_MIN 320
#define X_CENTER 640
#define X_MAX 1000

#define Y_MIN 320
#define Y_CENTER 650
#define Y_MAX 1000

#define TOLERANCE 5

#define CENTER_X 160
#define CENTER_Y 120
#define RADIUS 60

int prevDotX = CENTER_X;
int prevDotY = CENTER_Y;
String prevDirection = "";
int prevMappedX = 999;
int prevMappedY = 999;

void setup() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  Serial.begin(9600);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  tft.fillRect(10, 5, 300, 30, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_BLACK, ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(20, 12);
  tft.print("Joystick Visualizer");

  drawCircleBase();
  drawDot(CENTER_X, CENTER_Y);

  tft.drawRect(10, 160, 85, 25, ST77XX_BLUE);
  tft.setTextSize(2);
  tft.setCursor(15, 165);
  tft.print("Status");

  tft.drawRect(10, 190, 40, 25, ST77XX_ORANGE);
  tft.setCursor(15, 195);
  tft.print("X:");

  tft.drawRect(220, 195, 40, 25, ST77XX_ORANGE);
  tft.setCursor(230, 200);
  tft.print("Y:");
}

void loop() {
  int rawY = analogRead(JOYSTICK_X);
  int rawX = analogRead(JOYSTICK_Y);

  int mappedX = (rawX < X_CENTER)
                  ? map(rawX, X_MIN, X_CENTER, -50, 0)
                  : map(rawX, X_CENTER, X_MAX, 0, 50);

  int mappedY = (rawY < Y_CENTER)
                  ? map(rawY, Y_MIN, Y_CENTER, -50, 0)
                  : map(rawY, Y_CENTER, Y_MAX, 0, 50);

  if (abs(mappedX) < TOLERANCE) mappedX = 0;
  if (abs(mappedY) < TOLERANCE) mappedY = 0;

  String direction = "Stable";
  if (mappedY > 10) direction = "FRONT";
  else if (mappedY < -10) direction = "BACK";
  else if (mappedX > 10) direction = "RIGHT";
  else if (mappedX < -10) direction = "LEFT";

  int dotX = CENTER_X + mappedX;
  int dotY = CENTER_Y - mappedY;

  if (direction == "Stable") {
    dotX = CENTER_X;
    dotY = CENTER_Y;
  }

  if (dotX != prevDotX || dotY != prevDotY) {
    restoreBackground(prevDotX, prevDotY);
    drawDot(dotX, dotY);
    prevDotX = dotX;
    prevDotY = dotY;
  }

  if (direction != prevDirection) {
    int x = 220;
    int y = 165;
    int w = 85;
    int h = 25;
    int border = 1;

    tft.drawRect(x, y, w, h, ST77XX_GREEN);
    tft.fillRect(x + border, y + border, w - 2 * border, h - 2 * border, ST77XX_BLACK);
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.setCursor(x + 8, y + 5);
    tft.print(direction);

    prevDirection = direction;
  }

  if (mappedX != prevMappedX) {
    int x = 60;
    int y = 190;
    int w = 50;
    int h = 25;

    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.fillRect(x + 2, y + 2, w - 4, h - 4, ST77XX_BLACK);
    tft.setCursor(x + 8, y + 6);
    tft.print(mappedX);

    prevMappedX = mappedX;
  }

  if (mappedY != prevMappedY) {
    int x = 270;
    int y = 190;
    int w = 50;
    int h = 25;

    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.fillRect(x + 2, y + 2, w - 4, h - 4, ST77XX_BLACK);
    tft.setCursor(x + 8, y + 6);
    tft.print(mappedY);

    prevMappedY = mappedY;
  }

  delay(40);
}

void drawCircleBase() {
  tft.drawCircle(CENTER_X, CENTER_Y, RADIUS, ST77XX_WHITE);
  tft.drawFastVLine(CENTER_X, CENTER_Y - RADIUS, RADIUS * 2, ST77XX_WHITE);
  tft.drawFastHLine(CENTER_X - RADIUS, CENTER_Y, RADIUS * 2, ST77XX_WHITE);

  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(1);
  tft.setCursor(CENTER_X - 10, CENTER_Y - RADIUS - 10);
  tft.print("FWD");
  tft.setCursor(CENTER_X - 10, CENTER_Y + RADIUS + 5);
  tft.print("BWD");
  tft.setCursor(CENTER_X - RADIUS - 30, CENTER_Y - 3);
  tft.print("LEFT");
  tft.setCursor(CENTER_X + RADIUS + 5, CENTER_Y - 3);
  tft.print("RIGHT");
}

void drawDot(int x, int y) {
  tft.fillCircle(x, y, 4, ST77XX_RED);
}

void restoreBackground(int x, int y) {
  int size = 5;
  tft.fillRect(x - size, y - size, size * 2, size * 2, ST77XX_BLACK);

  int dx = x - CENTER_X;
  int dy = y - CENTER_Y;
  int dist = sqrt(dx * dx + dy * dy);

  if (x >= CENTER_X - size && x <= CENTER_X + size)
    tft.drawFastVLine(CENTER_X, CENTER_Y - RADIUS, RADIUS * 2, ST77XX_WHITE);

  if (y >= CENTER_Y - size && y <= CENTER_Y + size)
    tft.drawFastHLine(CENTER_X - RADIUS, CENTER_Y, RADIUS * 2, ST77XX_WHITE);

  if (abs(dist - RADIUS) <= size)
    tft.drawCircle(CENTER_X, CENTER_Y, RADIUS, ST77XX_WHITE);
}
