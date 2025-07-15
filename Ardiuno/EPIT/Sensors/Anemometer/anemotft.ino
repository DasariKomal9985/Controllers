#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <math.h>

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

#define sensorPin A10
const float vRef = 5.0;
const float fullScale = 30.0;
const float zeroWindVoltage = 1.21;

#define CENTER_X 230
#define CENTER_Y 170
#define POLE_WIDTH 6
#define POLE_HEIGHT 80
#define BASE_WIDTH 40
#define BASE_HEIGHT 20
#define ARM_LENGTH 40
#define CUP_RADIUS 15

#define BLACK     0x0000
#define WHITE     0xFFFF
#define RED       0xF800
#define GREEN     0x07E0
#define BLUE      0x001F
#define YELLOW    0xFFE0
#define CYAN      0x07FF
#define MAGENTA   0xF81F
#define ORANGE    0xFD20
#define PURPLE    0x780F
#define PINK      0xF81F
#define GRAY      0x8410
#define BROWN     0xA145
#define GOLD      0xFEA0
#define SILVER    0xC618
#define NAVY      0x000F
#define TEAL      0x0410
#define MAROON    0x8000

volatile float angle = 0;
volatile float windSpeed = 0;
float voltage = 0;
float lastVoltage = -1;
float lastWindSpeed = -1;

int oldX[3], oldY[3];

void setup() {
  Serial.begin(9600);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(BLACK);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.fillRect(5, 10, 320, 30, ST77XX_YELLOW);
  tft.setTextSize(3);
  tft.setTextColor(BLACK);
  tft.setCursor(70, 15);
  tft.print("Anemometer");

  tft.setTextColor(WHITE);
  tft.fillRect(5, 40, 150, 30, ST77XX_BLUE);
  tft.setCursor(20, 50);
  tft.setTextSize(2);
  tft.print("Wind Speed:");
  tft.fillRect(5, 80, 110, 30, ST77XX_BLUE);
  tft.setTextSize(2);
  tft.setCursor(20, 90);
  tft.print("Voltage:");

  drawBaseAndPole();

  Timer1.initialize(500000);
  Timer1.attachInterrupt(readSensor);

  Timer3.initialize(20000);
  Timer3.attachInterrupt(animateCups);
}

void loop() {}

void readSensor() {
  int adc = analogRead(sensorPin);
  voltage = adc * (vRef / 1023.0);
  voltage = constrain(voltage, 1.0, 5.0);

  float effectiveVoltage = voltage - zeroWindVoltage;
  if (effectiveVoltage < 0) effectiveVoltage = 0;

  windSpeed = (effectiveVoltage / (4.0 - (zeroWindVoltage - 1.0))) * fullScale;
  if (windSpeed < 0.3) windSpeed = 0;

  Serial.print("Voltage: ");
  Serial.print(voltage, 2);
  Serial.print(" V | Wind: ");
  Serial.println(windSpeed, 2);

  if (abs(voltage - lastVoltage) > 0.01) {
    char buf[8];
    tft.setTextColor(BLACK);
    tft.setCursor(20, 120);
    dtostrf(lastVoltage - zeroWindVoltage, 4, 2, buf);
    tft.print(buf);
    tft.print(" V");

    tft.setTextColor(RED);
    tft.setCursor(20, 120);
    dtostrf(effectiveVoltage, 4, 2, buf);
    tft.print(buf);
    tft.print(" V");

    lastVoltage = voltage;
  }

  if (abs(windSpeed - lastWindSpeed) > 0.1) {
    char buf[8];
    tft.setTextColor(BLACK);
    tft.setCursor(170, 50);
    dtostrf(lastWindSpeed, 4, 1, buf);
    tft.print(buf);
    tft.print(" m/s");

    tft.setTextColor(RED);
    tft.setCursor(170, 50);
    dtostrf(windSpeed, 4, 1, buf);
    tft.print(buf);
    tft.print(" m/s");

    lastWindSpeed = windSpeed;
  }
}

void animateCups() {
  static float lastAngle = -999;
  float step = windSpeed * 0.3;
  if (step < 0.01) return;

  angle += step;
  if (angle >= 360) angle -= 360;

  if (abs(angle - lastAngle) < 40.0) return;

  for (int i = 0; i < 3; i++) {
    float prevAngle = lastAngle + i * 120;
    float prevRad = radians(prevAngle);
    int px = CENTER_X + cos(prevRad) * ARM_LENGTH;
    int py = CENTER_Y - POLE_HEIGHT / 2 - sin(prevRad) * ARM_LENGTH;

    tft.drawLine(CENTER_X, CENTER_Y - POLE_HEIGHT / 2, px, py, BLACK);
    tft.fillCircle(px, py, CUP_RADIUS + 1, BLACK);
  }

  drawPoleOnly();

  for (int i = 0; i < 3; i++) {
    float newAngle = angle + i * 120;
    float rad = radians(newAngle);
    int x = CENTER_X + cos(rad) * ARM_LENGTH;
    int y = CENTER_Y - POLE_HEIGHT / 2 - sin(rad) * ARM_LENGTH;

    tft.drawLine(CENTER_X, CENTER_Y - POLE_HEIGHT / 2, x, y, WHITE);
    tft.fillCircle(x, y, CUP_RADIUS, PINK);

    oldX[i] = x;
    oldY[i] = y;
  }

  lastAngle = angle;
}

void drawBaseAndPole() {
  tft.fillRoundRect(CENTER_X - BASE_WIDTH / 2, CENTER_Y + POLE_HEIGHT / 2,
                    BASE_WIDTH, BASE_HEIGHT, 5, GRAY);
  drawPoleOnly();
}

void drawPoleOnly() {
  tft.fillRect(CENTER_X - POLE_WIDTH / 2, CENTER_Y - POLE_HEIGHT / 2,
               POLE_WIDTH, POLE_HEIGHT, BROWN);
}
