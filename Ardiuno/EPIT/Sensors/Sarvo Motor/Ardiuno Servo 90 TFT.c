#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <math.h>

// TFT pin definitions
#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9

// Servo and control pins
#define SERVO_PIN 13
#define POT_PIN A8  // Ensure you're on Arduino Mega (A8 exists). Uno has only A0..A5.

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Servo myServo;

// Gauge center and size (landscape 320x240)
const int CX = 210;
const int CY = 220;
const int R = 100;  // outer radius

// Needle geometry (kept inside to avoid redrawing labels/arc)
const int NEEDLE_TIP_R = R - 32;     // tip length (kept away from labels/arc)
const int NEEDLE_SIDE_R = R - 60;    // triangle base length
const float NEEDLE_HALF_W = 0.045f;  // radians (~2.6°)

// Inner clear radius: clears any previous needle fully, never touching labels (at R-18)
const int INNER_CLEAR_R = R - 26;  // 74 when R=100

// Servo icon placement
const int SERVO_X = 8;    // icon left
const int SERVO_Y = 170;  // icon top

// State
int lastAngle = -1;

// ----------------- Utility -----------------
static inline float deg2rad(float d) {
  return d * (PI / 180.0f);
}

// Angle-based color: green -> yellow/orange -> red
uint16_t colorFromAngle(int a) {
  a = constrain(a, 0, 180);
  uint8_t r = 0, g = 0, b = 0;
  if (a <= 120) {
    r = map(a, 0, 120, 0, 255);
    g = map(a, 0, 120, 255, 128);
    b = 0;
  } else {
    r = 255;
    g = map(a, 120, 180, 128, 0);
    b = 0;
  }
  return tft.color565(r, g, b);
}

// ----------------- UI -----------------
void initBacklight() {
  pinMode(TFT_BL, OUTPUT);
  // Use simple ON to avoid timer conflicts with Servo on some boards
  digitalWrite(TFT_BL, HIGH);
}

void drawTitle() {
  tft.fillRect(0, 0, 320, 30, ST77XX_BLUE);
  tft.setCursor(60, 5);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print("Servo Control");

  // Angle label
  tft.setCursor(10, 50);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.print("Angle:");
}

void updateAngleText(int angle) {
  // Clear area and rewrite
  tft.fillRect(110, 50, 100, 25, ST77XX_BLACK);
  tft.setCursor(110, 50);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.print(angle);
  tft.print(" deg");  // degree symbol
}

void drawColoredArc() {
  const int thickness = 10;  // pixels
  for (int a = 0; a <= 180; a++) {
    float rad = deg2rad(180 - a);
    uint16_t col = colorFromAngle(a);
    for (int t = 0; t < thickness; t++) {
      int x = CX + (R - t) * cos(rad);
      int y = CY - (R - t) * sin(rad);
      tft.drawPixel(x, y, col);
    }
  }
}

void drawTicksAndLabels() {
  // Minor ticks every 10°, major ticks every 30°
  for (int a = 0; a <= 180; a += 10) {
    float rad = deg2rad(180 - a);
    int inner = (a % 30 == 0) ? (R - 10) : (R - 7);
    int outer = R;
    int x0 = CX + inner * cos(rad);
    int y0 = CY - inner * sin(rad);
    int x1 = CX + outer * cos(rad);
    int y1 = CY - outer * sin(rad);
    tft.drawLine(x0, y0, x1, y1, ST77XX_WHITE);
  }
  for (int a = 0; a <= 180; a += 30) {
    float rad = deg2rad(180 - a);
    int lr = R - 18;  // label radius (outside needle tip)
    int lx = CX + lr * cos(rad);
    int ly = CY - lr * sin(rad);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(1);
    tft.setCursor(lx - 8, ly - 6);
    tft.print(a);
  }
}

void drawGaugeHub() {
  tft.fillCircle(CX, CY, 10, ST77XX_WHITE);
  tft.fillCircle(CX, CY, 4, ST77XX_BLACK);
}

void drawSpeedometerBase() {
  drawColoredArc();
  drawTicksAndLabels();
  drawGaugeHub();
}

// ----------------- Needle (trace-free) -----------------
void clearNeedleArea() {
  // Clear inner area where the needle can exist; never touches arc/labels
  tft.fillCircle(CX, CY, INNER_CLEAR_R, ST77XX_BLACK);
  drawGaugeHub();
}

void drawNeedleAt(int angle) {
  float rad = deg2rad(180 - angle);
  uint16_t ncol = colorFromAngle(angle);

  int nx = CX + NEEDLE_TIP_R * cos(rad);
  int ny = CY - NEEDLE_TIP_R * sin(rad);

  int bx = CX - 10 * cos(rad);  // back of needle
  int by = CY + 10 * sin(rad);

  // Offset for width
  int dx = (int)(NEEDLE_HALF_W * R * sin(rad));
  int dy = (int)(NEEDLE_HALF_W * R * cos(rad));

  // Rectangle needle (4 points)
  tft.fillTriangle(bx - dx, by - dy, bx + dx, by + dy, nx, ny, ncol);
  tft.fillTriangle(nx, ny, bx - dx, by - dy, bx + dx, by + dy, ncol);
}


// ----------------- Servo Icon (left) -----------------
void drawServoStatic() {
  uint16_t bodyCol = ST77XX_BLUE;  // blue body
  uint16_t rimCol = ST77XX_WHITE;

  // Main servo body
  tft.fillRoundRect(SERVO_X, SERVO_Y, 90, 60, 8, bodyCol);
  tft.drawRoundRect(SERVO_X, SERVO_Y, 90, 60, 8, rimCol);
  tft.fillRoundRect(SERVO_X + 18, SERVO_Y - 16, 54, 20, 6, rimCol);

  // Screw holes
  tft.fillCircle(SERVO_X + 10, SERVO_Y + 10, 3, ST77XX_BLACK);
  tft.fillCircle(SERVO_X + 80, SERVO_Y + 50, 3, ST77XX_BLACK);

  // Horn pivot
  int px = SERVO_X + 45;
  int py = SERVO_Y - 6;
  tft.fillCircle(px, py, 5, ST77XX_WHITE);
  tft.fillCircle(px, py, 2, ST77XX_BLACK);

  // Label box "MG995"
  int labelX = SERVO_X + 10;  // inside the blue body
  int labelY = SERVO_Y + 30;
  tft.fillRect(labelX, labelY, 70, 20, ST77XX_WHITE);  // white box
  tft.drawRect(labelX, labelY, 70, 20, ST77XX_BLACK);  // black border
  tft.setCursor(labelX + 6, labelY + 4);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_BLACK);
  tft.print("MG995");
}


void refreshServoHorn(int angle) {
  // Clear around horn pivot so no traces remain
  int px = SERVO_X + 45;
  int py = SERVO_Y - 6;
  tft.fillCircle(px, py, 34, ST77XX_BLACK);
  drawServoStatic();

  // Map 0..180 to -90..+90 for horn rotation
  int hornA = map(angle, 0, 180, -90, 90);
  uint16_t hornCol = ST77XX_RED;

  float nrad = deg2rad(hornA);
  int hx = px + 28 * cos(nrad);
  int hy = py - 28 * sin(nrad);

  // Thick horn (3px)
  tft.drawLine(px, py, hx, hy, hornCol);
  tft.drawLine(px + 2, py, hx + 2, hy, hornCol);
  tft.drawLine(px - 2, py, hx - 2, hy, hornCol);
  tft.fillCircle(hx, hy, 4, hornCol);

  // Restore pivot
  tft.fillCircle(px, py, 5, ST77XX_WHITE);
  tft.fillCircle(px, py, 2, ST77XX_BLACK);
}

// ----------------- POT read (stable) -----------------
int readAngleFromPot() {
  // Average 8 samples, then map; reduces jitter and random motion
  long sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(POT_PIN);
    delayMicroseconds(200);
  }
  int avg = sum / 8;  // 0..1023
  int angle = map(avg, 0, 1023, 0, 180);
  return constrain(angle, 0, 180);
}

// ----------------- Arduino -----------------
void setup() {
  initBacklight();

  myServo.attach(SERVO_PIN);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  drawTitle();
  drawSpeedometerBase();  // Static gauge (colored semicircle, ticks, hub)
  drawServoStatic();      // Static servo body

  // Initial render from current pot (no auto-sweep)
  int angle = readAngleFromPot();
  drawNeedleAt(angle);
  refreshServoHorn(angle);
  updateAngleText(angle);
  myServo.write(angle);
  lastAngle = angle;
}

void loop() {
  int angle = readAngleFromPot();

  // Deadband to avoid flicker/jitter
  if (abs(angle - lastAngle) >= 2) {
    // Only update what changed; no traces, no blink
    clearNeedleArea();
    drawNeedleAt(angle);
    refreshServoHorn(angle);
    updateAngleText(angle);
    myServo.write(angle);
    lastAngle = angle;
  }

  delay(15);
}
