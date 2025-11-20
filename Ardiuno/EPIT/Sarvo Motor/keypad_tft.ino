#include <Wire.h>
#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9
#define SERVO_PIN 13
#define PCF_ADDR 0x20

char keys[4][4] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Servo myServo;

int currentAngle = 90;
int lastNeedleAngle = -1;
String inputStr = "";


const int CX = 210;
const int CY = 220;
const int R = 100;

const int NEEDLE_TIP_R = R - 32;
const float NEEDLE_HALF_W = 0.045f;
const int INNER_CLEAR_R = R - 26;

const int SERVO_X = 8;
const int SERVO_Y = 170;

int lastAngle = -1;
void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  myServo.attach(SERVO_PIN);
  myServo.write(currentAngle);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  drawStaticUI();
  updateAngleText(currentAngle);
  drawSpeedometerBase();
  drawNeedle(currentAngle);
  drawServoStatic();
  refreshServoHorn(currentAngle);
}

void loop() {
  char key = scanKeypad();
  if (key) {
    if (key == 'C') {  // confirm entered angle
      int angle = inputStr.toInt();
      if (angle >= 0 && angle <= 180) {
        if (angle != currentAngle) {
          moveToAngle(angle);
        }
      }
      inputStr = "";
    } else if (key >= '0' && key <= '9') {
      inputStr += key;
    }

    // Update preview or final display
    if (inputStr.length() > 0) {
      int previewAngle = inputStr.toInt();
      updateAngleText(previewAngle);
    } else {
      updateAngleText(currentAngle);
    }

    delay(200);
  }
}

void moveToAngle(int target) {
  if (target == currentAngle) return;

  int step = (target > currentAngle) ? 1 : -1;
  int diff = abs(target - currentAngle);

  // dynamic step size: larger jumps move faster
  int stepSize = (diff > 90) ? 4 : (diff > 30 ? 2 : 1);

  for (int a = currentAngle; a != target; a += step * stepSize) {
    myServo.write(a);

    clearNeedleArea();
    drawNeedleAt(a);
    refreshServoHorn(a);

    delay(1);  // smaller delay = faster
  }

  // final correction to exact target
  myServo.write(target);
  clearNeedleArea();
  drawNeedleAt(target);
  refreshServoHorn(target);

  currentAngle = target;
}



void drawStaticUI() {
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.fillRect(0, 0, 320, 30, ST77XX_BLUE);
  tft.setCursor(60, 5);
  tft.print("Servo Control");

  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, 60);
  tft.print("Angle:");
}

void updateAngleText(int angle) {
  tft.fillRect(120, 60, 100, 25, ST77XX_BLACK);
  tft.setCursor(120, 60);
  tft.setTextColor(ST77XX_CYAN);
  tft.print(angle);
  tft.print((char)223);
}



void drawNeedle(int angle) {
  if (angle == lastNeedleAngle) return;

  int cx = 210, cy = 220, r = 100;

  if (lastNeedleAngle >= 0) {
    float oldRad = radians(180 - lastNeedleAngle);
    int ox = cx + (r - 20) * cos(oldRad);
    int oy = cy - (r - 20) * sin(oldRad);
    int ox2 = cx + (r - 60) * cos(oldRad + 0.04);
    int oy2 = cy - (r - 60) * sin(oldRad + 0.04);
    int ox3 = cx + (r - 60) * cos(oldRad - 0.04);
    int oy3 = cy - (r - 60) * sin(oldRad - 0.04);
    tft.fillTriangle(cx, cy, ox2, oy2, ox, oy, ST77XX_BLACK);
    tft.fillTriangle(cx, cy, ox3, oy3, ox, oy, ST77XX_BLACK);
    tft.fillCircle(cx, cy, 10, ST77XX_WHITE);
    tft.fillCircle(cx, cy, 4, ST77XX_BLACK);
  }

  float rad = radians(180 - angle);
  int nx = cx + (r - 20) * cos(rad);
  int ny = cy - (r - 20) * sin(rad);
  int nx2 = cx + (r - 60) * cos(rad + 0.04);
  int ny2 = cy - (r - 60) * sin(rad + 0.04);
  int nx3 = cx + (r - 60) * cos(rad - 0.04);
  int ny3 = cy - (r - 60) * sin(rad - 0.04);

  tft.fillTriangle(cx, cy, nx2, ny2, nx, ny, ST77XX_RED);
  tft.fillTriangle(cx, cy, nx3, ny3, nx, ny, ST77XX_RED);
  tft.fillCircle(cx, cy, 10, ST77XX_WHITE);
  tft.fillCircle(cx, cy, 4, ST77XX_BLACK);

  tft.fillRect(cx - 30, cy + 30, 60, 25, ST77XX_BLACK);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setCursor(cx - 22, cy + 30);
  tft.print(angle);
  tft.print((char)223);

  lastNeedleAngle = angle;
}

char scanKeypad() {
  for (int row = 0; row < 4; row++) {
    byte out = 0xFF;
    out &= ~(1 << (row + 4));
    writePCF(out);
    delay(2);
    byte in = readPCF();
    for (int col = 0; col < 4; col++) {
      if ((in & (1 << col)) == 0) {
        return keys[row][col];
      }
    }
  }
  return 0;
}

void writePCF(byte val) {
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(val);
  Wire.endTransmission();
}

byte readPCF() {
  Wire.requestFrom(PCF_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}

static inline float deg2rad(float d) {
  return d * (PI / 180.0f);
}

uint16_t colorFromAngle(int a) {
  a = constrain(a, 0, 180);
  uint8_t r = 0, g = 0;
  if (a <= 120) {
    r = map(a, 0, 120, 0, 255);
    g = map(a, 0, 120, 255, 128);
  } else {
    r = 255;
    g = map(a, 120, 180, 128, 0);
  }
  return tft.color565(r, g, 0);
}

// ----------------- Gauge Drawing -----------------
void drawColoredArc() {
  for (int a = 0; a <= 180; a++) {
    float rad = deg2rad(180 - a);
    uint16_t col = colorFromAngle(a);
    for (int t = 0; t < 10; t++) {
      int x = CX + (R - t) * cos(rad);
      int y = CY - (R - t) * sin(rad);
      tft.drawPixel(x, y, col);
    }
  }
}

void drawTicksAndLabels() {
  for (int a = 0; a <= 180; a += 10) {
    float rad = deg2rad(180 - a);
    int inner = (a % 30 == 0) ? (R - 10) : (R - 7);
    int outer = R;
    tft.drawLine(CX + inner * cos(rad), CY - inner * sin(rad),
                 CX + outer * cos(rad), CY - outer * sin(rad), ST77XX_WHITE);
  }
  for (int a = 0; a <= 180; a += 30) {
    float rad = deg2rad(180 - a);
    int lx = CX + (R - 18) * cos(rad);
    int ly = CY - (R - 18) * sin(rad);
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

// ----------------- Needle -----------------
void clearNeedleArea() {
  tft.fillCircle(CX, CY, INNER_CLEAR_R, ST77XX_BLACK);
  drawGaugeHub();
}

void drawNeedleAt(int angle) {
  float rad = deg2rad(180 - angle);
  uint16_t ncol = colorFromAngle(angle);

  int nx = CX + NEEDLE_TIP_R * cos(rad);
  int ny = CY - NEEDLE_TIP_R * sin(rad);

  int bx = CX - 10 * cos(rad);
  int by = CY + 10 * sin(rad);

  int dx = (int)(NEEDLE_HALF_W * R * sin(rad));
  int dy = (int)(NEEDLE_HALF_W * R * cos(rad));

  tft.fillTriangle(bx - dx, by - dy, bx + dx, by + dy, nx, ny, ncol);
  tft.fillTriangle(nx, ny, bx - dx, by - dy, bx + dx, by + dy, ncol);
}

// ----------------- Servo Icon + Horn -----------------
void drawServoStatic() {
  tft.fillRoundRect(SERVO_X, SERVO_Y, 90, 60, 8, ST77XX_BLUE);
  tft.drawRoundRect(SERVO_X, SERVO_Y, 90, 60, 8, ST77XX_WHITE);
  tft.fillRoundRect(SERVO_X + 18, SERVO_Y - 16, 54, 20, 6, ST77XX_WHITE);

  // MG995 label
  tft.fillRect(SERVO_X + 5, SERVO_Y + 35, 80, 20, ST77XX_WHITE);  // white box
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(SERVO_X + 15, SERVO_Y + 38);
  tft.print("MG995");

  int px = SERVO_X + 45, py = SERVO_Y - 6;
  tft.fillCircle(px, py, 5, ST77XX_WHITE);
  tft.fillCircle(px, py, 2, ST77XX_BLACK);
}

void refreshServoHorn(int angle) {
  int px = SERVO_X + 45, py = SERVO_Y - 6;
  tft.fillCircle(px, py, 34, ST77XX_BLACK);
  drawServoStatic();
  int hornA = map(angle, 0, 180, -90, 90);
  float nrad = deg2rad(hornA);
  int hx = px + 28 * cos(nrad);
  int hy = py - 28 * sin(nrad);
  tft.drawLine(px, py, hx, hy, ST77XX_RED);
  tft.fillCircle(hx, hy, 4, ST77XX_RED);
  tft.fillCircle(px, py, 5, ST77XX_WHITE);
  tft.fillCircle(px, py, 2, ST77XX_BLACK);
}