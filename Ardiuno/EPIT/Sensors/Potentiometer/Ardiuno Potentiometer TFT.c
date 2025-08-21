#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <math.h>

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Potentiometer
const int potPin = A8;

// Gauge geometry
const int CX = 120;
const int CY = 220;
const int R_ARC = 100;    // outer arc radius (scale)
const int R_NEEDLE = 80;  // needle length
const int ARC_THICK = 10;
float breathPhase = 0;
// Animation state
int potValue = 0;
int speedValue = 0;  // 0..100
int lastSpeedValue = -1;

float displayedAngleDeg = -180.0f;  // eased needle position
float targetAngleDeg = -180.0f;
float lastNeedleAngleDeg = -1000.0f;

bool didIntroSweep = false;

// Center pulse
int prevPulseR = -1;

// Rotating dots (flicker-free with wrap)
const int N_DOTS = 28;
float dotPhase = 0.0f;  // 0..180 mapped along arc
bool prevDotsValid = false;
int prevDotsX[N_DOTS], prevDotsY[N_DOTS];

// Highlighted major tick (0,20,...,100)
int prevHighlightVal = -1;

// ===== Utilities =====
static inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
static inline float speedToAngle(float s) {  // 0..100 -> -180..0
  return mapFloat(s, 0.0f, 100.0f, -180.0f, 0.0f);
}
static inline void polarToXY(int cx, int cy, float deg, float r, int &x, int &y) {
  float rad = radians(deg);
  x = cx + (int)(cos(rad) * r);
  y = cy + (int)(sin(rad) * r);
}
static inline float wrapSemi(float a) {  // wrap to [-180,0)
  while (a < -180.0f) a += 180.0f;
  while (a >= 0.0f) a -= 180.0f;
  return a;
}

// ===== Drawing Primitives =====
void drawArcThick(int cx, int cy, int r, int thickness, float aStart, float aEnd, uint16_t color) {
  int r1 = r - thickness / 2;
  int r2 = r + thickness / 2;
  for (int a = (int)aStart; a <= (int)aEnd; a++) {
    int x1, y1, x2, y2;
    polarToXY(cx, cy, a, r1, x1, y1);
    polarToXY(cx, cy, a, r2, x2, y2);
    tft.drawLine(x1, y1, x2, y2, color);
  }
}

void drawMajorTick(int value, uint16_t color) {  // value: 0..100 step 20
  float ang = speedToAngle(value);
  int x1, y1, x2, y2;
  polarToXY(CX, CY, ang, R_ARC - 10, x1, y1);
  polarToXY(CX, CY, ang, R_ARC, x2, y2);
  tft.drawLine(x1, y1, x2, y2, color);
}

void drawMinorTick(float value, uint16_t color) {  // value: 0..100
  float ang = speedToAngle(value);
  int x1, y1, x2, y2;
  polarToXY(CX, CY, ang, R_ARC - 6, x1, y1);
  polarToXY(CX, CY, ang, R_ARC, x2, y2);
  tft.drawLine(x1, y1, x2, y2, color);
}

void drawNumberAt(int value) {
  float ang = speedToAngle(value);
  int lx, ly;
  polarToXY(CX, CY, ang, R_ARC - 30, lx, ly);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);  // no-flicker background
  tft.setTextSize(2);
  tft.setCursor(lx - 10, ly - 10);
  tft.print(value);
}

// ===== Needle (knife/tapered) =====
void drawNeedleKnife(float angleDeg, uint16_t color) {
  // Knife shape: thin base -> widest mid -> pointed tip
  float rad = radians(angleDeg);
  float dx = cos(rad), dy = sin(rad);
  float nx = -dy, ny = dx;  // normal

  // Widths
  const float w0 = 3.0f;              // base width
  const float w1 = 11.0f;             // mid width
  const float L0 = 6.0f;              // base length offset from center
  const float L1 = R_NEEDLE * 0.60f;  // mid position
  const float L2 = R_NEEDLE;          // tip

  // Base points (near center)
  int bLx = CX + (int)(dx * L0 + nx * (-w0 / 2));
  int bLy = CY + (int)(dy * L0 + ny * (-w0 / 2));
  int bRx = CX + (int)(dx * L0 + nx * (w0 / 2));
  int bRy = CY + (int)(dy * L0 + ny * (w0 / 2));

  // Mid points (widest)
  int mLx = CX + (int)(dx * L1 + nx * (-w1 / 2));
  int mLy = CY + (int)(dy * L1 + ny * (-w1 / 2));
  int mRx = CX + (int)(dx * L1 + nx * (w1 / 2));
  int mRy = CY + (int)(dy * L1 + ny * (w1 / 2));

  // Tip point
  int tx = CX + (int)(dx * L2);
  int ty = CY + (int)(dy * L2);

  // Build with two triangles: base-diamond -> tip
  tft.fillTriangle(bLx, bLy, bRx, bRy, mLx, mLy, color);
  tft.fillTriangle(bRx, bRy, mRx, mRy, mLx, mLy, color);
  tft.fillTriangle(mLx, mLy, mRx, mRy, tx, ty, color);
}

// ===== Static Layout =====
void drawHeading() {
  tft.fillRoundRect(10, 10, 300, 30, 8, ST77XX_YELLOW);
  tft.setTextWrap(false);
  tft.setCursor(50, 18);
  tft.setTextColor(ST77XX_BLACK, ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.print("Potentiometer");

  // reserve area for upper text
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(20, 55);
  tft.print("Speed = 0  ");
}

void drawGaugeBackplate() {
  // Colored zones: Green 0-60, Yellow 60-85, Red 85-100
  drawArcThick(CX, CY, R_ARC, ARC_THICK, -180, speedToAngle(60), ST77XX_GREEN);
  drawArcThick(CX, CY, R_ARC, ARC_THICK, speedToAngle(60), speedToAngle(85), ST77XX_YELLOW);
  drawArcThick(CX, CY, R_ARC, ARC_THICK, speedToAngle(85), 0, ST77XX_RED);

  // Outer border (thin white ring)
  for (int a = -180; a <= 0; a++) {
    int x, y;
    polarToXY(CX, CY, a, R_ARC + ARC_THICK / 2 + 1, x, y);
    tft.drawPixel(x, y, ST77XX_WHITE);
  }

  // Major ticks + numbers (0..100 step 20)
  for (int i = 0; i <= 100; i += 20) {
    drawMajorTick(i, ST77XX_WHITE);
    drawNumberAt(i);
  }
  // Minor ticks every 10
  for (int i = 10; i < 100; i += 10) {
    drawMinorTick(i, ST77XX_YELLOW);
  }

  // Center hub ring
  tft.drawCircle(CX, CY, 10, ST77XX_WHITE);
  tft.drawCircle(CX, CY, 12, ST77XX_BLUE);

  // Bottom digital readout area (printed with background color, no clear)
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setTextSize(3);
  tft.setCursor(100, 265);
  tft.print(0);
  tft.setTextSize(2);
  tft.print(" km/h");
}

void redrawTicksAndNumbers() {
  // Redraw only dynamic-overwritten items (no arcs to save time)
  for (int i = 0; i <= 100; i += 20) {
    drawMajorTick(i, (i == prevHighlightVal) ? ST77XX_CYAN : ST77XX_WHITE);
    drawNumberAt(i);
  }
  for (int i = 10; i < 100; i += 10) {
    drawMinorTick(i, ST77XX_YELLOW);
  }
  // Re-draw center rings (they get cleared by center pulse erase)
  tft.drawCircle(CX, CY, 10, ST77XX_WHITE);
  tft.drawCircle(CX, CY, 12, ST77XX_BLUE);
}

// ===== Dynamic Elements =====
void updateSpeedUpperText(int speed) {
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);  // background to avoid flicker
  tft.setTextSize(2);
  tft.setCursor(20, 55);
  tft.print("Speed = ");
  tft.print(speed);
  tft.print("   ");  // pad to clear leftovers if digits shrink
}

void updateBottomDigital(int speed) {
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);  // background to avoid flicker
  tft.setTextSize(3);
  tft.setCursor(100, 265);
  tft.print(speed);
  tft.print("  ");  // pad
  tft.setTextSize(2);
  tft.print(" km/h");
}

void highlightMajorTick(int value) {
  // restore previous highlight
  if (prevHighlightVal >= 0) {
    drawMajorTick(prevHighlightVal, ST77XX_WHITE);
  }
  drawMajorTick(value, ST77XX_CYAN);
  prevHighlightVal = value;
}

void animateCenterPulse() {
  // pulse radius between 6..9
  float s = (sin(millis() * 0.006f) + 1.0f) * 0.5f;  // 0..1
  int r = 6 + (int)(s * 3);

  // erase previous pulse (only inner fill, not rings)
  if (prevPulseR >= 0) {
    tft.fillCircle(CX, CY, prevPulseR, ST77XX_BLACK);
  }

  // draw new pulse
  tft.fillCircle(CX, CY, r, ST77XX_BLUE);

  // keep static hub rings (draw once in setup/backplate, not every frame)
  prevPulseR = r;
}





void drawNeedleEased(float targetDeg) {
  // Update smoothing only if changed
  float delta = targetDeg - displayedAngleDeg;
  if (fabs(delta) > 0.1f) {
    displayedAngleDeg += delta * 0.50f;  // smoothing
    if (fabs(delta) < 0.5f) displayedAngleDeg = targetDeg;

    // Erase old needle
    if (lastNeedleAngleDeg > -900.0f) {
      drawNeedleKnife(lastNeedleAngleDeg, ST77XX_BLACK);
      redrawTicksAndNumbers();
    }

    // Highlight nearest major tick
    int major = (int)round(speedValue / 20.0f) * 20;
    if (major < 0) major = 0;
    if (major > 100) major = 100;
    highlightMajorTick(major);

    // Draw new needle
    drawNeedleKnife(displayedAngleDeg, ST77XX_RED);
    tft.fillCircle(CX, CY, 5, ST77XX_WHITE);

    lastNeedleAngleDeg = displayedAngleDeg;
  }
}


// Optional: startup sweep
void introSweepOnce() {
  if (didIntroSweep) return;
  didIntroSweep = true;

  int step = 12;  // bigger step = faster sweep, but less smooth
  int wait = 1;   // smaller delay = faster sweep

  // Sweep forward
  for (int a = -180; a <= 0; a += step) {
    if (lastNeedleAngleDeg > -900.0f) drawNeedleKnife(lastNeedleAngleDeg, ST77XX_BLACK);
    redrawTicksAndNumbers();
    drawNeedleKnife(a, ST77XX_RED);
    tft.fillCircle(CX, CY, 5, ST77XX_WHITE);
    lastNeedleAngleDeg = a;
    delay(wait);
  }

  // Sweep backward
  for (int a = 0; a >= -180; a -= step) {
    drawNeedleKnife(lastNeedleAngleDeg, ST77XX_BLACK);
    redrawTicksAndNumbers();
    drawNeedleKnife(a, ST77XX_RED);
    tft.fillCircle(CX, CY, 5, ST77XX_WHITE);
    lastNeedleAngleDeg = a;
    delay(wait);
  }
}

// ===== Arduino setup/loop =====
void setup() {
  Serial.begin(9600);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);

  drawHeading();
  drawGaugeBackplate();
}

void loop() {
  // Read and map speed
  potValue = analogRead(potPin);
  speedValue = map(potValue, 0, 1023, 0, 100);

  if (!didIntroSweep) {
    introSweepOnce();
  }

  // Update target angle
  targetAngleDeg = speedToAngle(speedValue);

  // Visual updates
  drawNeedleEased(targetAngleDeg);  // handles erase + restore without killing numbers
  animateCenterPulse();             // pulse with no flicker
                                    // semicircle wrap, continuous, low flicker

  // Update texts only on change, with background color (no blinking)
  if (speedValue != lastSpeedValue) {
    updateSpeedUpperText(speedValue);
    updateBottomDigital(speedValue);
    lastSpeedValue = speedValue;
  }

  delay(25);  // smooth
}

// ===== Bulb drawing =====
// ===== Bulb drawing =====
