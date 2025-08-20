#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <math.h>

// ==== TFT Pins ====
#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9

// ==== Limit Switch ====
#define SWITCH_PIN 47
bool switchPressed = false;
unsigned long pressStartTime = 0;
float holdTime = 0;

// ==== Arm Animation ====
float g_forearmAngleDeg = 0;
float g_handOffsetDeg = 0;
float fingerSwing = 0;  // How much fingers are spread
float fingerSpeed = 0.004;
float prevElbowAngle = -120;
float elbowSpeed = 0.001;

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ==== Joint Positions ====
int shoulderX, shoulderY;
int elbowX, elbowY;
int wristX, wristY;
int prevFingerOffsetPx = 0;  // previous frame's finger spread in pixels

// ==== Base Orientation ====
#define BASE_ORIENTATION_SIDE 1
#define BASE_ORIENTATION_BOTTOM 2
#define BASE_ORIENTATION BASE_ORIENTATION_SIDE

bool animationStopped = false;
bool brokenHandShown = false;
unsigned long brokenStartTime = 0;

// ==== Function Prototypes ====
void drawBaseOnly();
void drawShoulderArm(float angleDeg);
void drawForearm(float angleDeg, uint16_t color, bool inflate = false);
void drawWristHand(float angleDeg, uint16_t color, bool inflate = false, int fingerOffsetPx = 0);
void drawElbowJoint();
void drawWristJoint(bool erase = false);
void clearForearmHand();
void drawBrokenHand();
float startAngle = -170;
float endAngle = -80;
float swing = (sin(millis() * elbowSpeed) + 1) / 2;
float totalElbow = startAngle + swing * (endAngle - startAngle);
void setup() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  pinMode(SWITCH_PIN, INPUT);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  // Draw static parts
  drawBaseOnly();
  drawShoulderArm(35);

  float elbowOffset = -70;
  drawForearm(35 + elbowOffset, ST77XX_YELLOW);
  int initFingerOffsetPx = (int)round(fingerSwing * 3.0);
  prevFingerOffsetPx = initFingerOffsetPx;
  drawWristHand(35 + elbowOffset, ST77XX_WHITE, false, initFingerOffsetPx);

  drawElbowJoint();
  drawWristJoint(false);
}

void loop() {
  // ==== Read limit switch ====
  bool currentState = !digitalRead(SWITCH_PIN);  // active LOW
  if (currentState && !switchPressed) {
    switchPressed = true;
    pressStartTime = millis();
  } else if (!currentState && switchPressed) {
    switchPressed = false;
    holdTime = (millis() - pressStartTime) / 1000.0;
  }

  // ==== Broken hand logic ====
  if (!brokenHandShown && totalElbow > -90) {  // trigger when it reaches limit
    animationStopped = true;
    brokenHandShown = true;
    brokenStartTime = millis();
    drawBrokenHand();  // show broken hand
  }

  // Resume animation after 2 seconds of broken-hand
  if (brokenHandShown && millis() - brokenStartTime > 2000) {
    brokenHandShown = false;
    animationStopped = false;
    prevElbowAngle = -120;  // reset animation to start
  }

  // ==== Animate only if not stopped or broken ====
  if (!animationStopped && !switchPressed) {
    float startAngle = -170;
    float endAngle = -80;
    float swing = (sin(millis() * elbowSpeed) + 1) / 2;
    float totalElbow = startAngle + swing * (endAngle - startAngle);

    // Finger animation
    fingerSwing = (sin(millis() * fingerSpeed) + 1) / 2;
    int newFingerOffsetPx = (int)round(fingerSwing * 3.0);

    // Erase previous frame
    drawForearm(70 + prevElbowAngle, ST77XX_BLACK, true);
    drawWristHand(70 + prevElbowAngle, ST77XX_BLACK, true, prevFingerOffsetPx);
    drawWristJoint(true);

    // Draw new frame
    drawForearm(70 + totalElbow, ST77XX_YELLOW, false);
    drawWristHand(70 + totalElbow, ST77XX_WHITE, false, newFingerOffsetPx);
    drawElbowJoint();
    drawWristJoint(false);

    prevElbowAngle = totalElbow;
    prevFingerOffsetPx = newFingerOffsetPx;
  }

  delay(50);
}
void drawBrokenHand() {
  // Draw a red box as placeholder
  tft.fillRect(120, 20, 100, 60, ST77XX_RED);

  // Set text parameters
  tft.setTextSize(2);              // make text readable
  tft.setTextColor(ST77XX_WHITE);  // white text
  tft.setCursor(130, 40);          // position inside the red box
  tft.print("BROKEN!");
}




void drawElbowJoint() {
  int elbowR = 12;
  tft.fillCircle(elbowX, elbowY, elbowR, ST77XX_BLACK);
  tft.drawCircle(elbowX, elbowY, elbowR, ST77XX_WHITE);
  tft.drawCircle(elbowX, elbowY, elbowR - 2, ST77XX_GREEN);
}

void drawWristJoint(bool erase = false) {
  int wristR = 8;
  if (erase) {
    tft.fillCircle(wristX, wristY, wristR + 2, ST77XX_BLACK);
    return;
  }
  tft.fillCircle(wristX, wristY, wristR, ST77XX_BLACK);
  tft.drawCircle(wristX, wristY, wristR, ST77XX_WHITE);
  tft.drawCircle(wristX, wristY, wristR - 2, ST77XX_RED);
}




void clearForearmHand() {
  int padding = 10;  // extra space to fully erase lines/fingers
  tft.fillRect(elbowX - 20, elbowY - 20, wristX - elbowX + 60, wristY - elbowY + 60, ST77XX_BLACK);
}
// ==== Drawing Base ====
void drawBaseOnly() {
  tft.fillScreen(ST77XX_BLACK);

  if (BASE_ORIENTATION == BASE_ORIENTATION_SIDE) {
    int pillarX = 8, pillarY = 130, pillarW = 22, pillarH = 100;
    tft.fillRoundRect(pillarX, pillarY, pillarW, pillarH, 6, ST77XX_BLUE);
    tft.drawRoundRect(pillarX, pillarY, pillarW, pillarH, 6, ST77XX_WHITE);

    int plateX = pillarX + pillarW;
    int plateY = 140;
    int plateW = 10, plateH = 80;
    tft.fillRoundRect(plateX, plateY, plateW, plateH, 8, ST77XX_CYAN);
    tft.drawRoundRect(plateX, plateY, plateW, plateH, 8, ST77XX_WHITE);

    int shoulderR = 12;
    shoulderX = plateX + plateW + shoulderR;
    shoulderY = plateY + plateH / 2;

    // Shoulder joint (ring)
    tft.drawCircle(shoulderX, shoulderY, shoulderR, ST77XX_WHITE);
    tft.drawCircle(shoulderX, shoulderY, shoulderR - 2, ST77XX_BLUE);

  } else {
    int baseX = 20, baseY = 230, baseW = 280, baseH = 24;
    tft.fillRoundRect(baseX, baseY, baseW, baseH, 8, ST77XX_BLUE);
    tft.drawRoundRect(baseX, baseY, baseW, baseH, 8, ST77XX_WHITE);

    int pedW = 40, pedH = 30;
    int pedX = baseX + 80;
    int pedY = baseY - pedH;
    tft.fillRoundRect(pedX, pedY, pedW, pedH, 6, ST77XX_CYAN);
    tft.drawRoundRect(pedX, pedY, pedW, pedH, 6, ST77XX_WHITE);

    shoulderX = pedX + pedW / 2;
    shoulderY = pedY;

    tft.drawCircle(shoulderX, shoulderY, 12, ST77XX_WHITE);
    tft.drawCircle(shoulderX, shoulderY, 10, ST77XX_BLUE);
  }
}

// ==== Shoulder + Upper Arm + Elbow ====
void drawShoulderArm(float angleDeg) {
  int armLen = 80;
  int armWidth = 16;
  float theta = radians(angleDeg);

  int shoulderR = 12;
  int elbowR = 12;

  elbowX = shoulderX + armLen * cos(theta);
  elbowY = shoulderY + armLen * sin(theta);

  int startX = shoulderX + shoulderR * cos(theta);
  int startY = shoulderY + shoulderR * sin(theta);
  int endX = elbowX - elbowR * cos(theta);
  int endY = elbowY - elbowR * sin(theta);

  for (int i = -armWidth / 2; i <= armWidth / 2; i++) {
    tft.drawLine(startX, startY + i, endX, endY + i, ST77XX_CYAN);
  }

  tft.fillCircle(shoulderX, shoulderY, shoulderR, ST77XX_BLACK);
  tft.drawCircle(shoulderX, shoulderY, shoulderR, ST77XX_WHITE);
  tft.drawCircle(shoulderX, shoulderY, shoulderR - 2, ST77XX_BLUE);

  tft.fillCircle(elbowX, elbowY, elbowR, ST77XX_BLACK);
  tft.drawCircle(elbowX, elbowY, elbowR, ST77XX_WHITE);
  tft.drawCircle(elbowX, elbowY, elbowR - 2, ST77XX_GREEN);
}

// ==== Forearm ====
void drawForearm(float angleDeg, uint16_t color, bool inflate = false) {
  int foreLen = 60;
  int foreWidth = 14;
  int pad = inflate ? 2 : 0;  // inflate width when erasing to cover rounding artifacts

  float theta = radians(angleDeg);

  int elbowR = 12;
  int wristR = 8;

  // Update wrist position
  wristX = elbowX + foreLen * cos(theta);
  wristY = elbowY + foreLen * sin(theta);

  int startX = elbowX + elbowR * cos(theta);
  int startY = elbowY + elbowR * sin(theta);
  int endX = wristX - wristR * cos(theta);
  int endY = wristY - wristR * sin(theta);

  // Thicker when erasing to cover leftovers
  for (int i = -(foreWidth / 2 + pad); i <= (foreWidth / 2 + pad); i++) {
    int offsetX = i * sin(theta);
    int offsetY = -i * cos(theta);
    tft.drawLine(startX + offsetX, startY + offsetY, endX + offsetX, endY + offsetY, color);
  }
}



// ==== Hand + Palm + Fingers ====
void drawWristHand(float angleDeg, uint16_t color, bool inflate, int fingerOffsetPx) {
  int palmW = 36;
  int palmH = 40;
  int pad = inflate ? 3 : 0;  // increase to 4 if any tiny specks remain

  float theta = radians(angleDeg);

  int handOffsetX = 23;
  int handOffsetY = 0;
  int palmX = wristX + handOffsetX;
  int palmY = wristY + handOffsetY;

  // Palm (inflate when erasing)
  tft.fillRoundRect(palmX - (palmW / 2) - pad, palmY - (palmH / 2) - pad,
                    palmW + 2 * pad, palmH + 2 * pad, 6 + pad, color);

  int fingerLen = 14;
  int fingerW = 6;
  int gap = 4;

  // Thumb (use fingerOffsetPx, not fingerSwing)
  int thumbX = palmX - fingerW / 2 - fingerOffsetPx - pad;
  int thumbY = palmY - palmH / 2 - fingerLen - pad;
  tft.fillRoundRect(thumbX, thumbY, fingerW + 2 * pad, fingerLen + 2 * pad, 3 + pad, color);
  tft.fillRoundRect(thumbX, thumbY - fingerLen - pad, fingerW + 2 * pad, fingerLen + 2 * pad, 3 + pad, color);

  // Other 4 fingers (use fingerOffsetPx)
  for (int f = 0; f < 4; f++) {
    int fx = palmX + palmW / 2 + fingerOffsetPx - pad;
    int fy = palmY - palmH / 2 + 4 + f * (fingerW + gap) - pad;

    tft.fillRoundRect(fx, fy, fingerLen + 2 * pad, fingerW + 2 * pad, 3 + pad, color);
    tft.fillRoundRect(fx + fingerLen - pad, fy, fingerLen + 2 * pad, fingerW + 2 * pad, 3 + pad, color);
  }
}
