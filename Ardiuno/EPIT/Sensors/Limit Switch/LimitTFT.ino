#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <math.h>

// ==== TFT Pins ====
#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ===== Base Config =====
#define BASE_ORIENTATION_SIDE 1
#define BASE_ORIENTATION_BOTTOM 2
#define BASE_ORIENTATION BASE_ORIENTATION_SIDE

// ==== Joint Positions ====
int shoulderX, shoulderY;
int elbowX, elbowY;
int wristX, wristY;

// ==== Setup ====
void setup() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  drawBaseOnly();

  float shoulderAngle = 35;  // ✅ Shoulder bends downward
  float elbowOffset = -70;   // ✅ Elbow bends UP relative to shoulder
  drawShoulderArm(shoulderAngle);
  drawForearm(shoulderAngle + elbowOffset);
  drawWristHand(shoulderAngle + elbowOffset);
}

void loop() {
  delay(100);
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

    shoulderX = plateX + plateW;
    shoulderY = plateY + plateH / 2;

    int shoulderR = 12;  // add this here for clarity

    shoulderX = plateX + plateW + shoulderR;  // move right
    shoulderY = plateY + plateH / 2;

    // Shoulder joint (ring)
    tft.drawCircle(shoulderX, shoulderY, shoulderR, ST77XX_WHITE);
    tft.drawCircle(shoulderX, shoulderY, shoulderR - 2, ST77XX_BLUE);


    // Shoulder joint (ring)
    tft.drawCircle(shoulderX, shoulderY, 12, ST77XX_WHITE);
    tft.drawCircle(shoulderX, shoulderY, 10, ST77XX_BLUE);

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

    // Shoulder joint (ring)
    tft.drawCircle(shoulderX, shoulderY, 12, ST77XX_WHITE);
    tft.drawCircle(shoulderX, shoulderY, 10, ST77XX_BLUE);
  }
}

// ==== Shoulder + Upper Arm + Elbow ====
// ==== Shoulder + Upper Arm + Elbow ====
void drawShoulderArm(float angleDeg) {
  int armLen = 80;
  int armWidth = 16;
  float theta = radians(angleDeg);

  // Radii
  int shoulderR = 12;
  int elbowR = 12;

  // Elbow center
  elbowX = shoulderX + armLen * cos(theta);
  elbowY = shoulderY + armLen * sin(theta);

  // Trimmed arm endpoints
  int startX = shoulderX + shoulderR * cos(theta);
  int startY = shoulderY + shoulderR * sin(theta);
  int endX = elbowX - elbowR * cos(theta);
  int endY = elbowY - elbowR * sin(theta);

  // Arm thickness rectangle
  for (int i = -armWidth / 2; i <= armWidth / 2; i++) {
    tft.drawLine(startX, startY + i, endX, endY + i, ST77XX_CYAN);
  }

  // Redraw joints to smooth connection
  tft.fillCircle(shoulderX, shoulderY, shoulderR, ST77XX_BLACK);  // mask overlap
  tft.drawCircle(shoulderX, shoulderY, shoulderR, ST77XX_WHITE);
  tft.drawCircle(shoulderX, shoulderY, shoulderR - 2, ST77XX_BLUE);

  tft.fillCircle(elbowX, elbowY, elbowR, ST77XX_BLACK);  // mask overlap
  tft.drawCircle(elbowX, elbowY, elbowR, ST77XX_WHITE);
  tft.drawCircle(elbowX, elbowY, elbowR - 2, ST77XX_GREEN);
}


// ==== Forearm (angle from elbow) ====
void drawForearm(float angleDeg) {
  int foreLen = 60;
  int foreWidth = 14;

  float theta = radians(angleDeg);

  // Radii of joints
  int elbowR = 12;
  int wristR = 8;

  // Wrist center
  wristX = elbowX + foreLen * cos(theta);
  wristY = elbowY + foreLen * sin(theta);

  // Line endpoints (trimmed to joint borders)
  int startX = elbowX + elbowR * cos(theta);
  int startY = elbowY + elbowR * sin(theta);

  int endX = wristX - wristR * cos(theta);
  int endY = wristY - wristR * sin(theta);

  // Forearm rectangle (thickness)
  for (int i = -foreWidth / 2; i <= foreWidth / 2; i++) {
    tft.drawLine(startX, startY + i, endX, endY + i, ST77XX_YELLOW);
  }

  // Wrist joint
  tft.drawCircle(wristX, wristY, wristR, ST77XX_WHITE);
  tft.drawCircle(wristX, wristY, wristR - 2, ST77XX_RED);
}
// ==== Hand + Palm + Fingers ====
void drawWristHand(float angleDeg) {
  int palmW = 36;
  int palmH = 40;
  float theta = radians(angleDeg);

  // Palm position
  int handOffsetX = 23;
  int palmX = wristX + cos(theta) * 6 + handOffsetX;
  int palmY = wristY + sin(theta) * 6;

  // Palm
  tft.fillRoundRect(palmX - palmW / 2, palmY - palmH / 2, palmW, palmH, 6, ST77XX_WHITE);
  tft.drawRoundRect(palmX - palmW / 2, palmY - palmH / 2, palmW, palmH, 6, ST77XX_BLACK);

  // Thumb (vertical, top)
  int fingerLen = 14;
  int fingerW = 6;
  int thumbX = palmX - fingerW / 2;
  int thumbY = palmY - palmH / 2 - fingerLen;

  tft.fillRoundRect(thumbX, thumbY, fingerW, fingerLen, 3, ST77XX_WHITE);
  tft.drawRoundRect(thumbX, thumbY, fingerW, fingerLen, 3, ST77XX_BLACK);

  tft.fillRoundRect(thumbX, thumbY - fingerLen, fingerW, fingerLen, 3, ST77XX_WHITE);
  tft.drawRoundRect(thumbX, thumbY - fingerLen, fingerW, fingerLen, 3, ST77XX_BLACK);

  // 4 Fingers (horizontal, right side)
  int gap = 4;
  for (int f = 0; f < 4; f++) {
    int fx = palmX + palmW / 2;
    int fy = palmY - palmH / 2 + 4 + f * (fingerW + gap);

    tft.fillRoundRect(fx, fy, fingerLen, fingerW, 3, ST77XX_WHITE);
    tft.drawRoundRect(fx, fy, fingerLen, fingerW, 3, ST77XX_BLACK);

    tft.fillRoundRect(fx + fingerLen, fy, fingerLen, fingerW, 3, ST77XX_WHITE);
    tft.drawRoundRect(fx + fingerLen, fy, fingerLen, fingerW, 3, ST77XX_BLACK);
  }
}
