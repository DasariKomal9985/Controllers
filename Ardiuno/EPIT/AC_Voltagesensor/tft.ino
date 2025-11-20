/*
   AC Voltage Measurement with Arduino Mega + ST7789 TFT
   Startup: Welcome + Warning + Waveform Animation
   Third Screen: Letter-by-letter heading, thicker wires, U-turn electron flow,
                 voltage in center box + bottom, no long blocking delays
*/

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

// ==== TFT Pins ====
#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9

// Extra colors
#define ST77XX_LIGHTGRAY 0xC618
#define ST77XX_GRAY 0x8410
#define ST77XX_DARKGRAY 0x4208

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ==== AC Voltage Sensor ====
const int sensorPin = A12;
const float VREF = 5.0;
const int ADC_RES = 1024;
float calibrationFactor = 220.0;

// ==== Geometry for third screen ====
const int wireX = 60;         // left junction x
const int redY = 110;         // red wire y
const int greenY = 160;       // green wire y
const int wireHalfWidth = 8;  // thicker wires

// Outer box
const int boxX = wireX;
const int boxY = 90;
const int boxW = 120;
const int boxH = 120;
const unsigned long sampleInterval = 200;  // µs → 5 kHz sampling
unsigned long lastSampleUs = 0;
// Inner voltage box (centered inside outer box)
const int innerW = 72;
const int innerH = 44;
const int innerX = boxX + (boxW - innerW) / 2;
const int innerY = boxY + (boxH - innerH) / 2;

// ==== Live voltage sampling (non-blocking) ====
unsigned long lastComputeMs = 0;
double sumOfSquares = 0.0;
double offsetAccum = 0.0;  // <<< add this
unsigned long sampleCount = 0;
float currentVoltage = 0.0f;  // last computed mains RMS

// ==== Animation state ====
unsigned long lastFrameMs = 0;
int frame = 0;
const int numBalls = 8;
int prevX[numBalls];
int prevY[numBalls];

// ==== Utilities ====
void printTyping(const char* text, int x, int y, uint16_t color, int textSize, int delayMs) {
  tft.setCursor(x, y);
  tft.setTextColor(color);
  tft.setTextSize(textSize);
  for (int i = 0; text[i] != '\0'; i++) {
    tft.print(text[i]);
    delay(delayMs);
  }
}

// Draw thicker horizontal line by stacking lines vertically
void drawThickHLine(int x0, int x1, int y, uint16_t color, int halfWidth) {
  for (int i = -halfWidth; i <= halfWidth; i++) {
    tft.drawLine(x0, y + i, x1, y + i, color);
  }
}

// ==== Waveform Animation (kept short) ====
void drawWaveform() {
  int prevY = 200;
  for (int x = 0; x < 320; x++) {
    float ampMod = sin(x * 0.05) * 0.5 + 0.5;
    float amplitude = 10 + ampMod * 20;
    float phase = fmod(x * 0.1, 2.0);
    float triangle = (phase < 1.0) ? phase : (2.0 - phase);
    triangle = (triangle * 2.0) - 1.0;
    int y = 200 + (int)(amplitude * triangle);
    if (x > 0) tft.drawLine(x - 1, prevY, x, y, ST77XX_CYAN);
    prevY = y;
    delay(3);  // brief
  }
}

// ==== Third screen static scene (no dynamic content) ====
void drawFlowSceneStatic() {
  tft.fillScreen(ST77XX_BLACK);

  // Heading will be typed separately; reserve area
  // Wires (thick)
  drawThickHLine(0, wireX, redY, ST77XX_RED, wireHalfWidth);
  drawThickHLine(0, wireX, greenY + 26, ST77XX_GREEN, wireHalfWidth);

  // Gray terminals
  tft.fillCircle(wireX, redY, 8, ST77XX_GRAY);
  tft.fillCircle(wireX, greenY + 26, 8, ST77XX_GRAY);

  // Outer box
  tft.drawRect(boxX, boxY, boxW, boxH, ST77XX_WHITE);

  // Inner box (for voltage)
  tft.drawRect(innerX + 4, innerY, innerW, innerH, ST77XX_YELLOW);
}

// ==== Third screen heading typing ====
void typeThirdScreenHeading() {
  tft.setTextWrap(false);
  printTyping("AC Voltage Sensor", 40, 10, ST77XX_YELLOW, 2, 40);
}

// ==== Draw electrons + center voltage (one frame) ====
// Path: left on red -> to box -> down to green -> left on green (U-turn overall)
void animateElectronsFrame(float voltage) {
  frame++;

  // Electron path lengths
  const int L1 = wireX + 40;
  const int L2 = 70;
  const int L3 = wireX + 40;
  const int totalLen = L1 + L2 + L3;

  for (int i = 0; i < numBalls; i++) {
    int p = (frame * 6 + i * 40) % totalLen;
    int ex = -40, ey = redY;

    if (p < L1) {
      ex = -40 + p;
      ey = redY;
    } else if (p < L1 + L2) {
      int q = p - L1;
      if (q < 20) {
        ex = wireX + q;
        ey = redY;
      } else {
        ex = wireX + 20;
        int v = q - 20;
        if (v > 50) v = 50;
        ey = redY + v;
      }
    } else {
      int r = p - (L1 + L2);
      ex = wireX - r;
      ey = greenY + 26;
    }

    // Erase old ball by overdrawing with background
    if (frame > 1) {
      tft.fillCircle(prevX[i], prevY[i], 4, ST77XX_BLACK);
    }

    // Draw new ball
    tft.fillCircle(ex, ey, 4, ST77XX_CYAN);

    prevX[i] = ex;
    prevY[i] = ey;
  }

  // Update voltage in inner box (no blinking)
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);  // auto-clear bg
  tft.setTextSize(2);
  tft.setCursor(innerX + 20, innerY + 14);
  tft.print(voltage, 0);
  tft.print("V");
}

// ==== Bottom voltage (update only when value changes) ====
void drawBottomVoltage(float voltage) {
  // Clear prior area and draw fresh
  tft.fillRect(10, 250, 220, 40, ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(3);
  tft.setCursor(10, 260);
  tft.print(voltage, 1);
  tft.print(" V");
}

// ==== Startup sequence ====
void startupAnimation() {
  // Screen 1: Welcome
  tft.fillScreen(ST77XX_BLACK);
  printTyping("Welcome to", 70, 100, ST77XX_YELLOW, 3, 60);
  printTyping("AC Voltage Sensor", 10, 140, ST77XX_YELLOW, 3, 60);
  delay(800);

  // Screen 2: Warning
  tft.fillScreen(ST77XX_BLACK);
  printTyping("Be Aware", 90, 20, ST77XX_RED, 3, 80);
  delay(200);
  printTyping("Don't Touch the", 70, 60, ST77XX_WHITE, 2, 50);
  delay(200);
  printTyping("AC  Voltage   Sensor", 40, 100, ST77XX_WHITE, 2, 50);
  delay(400);

  // Waveform (short)
  drawWaveform();
  delay(400);

  // Screen 3: Static scene + heading typing
  drawFlowSceneStatic();
  typeThirdScreenHeading();
}

// ==== Setup ====
void setup() {
  Serial.begin(9600);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init(240, 320);
  tft.setRotation(1);

  startupAnimation();  // No placeholder voltage shown
}

// ==== Loop: non-blocking sampling + animation ====
// ==== Loop: non-blocking sampling + animation ====
// ==== Loop: non-blocking sampling + animation ====
void loop() {
  unsigned long nowUs = micros();
  unsigned long nowMs = millis();

  // 1) Take samples at fixed 200 µs intervals
  if (nowUs - lastSampleUs >= sampleInterval) {
    lastSampleUs = nowUs;

    int adcValue = analogRead(sensorPin);
    float sensorVoltage = (adcValue * VREF) / ADC_RES;

    sumOfSquares += sensorVoltage * sensorVoltage;
    offsetAccum += sensorVoltage;
    sampleCount++;
  }

  // 2) Compute RMS every ~1s
  if (nowMs - lastComputeMs >= 1000 && sampleCount > 0) {
    float mean = offsetAccum / sampleCount;
    float meanSq = sumOfSquares / sampleCount;

    float variance = meanSq - (mean * mean);
    if (variance < 0) variance = 0;

    float sensorRMS = sqrt(variance);
    currentVoltage = sensorRMS * calibrationFactor;

    Serial.print("AC Voltage (RMS): ");
    Serial.print(currentVoltage, 2);
    Serial.println(" V");

    drawBottomVoltage(currentVoltage);

    sumOfSquares = 0.0;
    offsetAccum = 0.0;
    sampleCount = 0;
    lastComputeMs = nowMs;
    drawBottomVoltage(currentVoltage);
    drawTopVoltage(currentVoltage);
  }

  // 3) Animate electrons @ ~60 fps
  if (nowMs - lastFrameMs >= 16) {
    animateElectronsFrame(currentVoltage);
    lastFrameMs = nowMs;
  }
}


// Right-side voltage display
void drawTopVoltage(float voltage) {
  static int lastVoltage = -1;  // store previous value to avoid redraws

  int roundedVoltage = (int)(voltage + 0.5);  // round to nearest int

  // Only update if value changed
  if (roundedVoltage != lastVoltage) {
    const int x = 60;  // adjust for centering
    const int y = 40;  // just below the heading

    tft.setTextSize(2);

    // Draw label ONCE (no flicker)
    if (lastVoltage == -1) {
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.setCursor(x, y);
      tft.print("Voltage = ");
    }

    // Clear just the number area (not the whole line)
    tft.fillRect(x + 120, y, 60, 20, ST77XX_BLACK);

    // Draw new value
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.setCursor(x + 120, y);
    tft.print(roundedVoltage);
    tft.print("V");

    lastVoltage = roundedVoltage;
  }
}
