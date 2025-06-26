#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Keypad.h>
#include <math.h>

#define ENA 10
#define DIR 11
#define PUL 12

#define TFT_CS   53
#define TFT_RST  8
#define TFT_DC   7
#define TFT_BL   9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};
byte rowPins[ROWS] = { 37, 38, 39, 40 };
byte colPins[COLS] = { 33, 34, 35, 36 };
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

char spinnerChars[4] = { '|', '/', '-', '\\' };
int spinIndex = 0;
int fanAngle = 0;
uint16_t fanColors[4] = {
  ST77XX_RED, ST77XX_GREEN, ST77XX_BLUE, ST77XX_YELLOW
};

int lastSteps = -1;
char lastSpin = '\0';
unsigned long lastUpdate = 0;

#define STEP_LABEL_X 20
#define STEP_LABEL_Y 120
#define STEP_VALUE_X 120
#define STEP_VALUE_Y 120

#define SPIN_LABEL_X 20
#define SPIN_LABEL_Y 180
#define FAN_CENTER_X 260
#define FAN_CENTER_Y 200

void drawBoxes() {
  tft.fillRect(10, 10, 310, 40, ST77XX_YELLOW);
  tft.fillRect(10, 60, 310, 40, ST77XX_WHITE);
  tft.drawRect(10, 110, 310, 40, ST77XX_WHITE);
  tft.drawRect(10, 160, 100, 80, ST77XX_WHITE);
  tft.drawRect(210, 160, 100, 80, ST77XX_WHITE);
}

void drawFan(int cx, int cy, int radius, int angleOffset) {
  tft.fillRect(211, 161, 98, 78, ST77XX_BLACK);
  tft.fillCircle(cx, cy, 6, ST77XX_WHITE);
  for (int i = 0; i < 4; i++) {
    float angle = radians(i * 90 + angleOffset);
    int x1 = cx + radius * cos(angle - 0.2);
    int y1 = cy + radius * sin(angle - 0.2);
    int x2 = cx + radius * cos(angle + 0.2);
    int y2 = cy + radius * sin(angle + 0.2);
    tft.fillTriangle(cx, cy, x1, y1, x2, y2, fanColors[i]);
    int xtip = cx + (radius - 2) * cos(angle);
    int ytip = cy + (radius - 2) * sin(angle);
    tft.fillCircle(xtip, ytip, 4, fanColors[i]);
  }
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  digitalWrite(ENA, LOW);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);

  drawBoxes();

  tft.setCursor(20, 20);
  tft.setTextColor(ST77XX_BLACK);
  tft.print("Stepper Motor & Driver");

  tft.fillRect(STEP_LABEL_X, STEP_LABEL_Y, 100, 20, ST77XX_BLACK);
  tft.setCursor(STEP_LABEL_X, STEP_LABEL_Y);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("Steps:");

  tft.fillRect(SPIN_LABEL_X, SPIN_LABEL_Y, 80, 20, ST77XX_BLACK);
  tft.setCursor(SPIN_LABEL_X, SPIN_LABEL_Y);
  tft.setTextColor(ST77XX_MAGENTA);
  tft.print("Spin");

  tft.setCursor(20, 70);
  tft.setTextColor(ST77XX_RED);
  tft.print("Waiting for input...");
}

void loop() {
  char key = keypad.getKey();
  if (key) {
    int steps = 0;
    bool dir = true;
    int speedDelay = 1000;

    switch (key) {
      case '1': steps = 100; break;
      case '2': steps = 200; break;
      case '3': steps = 300; break;
      case '4': steps = 400; break;
      case '5': steps = 500; break;
      case '6': steps = 600; break;
      case '7': steps = 700; break;
      case '8': steps = 800; break;
      case '9': steps = 900; break;
      case 'A': steps = 1000; break;
      case '0': steps = 1600; break;
      case '*': steps = 1600; dir = false; break;
      case 'B': steps = 100; dir = false; break;
      case 'C': steps = 500; dir = false; break;
      case 'D': steps = 1000; dir = false; break;
      case '#':
        tft.fillRect(11, 61, 218, 38, ST77XX_BLACK);
        tft.setCursor(20, 70);
        tft.setTextColor(ST77XX_RED);
        tft.print("Status: STOPPED");
        return;
    }

    digitalWrite(DIR, dir ? HIGH : LOW);
    tft.fillRect(11, 61, 310, 38, ST77XX_BLACK);
    tft.setCursor(20, 70);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("Status: ");
    tft.print(dir ? "CW" : "CCW");

    lastSteps = -1;
    lastSpin = '\0';
    lastUpdate = millis();

    for (int i = 0; i < steps; i++) {
      digitalWrite(PUL, HIGH);
      delayMicroseconds(speedDelay);
      digitalWrite(PUL, LOW);
      delayMicroseconds(speedDelay);

      if (millis() - lastUpdate > 30) {
        if (i != lastSteps) {
          tft.fillRect(STEP_VALUE_X, STEP_VALUE_Y, 100, 20, ST77XX_BLACK);
          tft.setCursor(STEP_VALUE_X, STEP_VALUE_Y);
          tft.setTextColor(ST77XX_CYAN);
          tft.print(i);
          lastSteps = i;
        }

        char spinChar = spinnerChars[spinIndex];
        if (spinChar != lastSpin) {
          tft.fillRect(80, SPIN_LABEL_Y, 20, 20, ST77XX_BLACK);
          tft.setCursor(80, SPIN_LABEL_Y);
          tft.setTextColor(ST77XX_MAGENTA);
          tft.print(spinChar);
          lastSpin = spinChar;
        }

        drawFan(FAN_CENTER_X, FAN_CENTER_Y, 28, fanAngle);
        fanAngle = (fanAngle + 30) % 360;

        spinIndex = (spinIndex + 1) % 4;
        lastUpdate = millis();
      }
    }

    tft.fillRect(11, 61, 218, 38, ST77XX_BLACK);
    tft.setCursor(20, 70);
    tft.setTextColor(ST77XX_GREEN);
    tft.print("Status: Done");
  }
}
