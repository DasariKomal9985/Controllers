#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Wire.h>
#include <SPI.h>

#define TFT_CS   53
#define TFT_RST  8
#define TFT_DC   7
#define TFT_BL   9

#define POT_PIN A8
#define BUTTON_PIN 46
#define IN1 44
#define IN2 45
#define ENA 3

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

bool motorRunning = false;
bool directionClockwise = true;
bool buttonPressed = false;

unsigned long pressStartTime = 0;
int holdSeconds = 0;
int lastReportedHold = -1;

int motorSpeed = 0;
int lastMotorSpeed = -1;
bool lastMotorState = false;
String lastDirection = "";

void drawStaticLayout();
void moveClockwise();
void moveAntiClockwise();
void applyMotor();
void handleButton();
void handlePotentiometer();
void updateDisplay();

void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init(240, 320);
  tft.setRotation(1);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  drawStaticLayout();
  moveClockwise();
}

void loop() {
  handleButton();
  handlePotentiometer();
  updateDisplay();
}

void moveClockwise() {
  directionClockwise = true;
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void moveAntiClockwise() {
  directionClockwise = false;
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void applyMotor() {
  if (motorRunning) {
    if (directionClockwise) moveClockwise();
    else moveAntiClockwise();
    analogWrite(ENA, motorSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void handleButton() {
  bool state = digitalRead(BUTTON_PIN);
  if (state == HIGH && !buttonPressed) {
    buttonPressed = true;
    pressStartTime = millis();
    holdSeconds = 0;
    lastReportedHold = -1;
  }
  if (buttonPressed && state == HIGH) {
    unsigned long duration = millis() - pressStartTime;
    int currentHold = duration / 1000;
    if (currentHold != lastReportedHold) {
      lastReportedHold = currentHold;
    }
  }
  if (buttonPressed && state == LOW) {
    holdSeconds = (millis() - pressStartTime) / 1000;
    if (holdSeconds >= 5) {
      directionClockwise = !directionClockwise;
      if (motorRunning) applyMotor();
    } else if (holdSeconds >= 2) {
      motorRunning = !motorRunning;
      applyMotor();
    }
    buttonPressed = false;
    holdSeconds = 0;
    lastReportedHold = -1;
  }
}

void handlePotentiometer() {
  int potVal = analogRead(POT_PIN);
  int newSpeed = map(potVal, 0, 1023, 0, 255);
  if (motorRunning && newSpeed != motorSpeed) {
    motorSpeed = newSpeed;
    analogWrite(ENA, motorSpeed);
  } else if (!motorRunning) {
    analogWrite(ENA, 0);
  }
}

void drawStaticLayout() {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(0, 0, 320, 30, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(80, 7);
  tft.print("L298N Driver");
  tft.drawRect(0, 35, 320, 40, ST77XX_WHITE);
  tft.setCursor(10, 48);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.print("2 Sec = ON/OFF   |   5 Sec = DIR CHANGE");
  tft.drawRect(10, 90, 145, 35, ST77XX_WHITE);
  tft.setCursor(15, 98);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.print("Hold Time");
  tft.drawRect(165, 90, 145, 35, ST77XX_WHITE);
  tft.drawRect(10, 135, 145, 35, ST77XX_WHITE);
  tft.setCursor(15, 143);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.print("Direction");
  tft.drawRect(165, 135, 145, 35, ST77XX_WHITE);
}

void updateDisplay() {
  static int prevHoldShown = -1;
  if (lastReportedHold != prevHoldShown) {
    tft.fillRect(170, 95, 130, 25, ST77XX_BLACK);
    tft.setCursor(175, 102);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    if (buttonPressed) {
      tft.print(lastReportedHold);
      tft.print("s");
    } else {
      tft.print("-");
    }
    prevHoldShown = lastReportedHold;
  }

  String currentDir = directionClockwise ? "Clockwise" : "AntiClock";
  if (currentDir != lastDirection) {
    tft.fillRect(170, 140, 130, 25, ST77XX_BLACK);
    tft.setCursor(175, 147);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.print(currentDir);
    lastDirection = currentDir;
  }

  int barY = 200;
  int barH = 30;
  int barX = 10;
  int barW = 300;

  if (motorRunning && motorSpeed != lastMotorSpeed) {
    if (motorSpeed >= 240) {
      tft.fillRect(barX, barY, barW, barH, ST77XX_BLACK);
      tft.setCursor(60, barY + 5);
      tft.setTextColor(ST77XX_RED);
      tft.setTextSize(2);
      tft.print("Max Limit Reached");
    } else if (motorSpeed <= 30) {
      tft.fillRect(barX, barY, barW, barH, ST77XX_BLACK);
      tft.setCursor(60, barY + 5);
      tft.setTextColor(ST77XX_ORANGE);
      tft.setTextSize(2);
      tft.print("Low Limit Reached");
    } else {
      int fillW = map(motorSpeed, 0, 255, 0, barW);
      for (int x = 0; x < barW; x++) {
        float ratio = (float)x / barW;
        uint8_t r = ratio * 255;
        uint8_t g = 255 - r;
        uint8_t b = (1 - abs(0.5 - ratio) * 2) * 255;
        uint16_t color = tft.color565(r, g, b);
        for (int y = 0; y < barH; y++) {
          if (x < fillW)
            tft.drawPixel(barX + x, barY + y, color);
          else
            tft.drawPixel(barX + x, barY + y, ST77XX_BLACK);
        }
      }
    }
    lastMotorSpeed = motorSpeed;
  }

  if (!motorRunning && lastMotorState != motorRunning) {
    tft.fillRect(barX, barY, barW, barH, ST77XX_BLACK);
    tft.setCursor(100, barY + 5);
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(2);
    tft.print("MOTOR OFF");
    lastMotorSpeed = 0;
  }

  lastMotorState = motorRunning;
}
