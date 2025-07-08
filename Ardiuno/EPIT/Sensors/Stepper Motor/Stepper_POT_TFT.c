#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Wire.h>
#include <SPI.h>
#include <TimerOne.h>

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9

#define STEP_PIN 12
#define DIR_PIN 11
#define EN_PIN 10
#define POT_PIN A8
#define BUTTON_PIN 46

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

volatile bool stepState = false;
bool motorRunning = false;
bool directionClockwise = true;

bool buttonPressed = false;
unsigned long pressStartTime = 0;
int holdSeconds = 0;
int lastReportedHold = -1;

int lastSpeed = -1;
bool lastMotorState = false;
String lastDirection = "";

void setup() {
  Serial.begin(9600);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(EN_PIN, HIGH);
  digitalWrite(DIR_PIN, directionClockwise ? HIGH : LOW);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  drawStaticLayout();
  Timer1.initialize(1000);
  Timer1.attachInterrupt(stepperISR);
}

void loop() {
  handleButton();
  updateStepperSpeed();
  updateTFT();
  delay(50);
}

void stepperISR() {
  if (motorRunning) {
    digitalWrite(STEP_PIN, stepState);
    stepState = !stepState;
  } else {
    digitalWrite(STEP_PIN, LOW);
  }
}

void updateStepperSpeed() {
  int potVal = analogRead(POT_PIN);
  int speed = map(potVal, 0, 1023, 2000, 200);
  Timer1.setPeriod(speed);
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
    int currentHold = (millis() - pressStartTime) / 1000;
    if (currentHold != lastReportedHold) {
      lastReportedHold = currentHold;
    }
  }

  if (buttonPressed && state == LOW) {
    int duration = (millis() - pressStartTime) / 1000;

    if (duration >= 5) {
      directionClockwise = !directionClockwise;
      digitalWrite(DIR_PIN, directionClockwise ? HIGH : LOW);
      Serial.println("Direction toggled");
    } else if (duration >= 2) {
      motorRunning = !motorRunning;
      digitalWrite(EN_PIN, motorRunning ? LOW : HIGH);
      Serial.println("Motor toggled");
    }

    buttonPressed = false;
    holdSeconds = 0;
    lastReportedHold = -1;
  }
}

void drawStaticLayout() {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(10, 10, 300, 50, ST77XX_YELLOW);
  tft.setCursor(20, 17);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(3);
  tft.print("Stepper Motor");
  tft.fillRect(10, 70, 300, 35, ST77XX_BLUE);
  tft.setCursor(20, 80);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print("2sec = ON/OFF 5sec = DIR");
  tft.fillRect(10, 110, 300, 35, ST77XX_GREEN);
  tft.setCursor(20, 120);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.print("Holding:");
  tft.fillRect(10, 150, 300, 35, ST77XX_WHITE);
  tft.setCursor(20, 160);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.print("Direction:");
  tft.drawRect(10, 190, 300, 50, ST77XX_WHITE);
}

void updateTFT() {
  static int lastSpeed = -1;
  static bool lastRunning = false;
  static int lastHold = -1;
  static String lastDir = "";
  static String lastStatus = "";

  if (lastHold != lastReportedHold) {
    tft.fillRect(150, 118, 140, 20, ST77XX_GREEN);
    tft.setCursor(160, 120);
    tft.setTextColor(ST77XX_BLACK);
    tft.setTextSize(3);
    if (buttonPressed) {
      tft.print(lastReportedHold);
      tft.print("s   ");
    } else {
      tft.print("-     ");
    }
    lastHold = lastReportedHold;
  }

  String dir = directionClockwise ? "Clockwise" : "AntiClockwise";
  if (dir != lastDir) {
    tft.fillRect(130, 155, 170, 25, ST77XX_WHITE);
    tft.setCursor(140, 160);
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(2);
    tft.print(dir);
    lastDir = dir;
  }

  int barWidth = 300;
  int barHeight = 40;
  int barX = 10;
  int barY = 195;
  int potValue = analogRead(POT_PIN);
  int mappedSpeed = map(potValue, 0, 1023, 0, 255);

  if (motorRunning) {
    if (mappedSpeed > 240 && lastStatus != "max") {
      tft.fillRect(barX, barY, barWidth, barHeight, ST77XX_BLACK);
      tft.setCursor(60, 205);
      tft.setTextColor(ST77XX_RED);
      tft.setTextSize(2);
      tft.print("Max Limit Reached");
      lastStatus = "max";
    } else if (mappedSpeed <= 30 && lastStatus != "low") {
      tft.fillRect(barX, barY, barWidth, barHeight, ST77XX_BLACK);
      tft.setCursor(50, 205);
      tft.setTextColor(ST77XX_YELLOW);
      tft.setTextSize(2);
      tft.print("Low Level Reached");
      lastStatus = "low";
    } else if (mappedSpeed > 30 && mappedSpeed <= 240) {
      int fillW = map(potValue, 0, 1023, 0, barWidth);

      for (int x = 0; x < barWidth; x++) {
        float ratio = (float)x / barWidth;
        uint8_t r = ratio * 255;
        uint8_t g = 255 - r;
        uint8_t b = (1 - abs(0.5 - ratio) * 2) * 255;
        uint16_t color = tft.color565(r, g, b);
        for (int y = 0; y < barHeight; y++) {
          if (x < fillW)
            tft.drawPixel(barX + x, barY + y, color);
          else
            tft.drawPixel(barX + x, barY + y, ST77XX_BLACK);
        }
      }
      lastStatus = "normal";
    }
  } else if (lastRunning != motorRunning) {
    tft.fillRect(10, 190, 300, 50, ST77XX_BLACK);
    tft.setCursor(90, 200);
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(2);
    tft.print("MOTOR OFF");
    lastStatus = "off";
  }

  lastRunning = motorRunning;
}
