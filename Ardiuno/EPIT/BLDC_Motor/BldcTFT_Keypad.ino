#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#define ESC_PIN 2
#define PCF_ADDR 0x20

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Servo esc;

char keys[4][4] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

String inputSpeed = "";
bool motorRunning = false;
int percentSpeed = 0;
int lastSpeed = 0;

bool prevMotorState = false;
int prevDisplayedSpeed = -1;
int prevBarLen = -1;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  esc.attach(ESC_PIN);
  esc.writeMicroseconds(1000);
  delay(2000);
  lastSpeed = EEPROM.read(0);
  if (lastSpeed > 100) lastSpeed = 0;
  percentSpeed = 0;
  motorRunning = false;
  tft.fillRect(0, 0, 320, 50, ST77XX_WHITE);
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(10, 15);
  tft.print("BLDC Motor Keypad");
  tft.drawRect(0, 55, 320, 40, ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 70);
  tft.print("0 to 9 Speed, A : Start, B : Start, C : Clear");
  tft.drawRect(0, 100, 150, 40, ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 110);
  tft.print("Speed: ");
  tft.drawRect(0, 150, 150, 40, ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 160);
  tft.print("Motor:");
  drawSpeedBar(0);
}

void loop() {
  handleKeypad();
  updateTFT();
}

void handleKeypad() {
  char key = scanKeypad();
  if (key != 0) {
    if (key >= '0' && key <= '9') {
      inputSpeed += key;
      int temp = inputSpeed.toInt();
      if (temp > 100) {
        inputSpeed = "";
        return;
      }
    } else if (key == 'A') {
      if (inputSpeed.length() > 0) {
        int val = inputSpeed.toInt();
        if (val >= 0 && val <= 100) {
          lastSpeed = val;
          percentSpeed = lastSpeed;
          EEPROM.write(0, percentSpeed);
          motorRunning = true;
          applyESC(percentSpeed);
        }
        inputSpeed = "";
      } else {
        motorRunning = true;
        percentSpeed = lastSpeed;
        applyESC(percentSpeed);
      }
    } else if (key == 'B') {
      motorRunning = false;
      esc.writeMicroseconds(1000);
      percentSpeed = 0;
      inputSpeed = "";
    } else if (key == 'C') {
      inputSpeed = "";
    }
    delay(250);
  }
}

void applyESC(int percent) {
  int pulse = map(percent, 0, 100, 1000, 2000);
  esc.writeMicroseconds(pulse);
}

void updateTFT() {
  int displaySpeed = (inputSpeed.length() > 0) ? inputSpeed.toInt() : percentSpeed;
  if (displaySpeed != prevDisplayedSpeed) {
    tft.drawRect(160, 100, 150, 40, ST77XX_WHITE);
    tft.setTextSize(2);
    tft.fillRect(180, 110, 100, 20, ST77XX_BLACK);
    tft.setCursor(190, 110);
    tft.print(String(displaySpeed) + "%");
    prevDisplayedSpeed = displaySpeed;
  }
  if (motorRunning != prevMotorState) {
    tft.drawRect(160, 150, 150, 40, ST77XX_WHITE);
    tft.setTextSize(2);
    tft.fillRect(162, 155, 80, 30, ST77XX_BLACK);
    tft.setCursor(180, 160);
    tft.print(motorRunning ? "ON" : "OFF");
    prevMotorState = motorRunning;
  }
  int barLen = map(motorRunning ? percentSpeed : 0, 0, 100, 0, 100);
  if (barLen != prevBarLen) {
    drawSpeedBar(percentSpeed);
    prevBarLen = barLen;
  }
}

void drawSpeedBar(int percent) {
  const int totalBars = 25;
  const int barWidth = 11;
  const int barHeight = 30;
  const int xStart = 0;
  const int yStart = 200;
  int activeBars = map(percent, 0, 100, 0, totalBars);
  for (int i = 0; i < totalBars; i++) {
    int x = xStart + i * (barWidth + 1);
    uint16_t color;
    if (i < activeBars) {
      if (i < 8) color = ST77XX_GREEN;
      else if (i < 17) color = ST77XX_YELLOW;
      else color = ST77XX_RED;
      tft.fillRect(x, yStart, barWidth, barHeight, color);
    } else {
      tft.fillRect(x, yStart, barWidth, barHeight, ST77XX_BLACK);
      tft.drawRect(x, yStart, barWidth, barHeight, ST77XX_WHITE);
    }
  }
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
