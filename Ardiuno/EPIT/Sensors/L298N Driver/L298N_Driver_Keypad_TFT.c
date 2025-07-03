#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Wire.h>
#include <SPI.h>

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9
#define PCF_ADDR 0x20

#define IN1 44
#define IN2 45
#define ENA 3

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

char keys[4][4] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

String inputSpeed = "";
int speedValue = 0;
String direction = "Stopped";

void setup() {
  Wire.begin();
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init(240, 320);
  tft.setRotation(1);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  drawStaticLayout();
  updateDisplay();
}

void loop() {
  char key = scanKeypad();
  if (key != 0) {
    if (key >= '0' && key <= '9') {
      if (inputSpeed.length() < 3) inputSpeed += key;
    } else if (key == '*') {
      direction = "ClockWise";
      applySpeed();
      moveClockwise();
      inputSpeed = "";
    } else if (key == '#') {
      direction = "AntiClockWise";
      applySpeed();
      moveAntiClockwise();
      inputSpeed = "";
    } else if (key == 'A') {
      direction = "Stopped Motor";
      stopMotor();
    } else if (key == 'C') {
      inputSpeed = "";
      stopMotor();
    }
    updateDisplay();
    delay(200);
  }
}

void applySpeed() {
  speedValue = inputSpeed.toInt();
  if (speedValue > 250) speedValue = 250;

  if (speedValue < 50) {
    stopMotor();
    direction = "Stopped Motor";
  } else {
    analogWrite(ENA, speedValue);
  }
}

void moveClockwise() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void moveAntiClockwise() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  speedValue = 0;
}

void drawStaticLayout() {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(10, 10, 310, 35, ST77XX_YELLOW);
  tft.setCursor(20, 18);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(3);
  tft.print("L298N Driver");
  tft.drawRect(10, 50, 310, 30, ST77XX_WHITE);
  tft.setCursor(14, 58);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.print("0-9 Speed   * - CW   # - ACW   C - Clear  A - Stop");
  tft.fillRect(10, 90, 220, 30, ST77XX_GREEN);
  tft.setCursor(15, 98);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print("Direction:");
  tft.fillRect(10, 130, 220, 30, ST77XX_GREEN);
  tft.setCursor(15, 138);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print("Speed:");
  tft.drawRect(10, 175, 300, 60, ST77XX_WHITE);
}

void updateDisplay() {
  tft.fillRect(150, 90, 200, 30, ST77XX_BLACK);
  tft.setCursor(155, 98);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print(direction);

  tft.fillRect(150, 130, 200, 30, ST77XX_BLACK);
  tft.setCursor(165, 138);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  if (inputSpeed.length() > 0) {
    tft.setCursor(165, 138);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.print("     ");
    tft.setCursor(165, 138);
    tft.print(inputSpeed);
  } else {
    char speedStr[4];
    sprintf(speedStr, "%03d", speedValue);
    tft.setCursor(165, 138);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.print("     ");
    tft.setCursor(165, 138);
    tft.print(speedStr);
  }

  tft.fillRect(11, 176, 298, 58, ST77XX_BLACK);

  if (direction == "Stopped Motor") {
    tft.setCursor(20, 195);
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(3);
    tft.print("STOPPED MOTORS");
  } else {
    int bars = map(speedValue, 0, 250, 0, 27);
    int xStart = 10;
    int yBase = 234;
    int barWidth = 9;
    int barSpacing = 2;
    int height = 58;

    for (int i = 0; i < 27; i++) {
      int x = xStart + i * (barWidth + barSpacing);
      if (i < bars) {
        uint16_t color = ST77XX_RED;
        if (speedValue <= 100) color = ST77XX_GREEN;
        else if (speedValue <= 150) color = ST77XX_YELLOW;
        else if (speedValue <= 220) color = ST77XX_ORANGE;
        else if (speedValue > 230) color = ST77XX_RED;

        tft.fillRect(x, yBase - height, barWidth, height, color);
      } else {
        tft.fillRect(x, yBase - height, barWidth, height, ST77XX_BLACK);
        tft.drawRect(x, yBase - height, barWidth, height, ST77XX_WHITE);
      }
    }

    
    if (speedValue > 230) {
      tft.setCursor(15, 195);
      tft.setTextColor(ST77XX_WHITE);
      tft.setTextSize(2);
      tft.print("Please Slow Down Motors");

      delay(5000);  
      stopMotor();
      direction = "Stopped Motor";
      updateDisplay();  
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
