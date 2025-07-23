#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "bmm150.h"
#include "bmm150_defs.h"

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9
#define ST77XX_BLACK 0x0000
#define ST77XX_WHITE 0xFFFF
#define ST77XX_RED 0xF800
#define ST77XX_GREEN 0x07E0
#define ST77XX_BLUE 0x001F
#define ST77XX_YELLOW 0xFFE0
#define ST77XX_CYAN 0x07FF
#define ST77XX_MAGENTA 0xF81F
#define ST77XX_ORANGE 0xFD20
#define ST77XX_GRAY 0x8410
#define ST77XX_BROWN 0xA145
#define ST77XX_PURPLE 0x780F
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

#define PCF_ADDR 0x20
#define RELAY_PIN 37

char keys[4][4] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

String currentInput = "";
String storedPasscode = "";
bool passcodeSet = false;
bool relayActive = false;
unsigned long relayStart = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);

  drawStaticUI();
  updatePasscodeDisplay();
  updateStatusDisplay("Door Closed");
}

void loop() {
  char key = scanKeypad();

  if (key) {
    if (key == 'C') {
      currentInput = "";
      updatePasscodeDisplay();
    } else if (key >= '0' && key <= '9') {
      if (currentInput.length() < 4) {
        currentInput += key;
        updatePasscodeDisplay();
      }
    } else if (key == 'D') {
      if (currentInput.length() == 4) {
        storedPasscode = currentInput;
        passcodeSet = true;
        currentInput = "";
        updatePasscodeDisplay();
        updateStatusDisplay("Passcode Set");
      } else {
        updateStatusDisplay("Enter 4 Digits");
      }
    } else if (key == 'A') {
      if (!passcodeSet) {
        updateStatusDisplay("Set Pass First!");
      } else if (currentInput.length() == 4) {
        if (currentInput == storedPasscode) {
          digitalWrite(RELAY_PIN, LOW);
          relayStart = millis();
          relayActive = true;
          updateStatusDisplay("Door Open");
        } else {
          updateStatusDisplay("Wrong Pass");
        }
        currentInput = "";
        updatePasscodeDisplay();
      } else {
        updateStatusDisplay("Enter 4 Digits");
      }
    }
  }

  if (relayActive && (millis() - relayStart >= 3000)) {
    digitalWrite(RELAY_PIN, HIGH);
    relayActive = false;
    updateStatusDisplay("Door Closed");
  }
}

void drawStaticUI() {
  tft.fillRoundRect(0, 0, 320, 40, 5, ST77XX_WHITE);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(35, 10);
  tft.setTextSize(3);
  tft.println("Solenoid Valve");

  tft.fillRoundRect(0, 45, 320, 40, 5, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_RED);
  tft.setCursor(20, 55);
  tft.setTextSize(2);
  tft.println("Enter Passcode:");

  tft.drawRoundRect(0, 90, 150, 40, 5, ST77XX_WHITE);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(20, 100);
  tft.println("Status:");
}

void updatePasscodeDisplay() {
  tft.fillRect(200, 55, 50, 20, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_PURPLE);
  tft.setCursor(200, 55);
  if (currentInput.length() > 0) {
    tft.print(currentInput);
  } else {
    tft.print("____");
  }
}

void updateStatusDisplay(const char* msg) {
  tft.fillRect(0, 135, 240, 40, ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 150);
  tft.print(msg);
}

void writePCF(byte data) {
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(data);
  Wire.endTransmission();
}

byte readPCF() {
  Wire.requestFrom(PCF_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
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
        while ((readPCF() & (1 << col)) == 0);
        return keys[row][col];
      }
    }
  }
  return 0;
}
