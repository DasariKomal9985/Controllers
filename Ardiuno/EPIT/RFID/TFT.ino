#include <Wire.h>
#include <MFRC522_I2C.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#define I2C_ADDR_RFID 0x28
#define RST_PIN -1
MFRC522_I2C mfrc522(I2C_ADDR_RFID, RST_PIN, &Wire);

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

byte validUID[4] = { 0x61, 0x2E, 0x38, 0x02 };

void drawUI() {
  tft.drawRect(0, 0, 320, 40, ST77XX_WHITE);
  tft.setCursor(50, 8);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(3);
  tft.print("RFID Scanner");

  tft.drawRect(0, 50, 320, 40, ST77XX_WHITE);
  tft.setCursor(10, 60);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.print("ID: ");

  tft.drawRect(0, 100, 320, 40, ST77XX_WHITE);
  tft.setCursor(10, 110);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print("Status: Waiting...");
}

void updateUID(String uidStr) {
  tft.fillRect(70, 60, 180, 20, ST77XX_BLACK);
  tft.setCursor(80, 60);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print(uidStr);
}

void updateStatus(bool isValid) {
  tft.fillRect(100, 110, 160, 20, ST77XX_BLACK);
  tft.setCursor(100, 110);
  if (isValid) {
    tft.setTextColor(ST77XX_GREEN);
    tft.print("Card Matched");
  } else {
    tft.setTextColor(ST77XX_RED);
    tft.print("Wrong Card");
  }
}

void setup() {
  Serial.begin(9600);
  Wire.setClock(100000);
  Wire.begin();

  tft.init(240, 320);
  tft.setRotation(1);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);

  drawGateClosed();
  drawGateClosedRight();
  mfrc522.PCD_Init();
  delay(100);
  tft.drawRect(0, 145, 170, 90, ST77XX_WHITE);
  drawUI();
}

void loop() {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
    delay(300);
    return;
  }

  String uidStr = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    if (mfrc522.uid.uidByte[i] < 0x10) uidStr += "0";
    uidStr += String(mfrc522.uid.uidByte[i], HEX);
    if (i < mfrc522.uid.size - 1) uidStr += " ";
  }
  uidStr.toUpperCase();
  updateUID(uidStr);
  Serial.print("Card UID: ");
  Serial.println(uidStr);

  bool isValid = true;
  for (byte i = 0; i < 4; i++) {
    if (mfrc522.uid.uidByte[i] != validUID[i]) {
      isValid = false;
      break;
    }
  }

  if (isValid) {
    Serial.println("RFID confirmed");
    updateStatus(true);

    for (int offset = 0; offset <= 20; offset += 2) {
      drawGateStep(offset);
      drawGateStepRight(offset);
      delay(50);
    }
    drawStatusBox(190, 160, true);
    drawStatusBox(290, 160, true);

    displayGateMessage("Gate Open");
    countdownTimer(3);
    displayGateMessage("Gate Closed");

    for (int offset = 20; offset >= 0; offset -= 2) {
      drawGateStep(offset);
      drawGateStepRight(offset);
      delay(50);
    }
    drawStatusBox(190, 160, false);
    drawStatusBox(290, 160, false);

  } else {
    Serial.println("Wrong card!");
    updateStatus(false);
    displayGateMessage("Update Card");

    drawGateDenied();
    drawGateDeniedRight();
    delay(1500);

    drawGateClosed();
    drawGateClosedRight();
    displayGateMessage("Gate Closed");
  }

  mfrc522.PICC_HaltA();
}

void drawGateClosed() {
  tft.fillRect(180, 150, 40, 80, ST77XX_BLACK);
  tft.fillRect(180, 150, 40, 80, ST77XX_WHITE);
  tft.fillTriangle(220, 150, 240, 150, 220, 230, ST77XX_RED);
  drawStatusBox(190, 160, false);
}

void drawGateDenied() {
  drawStatusBox(190, 160, false);
}

void drawGateOpenAnimated() {
  for (int offset = 0; offset <= 20; offset += 2) {
    drawGateStep(offset);
    delay(50);
  }
  drawStatusBox(190, 160, true);
}

void drawGateCloseAnimated() {
  for (int offset = 20; offset >= 0; offset -= 2) {
    drawGateStep(offset);
    delay(50);
  }
  drawStatusBox(190, 160, false);
}

void drawGateStep(int offset) {
  tft.fillTriangle(220, 150, 240, 150, 220, 230, ST77XX_BLACK);

  int topLeftX = 220;
  int topLeftY = 150;
  int topRightX = 240 - offset;
  int topRightY = 150;
  int bottomX = 220;
  int bottomY = 230;

  tft.fillTriangle(topLeftX, topLeftY, topRightX, topRightY, bottomX, bottomY, ST77XX_RED);
}

void drawStatusBox(int x, int y, bool isAllowed) {
  tft.fillRect(x, y, 20, 20, ST77XX_BLACK);

  if (isAllowed) {
    tft.drawLine(x + 3, y + 10, x + 8, y + 15, ST77XX_GREEN);
    tft.drawLine(x + 8, y + 15, x + 17, y + 5, ST77XX_GREEN);
  } else {
    tft.drawLine(x + 4, y + 4, x + 16, y + 16, ST77XX_RED);
    tft.drawLine(x + 16, y + 4, x + 4, y + 16, ST77XX_RED);
  }
}

void drawGateClosedRight() {
  tft.fillRect(280, 150, 40, 80, ST77XX_BLACK);
  tft.fillRect(280, 150, 40, 80, ST77XX_WHITE);
  tft.fillTriangle(280, 150, 260, 150, 280, 230, ST77XX_RED);
  drawStatusBox(290, 160, false);
}

void drawGateDeniedRight() {
  drawStatusBox(290, 160, false);
}

void drawGateOpenAnimatedRight() {
  for (int offset = 0; offset <= 20; offset += 2) {
    drawGateStepRight(offset);
    delay(50);
  }
  drawStatusBox(290, 160, true);
}

void drawGateCloseAnimatedRight() {
  for (int offset = 20; offset >= 0; offset -= 2) {
    drawGateStepRight(offset);
    delay(50);
  }
  drawStatusBox(290, 160, false);
}

void drawGateStepRight(int offset) {
  tft.fillTriangle(280, 150, 260, 150, 280, 230, ST77XX_BLACK);

  int topRightX = 280;
  int topRightY = 150;
  int topLeftX = 260 + offset;
  int topLeftY = 150;
  int bottomX = 280;
  int bottomY = 230;

  tft.fillTriangle(topRightX, topRightY, topLeftX, topLeftY, bottomX, bottomY, ST77XX_RED);
}

void displayGateMessage(String message) {
  tft.fillRect(5, 150, 160, 80, ST77XX_BLACK);
  tft.setCursor(10, 160);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.println(message);
}

void countdownTimer(int seconds) {
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_GREEN);
  tft.fillRect(5, 185, 160, 40, ST77XX_BLACK);
  tft.setCursor(10, 190);
  tft.print("Close in:");

  for (int i = seconds; i >= 1; i--) {
    tft.fillRect(115, 190, 40, 20, ST77XX_BLACK);
    tft.setCursor(120, 190);
    tft.print(i);
    tft.print("s");
    delay(1000);
  }

  tft.fillRect(5, 185, 160, 40, ST77XX_BLACK);

  tft.fillRect(70, 60, 180, 20, ST77XX_BLACK);
  tft.setCursor(80, 60);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("--");

  tft.fillRect(100, 110, 160, 20, ST77XX_BLACK);
  tft.setCursor(100, 110);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("Waiting...");
}