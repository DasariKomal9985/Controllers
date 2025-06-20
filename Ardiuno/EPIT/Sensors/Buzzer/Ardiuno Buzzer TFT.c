#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9
#define BUZZER_PIN 41
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
unsigned long startTime = 0;
int currentStage = 0;
void setup() {
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  drawStaticLayout();
  startTime = millis();
}
void loop() {
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - startTime;
  if (elapsed < 15000) {
    if (elapsed < 5000) {
      beepStage(1);
    } else if (elapsed < 10000) {
      beepStage(2);
    } else {
      beepStage(3);
    }
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}
void drawStaticLayout() {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(0, 10, 320, 40, ST77XX_YELLOW);
  tft.setCursor(90, 20);
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_BLACK);
  tft.print("Buzzer");
  tft.fillRect(0, 55, 320, 40, ST77XX_BLUE);
  tft.setCursor(100, 65);
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("Status");
  tft.fillRect(0, 100, 320, 40, ST77XX_GREEN);
  tft.setCursor(80, 110);
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_BLACK);
  tft.print("Beep Mode");
  drawModeBoxes(1);
}
void drawModeBoxes(int active) {
  tft.fillRect(20, 160, 60, 60, ST77XX_RED);
  tft.fillRect(140, 160, 60, 60, ST77XX_GREEN);
  tft.fillRect(250, 160, 60, 60, ST77XX_BLUE);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  if (active == 1) {
    tft.drawRect(15, 155, 70, 70, ST77XX_WHITE);
  } else if (active == 2) {
    tft.drawRect(135, 155, 70, 70, ST77XX_WHITE);
  } else if (active == 3) {
    tft.drawRect(245, 155, 70, 70, ST77XX_WHITE);
  }
  tft.setCursor(40, 180);
  tft.print("5");
  tft.setCursor(170, 180);
  tft.print("1");
  tft.setCursor(265, 180);
  tft.print("0.5");
}
void beepStage(int stage) {
  static unsigned long lastBeepTime = 0;
  static bool buzzerState = false;
  switch (stage) {
    case 1:
      if (currentStage != 1) {
        drawModeBoxes(1);
        Serial.println("Stage 1: 1s Continuous Beep");
        currentStage = 1;
      }
      digitalWrite(BUZZER_PIN, HIGH);
      break;
    case 2:
      if (currentStage != 2) {
        drawModeBoxes(2);
        Serial.println("Stage 2: 0.5s ON/OFF Beep");
        currentStage = 2;
        lastBeepTime = millis();
        buzzerState = true;
        digitalWrite(BUZZER_PIN, HIGH);
      }
      if (millis() - lastBeepTime >= 500) {
        buzzerState = !buzzerState;
        digitalWrite(BUZZER_PIN, buzzerState);
        lastBeepTime = millis();
      }
      break;
    case 3:
      if (currentStage != 3) {
        drawModeBoxes(3);
        Serial.println("Stage 3: 0.05s ON/OFF Beep");
        currentStage = 3;
        lastBeepTime = millis();
        buzzerState = true;
        digitalWrite(BUZZER_PIN, HIGH);
      }
      if (millis() - lastBeepTime >= 50) {
        buzzerState = !buzzerState;
        digitalWrite(BUZZER_PIN, buzzerState);
        lastBeepTime = millis();
      }
      break;
  }
}
