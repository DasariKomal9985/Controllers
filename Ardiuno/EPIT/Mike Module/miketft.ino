
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <math.h>
#define TFT_CS     53
#define TFT_RST    8
#define TFT_DC     7
#define TFT_BL     9
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
#define SOUND_SENSOR A4
#define BLACK       0x0000
#define BLUE        0x001F
#define RED         0xF800
#define GREEN       0x07E0
#define CYAN        0x07FF
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define WHITE       0xFFFF
#define TEAL        0x00FF
#define LIGHT_BLUE  0x07FF
#define MIN_DB 50
#define MAX_DB 90
#define SAMPLE_WINDOW 50
#define HEAD_X 80
#define HEAD_Y 180
#define HEAD_WIDTH 100
#define HEAD_HEIGHT 120
#define WAVE_MAX_WIDTH 120
#define WAVE_COUNT 5
int sample;
float peakToPeak = 0;
unsigned int signalMax = 0;
unsigned int signalMin = 1024;
int db = MIN_DB;
void setup() {
  Serial.begin(9600);
  pinMode(SOUND_SENSOR, INPUT);
  tft.init(240, 320);
  tft.setRotation(1);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, 1);
  tft.fillScreen(BLACK);
  drawStaticElements();
}
void loop() {
  unsigned long startMillis = millis();
  float peakToPeak = 0;
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
  while (millis() - startMillis < SAMPLE_WINDOW) {
    sample = analogRead(SOUND_SENSOR);
    if (sample < 1024) {
      if (sample > signalMax) signalMax = sample;
      if (sample < signalMin) signalMin = sample;
    }
  }
  peakToPeak = signalMax - signalMin;
  int16_t db = map(peakToPeak, 20, 600, 50, 90);
  db = constrain(db, 50, 90);
  if (db <= 55) {
    db = 52;
  }
  updateDisplay(db);
  delay(100);
}
void drawStaticElements() {
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(60, 20);
  tft.print("SOUND LEVEL METER");
  drawHumanHead(TEAL);
}
void drawHumanHead(uint16_t color) {
  int centerX = HEAD_X;
  int centerY = HEAD_Y;
  int headX = centerX - HEAD_WIDTH/2;
  int headY = centerY - HEAD_HEIGHT/2;
  tft.fillRect(headX - 5, headY - 5, HEAD_WIDTH + 10, HEAD_HEIGHT + 10, BLACK);
  tft.fillRoundRect(headX, headY, HEAD_WIDTH, HEAD_HEIGHT/1.5,HEAD_WIDTH/3, color);
  int hairWidth = HEAD_WIDTH;
  int hairHeight = HEAD_HEIGHT/4;
  int hairX = headX;
  int hairY = headY - HEAD_HEIGHT/20;
  tft.fillRoundRect(hairX, hairY, hairWidth, hairHeight, hairHeight/2, TEAL);
  int eyeSize = HEAD_WIDTH/10;
  int leftEyeX = headX + HEAD_WIDTH/3 - eyeSize/2;
  int eyeY = headY + HEAD_HEIGHT/4;
  tft.fillCircle(leftEyeX, eyeY, eyeSize, WHITE);
  tft.fillCircle(leftEyeX, eyeY, eyeSize/2, BLUE);
  int rightEyeX = headX + HEAD_WIDTH*2/3 - eyeSize/2;
  tft.fillCircle(rightEyeX, eyeY, eyeSize, WHITE);
  tft.fillCircle(rightEyeX, eyeY, eyeSize/2, BLUE);
  int eyebrowWidth = eyeSize * 1.5;
  int eyebrowHeight = eyeSize/3;
  int eyebrowY = eyeY - eyeSize - eyebrowHeight/2;
  tft.fillRect(leftEyeX - eyebrowWidth/3, eyebrowY, eyebrowWidth, eyebrowHeight, BLACK);
  tft.fillRect(rightEyeX - eyebrowWidth/3, eyebrowY, eyebrowWidth, eyebrowHeight, BLACK);
  int noseWidth = HEAD_WIDTH/8;
  int noseHeight = HEAD_HEIGHT/8;
  int noseX = centerX - noseWidth/2;
  int noseY = eyeY + eyeSize + noseHeight/4;
  for (int i = 0; i < noseHeight; i++) {
    int lineWidth = noseWidth * i / noseHeight;
    tft.drawFastHLine(noseX + noseWidth/2 - lineWidth/2, noseY + i, lineWidth, BLACK);
  }
  int mouthWidth = HEAD_WIDTH/2;
  int mouthHeight = HEAD_HEIGHT/12;
  int mouthX = centerX - mouthWidth/2;
  int mouthY = noseY + noseHeight + mouthHeight;
  tft.fillRoundRect(mouthX, mouthY, mouthWidth, mouthHeight, mouthHeight, color);
  tft.fillRoundRect(mouthX + mouthWidth/4, mouthY + mouthHeight/4,mouthWidth/2, mouthHeight/2, mouthHeight/2, BLACK);
  int neckWidth = HEAD_WIDTH/3;
  int neckHeight = HEAD_HEIGHT/4;
  int neckX = centerX - neckWidth/2;
  int neckY = headY + HEAD_HEIGHT/1.5 - 2;
  tft.fillRect(neckX, neckY, neckWidth, neckHeight, color);
  int shoulderWidth = HEAD_WIDTH;
  int shoulderHeight = HEAD_HEIGHT/8;
  int shoulderX = centerX - shoulderWidth/2;
  int shoulderY = neckY + neckHeight - 2;
  tft.fillRoundRect(shoulderX, shoulderY, shoulderWidth, shoulderHeight,shoulderHeight/2, color);
}
int previousDB = 0;
String previousLevel = "";
void updateDisplay(int currentDB) {
  static bool firstRun = true;
  if (firstRun) {
    tft.setTextSize(2);
    tft.setTextColor(WHITE);
    tft.setCursor(140, 50);
    tft.print("Loudness: ");
    firstRun = false;
  }
  if (previousDB != currentDB) {
    tft.fillRect(240, 50, 100, 20, BLACK);
    tft.setTextSize(2);
    tft.setTextColor(WHITE);
    tft.setCursor(240, 50);
    tft.print(currentDB);
    tft.print("dB");
    previousDB = currentDB;
  }
  String currentLevel;
  uint16_t levelColor;
  if (currentDB <= 55) {
    currentLevel = "Level: Quiet";
    levelColor = GREEN;
  }
  else if (currentDB > 55 && currentDB < 80) {
    currentLevel = "Level: Moderate";
    levelColor = YELLOW;
  }
  else if (currentDB >= 80) {
    currentLevel = "Level: High";
    levelColor = RED;
  }
  if (previousLevel != currentLevel) {
    tft.fillRect(135, 80, 200, 30, BLACK);
    tft.setTextSize(2);
    tft.setTextColor(levelColor);
    tft.setCursor(135, 80);
    tft.print(currentLevel);
    previousLevel = currentLevel;
  }
  drawSoundWaves(currentDB);
}
void drawSoundWaves(int currentDB) {
  int numElements = map(currentDB, 50, 90, 0, WAVE_COUNT);
  numElements = constrain(numElements, 0, WAVE_COUNT);
  if (currentDB <= 55) {
    numElements = 0;
  }
  int waveOriginX = HEAD_X + HEAD_WIDTH/2;
  int waveOriginY = HEAD_Y + HEAD_HEIGHT/3;
  tft.fillRect(waveOriginX, waveOriginY - HEAD_HEIGHT/2,WAVE_MAX_WIDTH, HEAD_HEIGHT, BLACK);
  for (int i = 1; i <= numElements; i++) {
    int radius = 10 + (i * 8);
    uint16_t waveColor;
    int thickness;
    if (i % 4 == 0) {
      waveColor = TEAL;
      thickness = 3;
    } else if (i % 3 == 0) {
      waveColor = TEAL;
      thickness = 2;
    } else if (i % 2 == 0) {
      waveColor = TEAL;
      thickness = 2;
    } else {
      waveColor = TEAL;
      thickness = 1;
    }
    for (int t = 0; t < thickness; t++) {
      for (int angle = -60; angle <= 60; angle += 2) {
        float radians = angle * PI / 180.0;
        int x = waveOriginX + cos(radians) * (radius + t);
        int y = waveOriginY + sin(radians) * (radius + t);
        tft.drawPixel(x, y, waveColor);
        tft.drawPixel(x, y+1, waveColor);
      }
    }
  }
}
