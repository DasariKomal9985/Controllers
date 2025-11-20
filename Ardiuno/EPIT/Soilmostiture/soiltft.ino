#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
#define SoilMoisture A5
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define LIGHT_GREEN 0xAFE5
#define DARK_GREEN 0x0720
#define LIGHT_BLUE 0x867F
#define DARK_BLUE 0x0014
#define SOIL_BROWN 0x8410
#define SOIL_DARK 0x6308
#define GRAY 0xC618
#define DRY_THRESHOLD 30
#define WET_THRESHOLD 70
unsigned int moisturePercent = 0;
int previousPercent = -1;
void setup() {
  Serial.begin(9600);
  pinMode(SoilMoisture, INPUT);
  pinMode(TFT_BL, OUTPUT);
  tft.init(240, 320);
  tft.setRotation(1);
  digitalWrite(TFT_BL, HIGH);
  tft.fillScreen(BLACK);
  drawStaticElements();
}
void loop() {
  int SoilMoistureValue = analogRead(SoilMoisture);
  int16_t SoilMapedValue = map(SoilMoistureValue, 0, 1023, 0, 100);
  Serial.print("Soil Values : ");
  Serial.println(SoilMapedValue);
  Serial.print("Raw: ");
  Serial.print(SoilMoistureValue);
  Serial.print(" | Percent: ");
  Serial.println(SoilMapedValue);
  if (SoilMapedValue != previousPercent) {
    updateDisplay(SoilMapedValue);
    previousPercent = SoilMapedValue;
  }
  delay(500);
}
void drawStaticElements() {
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(20, 20);
  tft.print("SOIL MOISTURE METER");
  drawSoil();
  drawMoistureMeter();
}
void drawSoil() {
  int soilHeight = SCREEN_HEIGHT / 2.5;
  int soilY = SCREEN_HEIGHT - soilHeight;
  tft.fillRect(0, soilY, 320, soilHeight, SOIL_BROWN);
  for (int i = 0; i < 40; i++) {
    int spotX = random(SCREEN_WIDTH);
    int spotY = random(soilY, SCREEN_HEIGHT);
    int spotSize = random(3, 8);
    tft.fillCircle(spotX, spotY, spotSize, SOIL_DARK);
  }
}
void drawMoistureMeter() {
  int meterWidth = 80;
  int meterHeight = 100;
  int meterX = SCREEN_WIDTH / 2 - meterWidth / 2;
  int meterY = SCREEN_HEIGHT / 2 - meterHeight;
  tft.fillRoundRect(meterX, meterY, meterWidth, meterHeight, 10, LIGHT_GREEN);
  tft.drawRoundRect(meterX, meterY, meterWidth, meterHeight, 10, DARK_GREEN);
  tft.drawRoundRect(meterX + 1, meterY + 1, meterWidth - 2, meterHeight - 2, 9, DARK_GREEN);
  int screenWidth = meterWidth - 20;
  int screenHeight = meterHeight / 3;
  int screenX = meterX + 10;
  int screenY = meterY + 15;
  tft.fillRect(screenX, screenY, screenWidth, screenHeight, LIGHT_BLUE);
  tft.drawRect(screenX, screenY, screenWidth, screenHeight, DARK_BLUE);
  int buttonWidth = 20;
  int buttonHeight = 5;
  int buttonY = meterY + meterHeight - 25;
  tft.fillRect(meterX + meterWidth / 2 - buttonWidth / 2, buttonY, buttonWidth, buttonHeight, BLACK);
  tft.fillRect(meterX + meterWidth / 2 - buttonWidth / 2, buttonY + 15, buttonWidth, buttonHeight, BLACK);
  int probeWidth = 8;
  int probeHeight = 120;
  int probe1X = meterX + meterWidth / 3 - probeWidth / 2;
  int probe2X = meterX + 2 * meterWidth / 3 - probeWidth / 2;
  int probeY = meterY + meterHeight;
  tft.fillRect(probe1X, probeY, probeWidth, probeHeight, GRAY);
  tft.fillRect(probe2X, probeY, probeWidth, probeHeight, GRAY);
  tft.fillRect(probe1X, probeY + probeHeight - 10, probeWidth, 10, DARK_GREEN);
  tft.fillRect(probe2X, probeY + probeHeight - 10, probeWidth, 10, DARK_GREEN);
}
void updateDisplay(int moisturePercent) {
  int meterWidth = 80;
  int meterHeight = 100;
  int meterX = SCREEN_WIDTH / 2 - meterWidth / 2;
  int meterY = SCREEN_HEIGHT / 2 - meterHeight;
  int screenWidth = meterWidth - 20;
  int screenHeight = meterHeight / 3;
  int screenX = meterX + 10;
  int screenY = meterY + 15;
  tft.fillRect(screenX + 5, screenY + 5, screenWidth - 10, screenHeight + 10, LIGHT_BLUE);
  String statusText;
  uint16_t textColor;
  if (moisturePercent < DRY_THRESHOLD) {
    statusText = "DRY";
    textColor = RED;
  } else if (moisturePercent < WET_THRESHOLD) {
    statusText = "MOIST";
    textColor = YELLOW;
  } else {
    statusText = "WET";
    textColor = GREEN;
  }
  tft.setTextSize(2.5);
  tft.setTextColor(BLACK);
  tft.setCursor(screenX + 10, screenY + 8);
  tft.print(moisturePercent);
  tft.print("%");
  tft.setTextSize(1.5);
  tft.setTextColor(textColor);
  tft.setCursor(screenX + 15, screenY + 35);
  tft.print(statusText);
  updateSoilMoisture(moisturePercent);
}
void updateSoilMoisture(int moisturePercent) {
  int meterWidth = 80;
  int meterHeight = 100;
  int meterX = SCREEN_WIDTH / 2 - meterWidth / 2;
  int meterY = SCREEN_HEIGHT / 2 - meterHeight;
  int probeWidth = 8;
  int probeHeight = 60;
  int probe1X = meterX + meterWidth / 3 - probeWidth / 2;
  int probe2X = meterX + 2 * meterWidth / 3 - probeWidth / 2;
  int probeY = meterY + meterHeight;
  int soilHeight = SCREEN_HEIGHT / 3;
  int soilY = SCREEN_HEIGHT - soilHeight;
  drawSoil();
  tft.fillRect(probe1X, probeY, probeWidth, probeHeight, GRAY);
  tft.fillRect(probe2X, probeY, probeWidth, probeHeight, GRAY);
  tft.fillRect(probe1X, probeY + probeHeight - 10, probeWidth, 10, DARK_GREEN);
  tft.fillRect(probe2X, probeY + probeHeight - 10, probeWidth, 10, DARK_GREEN);
  if (moisturePercent > 0) {
    int moistureHeight = map(moisturePercent, 0, 100, 0, probeHeight - 5);
    int moistureY = probeY + probeHeight - 5 - moistureHeight;
    for (int i = 0; i < moistureHeight; i++) {
      int radius = map(i, 0, moistureHeight, 3, 15);
      tft.fillCircle(probe1X + probeWidth / 2, moistureY + i, radius, BLUE);
      tft.fillCircle(probe2X + probeWidth / 2, moistureY + i, radius, BLUE);
    }
    tft.fillRect(probe1X, probeY, probeWidth, probeHeight, GRAY);
    tft.fillRect(probe2X, probeY, probeWidth, probeHeight, GRAY);
    tft.fillRect(probe1X, probeY + probeHeight - 10, probeWidth, 10, DARK_GREEN);
    tft.fillRect(probe2X, probeY + probeHeight - 10, probeWidth, 10, DARK_GREEN);
  }
}