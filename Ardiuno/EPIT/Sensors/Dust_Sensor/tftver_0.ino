#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

// ==== TFT Pins ====
#define TFT_CS 53
#define TFT_DC 7
#define TFT_RST 8
#define TFT_BL 9

// ==== Dust Sensor Pins ====
#define SENSOR_PIN A3
#define LED_PIN 6

// ==== TFT Object ====
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ==== AQI Table ====
struct AQILevel {
  float minPM;
  float maxPM;
  int qualityNumber;
  const char* evaluation;
} AQI_TABLE[] = {
  { 0, 50, 1, "Excellent" },
  { 50, 100, 2, "Average" },
  { 100, 115, 3, "Light Pollution" },
  { 115, 130, 4, "Moderate Pollution" },
  { 130, 200, 5, "Heavy Pollution" },
  { 200, 500, 6, "Serious Pollution" }
};

float prevDustDensity = -1;
int prevQualityNumber = -1;
String prevEvaluation = "";
#define CHANGE_TOLERANCE 3.0

// ==== Status bar settings ====
const int barY = 130;
const int barHeight = 30;
const int boxCount = 6;
const int boxWidth = 320 / boxCount;
uint16_t statusColors[boxCount] = {
  ST77XX_GREEN,
  ST77XX_YELLOW,
  ST77XX_ORANGE,
  ST77XX_MAGENTA,
  ST77XX_RED,
  ST77XX_WHITE
};

// ==== Animation variables ====
int lastArrowIndex = -1;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  Serial.begin(9600);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  // Step 1: Welcome screen (2 sec)
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(20, 100);
  tft.print("Welcome to");
  tft.setCursor(20, 140);
  tft.print("Dust Sensor");
  delay(2000);

  

  // Step 2: Blank screen for 1 sec
  tft.fillScreen(ST77XX_BLACK);
  delay(1000);

  // Step 3: Pollution Scale (3 sec)
  drawPollutionScale();
  delay(3000);

  // Step 4: Main screen
  drawTitle();
  drawStatusBarScale();
}


void drawTitle() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(4);
  tft.setCursor(20, 20);
  tft.print("Dust Sensor");
}
void drawStatusBarScale() {
  for (int i = 0; i < boxCount; i++) {
    tft.fillRect(i * boxWidth, barY, boxWidth, barHeight, statusColors[i]);
    tft.drawRect(i * boxWidth, barY, boxWidth, barHeight, ST77XX_WHITE);

    // Labels
    String label;
    switch (i) {
      case 0: label = "Good"; break;
      case 1: label = "Average"; break;
      case 2: label = "Light"; break;
      case 3: label = "Moderate"; break;
      case 4: label = "Heavy"; break;
      case 5: label = "Serious"; break;
    }
    int16_t x1, y1;
    uint16_t w, h;
    tft.setTextSize(1);
    tft.getTextBounds(label, 0, 0, &x1, &y1, &w, &h);
    int labelX = i * boxWidth + (boxWidth - w) / 2;
    int labelY = barY + (barHeight - h) / 2;  // **vertical center**
    tft.setTextColor(ST77XX_BLACK, statusColors[i]);
    tft.setCursor(labelX, labelY);
    tft.print(label);
  }
}
void drawPollutionScale() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(40, 10);
  tft.print("Pollution Scale");

  // Data from AQI table colors & labels
  uint16_t scaleColors[6] = {
    ST77XX_GREEN, ST77XX_YELLOW, ST77XX_ORANGE,
    ST77XX_RED, ST77XX_MAGENTA, ST77XX_BLUE
  };
  const char* scaleLabels[6] = {
    "Good", "Moderate", "Unhealthy (SG)",
    "Unhealthy", "Very Unhealthy", "Hazardous"
  };
  int startY = 40;
  int boxHeight = 25;

  for (int i = 0; i < 6; i++) {
    tft.fillRect(10, startY + (i * (boxHeight + 5)), 220, boxHeight, scaleColors[i]);
    tft.drawRect(10, startY + (i * (boxHeight + 5)), 220, boxHeight, ST77XX_WHITE);
    tft.setTextColor(ST77XX_BLACK, scaleColors[i]);
    tft.setTextSize(1);
    tft.setCursor(15, startY + (i * (boxHeight + 5)) + 7);  // vertical center
    tft.print(scaleLabels[i]);
  }
}

float readDustDensity() {
  digitalWrite(LED_PIN, HIGH);
  delayMicroseconds(280);

  int rawValue = analogRead(SENSOR_PIN);
  digitalWrite(LED_PIN, LOW);

  float voltage = (rawValue / 1023.0) * 5.0;
  float dustDensity = (voltage / 0.5) * 100.0;

  if (dustDensity < 0) dustDensity = 0;
  if (dustDensity > 500) dustDensity = 500;

  return dustDensity;
}

void drawPM25(float dustDensity) {
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);  // background color added
  tft.setCursor(10, 70);
  tft.print("PM2.5: ");
  tft.print(dustDensity, 1);
  tft.print(" ug/m3   ");  // extra spaces to overwrite old digits
}

void drawAQI(int qualityNumber, String evaluation) {
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK); // background color prevents old text residue
  tft.setTextSize(2);
  tft.setCursor(10, 100);
  tft.print("AQI: ");
  tft.print(qualityNumber);
  tft.print(" ");
  tft.print(evaluation);
  tft.print("     "); // clear trailing chars
}

// ==== Animation functions ====
void animateStatusBox(int index, bool highlight) {
  if (index < 0 || index >= boxCount) return;
  uint16_t color = highlight ? ST77XX_WHITE : statusColors[index];
  tft.drawRect(index * boxWidth, barY, boxWidth, barHeight, color);
}

void drawArrow(int index) {
  const int arrowBaseY = barY + barHeight + 20;

  if (lastArrowIndex != -1) {
    int eraseX = lastArrowIndex * boxWidth + boxWidth / 2;
    tft.fillTriangle(eraseX - 6, arrowBaseY + 10, eraseX + 6, arrowBaseY + 10, eraseX, arrowBaseY, ST77XX_BLACK);
    tft.fillRect(eraseX - 1, arrowBaseY + 10, 3, 10, ST77XX_BLACK);
  }

  int arrowX = index * boxWidth + boxWidth / 2;
  tft.fillTriangle(arrowX - 6, arrowBaseY + 10, arrowX + 6, arrowBaseY + 10, arrowX, arrowBaseY, ST77XX_WHITE);
  tft.fillRect(arrowX - 1, arrowBaseY + 10, 3, 10, ST77XX_WHITE);

  lastArrowIndex = index;
}

void loop() {
  float dustDensity = readDustDensity();

  int qualityNumber = 0;
  const char* evaluation = "";
  for (int i = 0; i < 6; i++) {
    if (dustDensity >= AQI_TABLE[i].minPM && dustDensity <= AQI_TABLE[i].maxPM) {
      qualityNumber = AQI_TABLE[i].qualityNumber;
      evaluation = AQI_TABLE[i].evaluation;
      break;
    }
  }

  Serial.print("PM2.5: ");
  Serial.print(dustDensity);
  Serial.print(" ug/m3 | AQI: ");
  Serial.print(qualityNumber);
  Serial.print(" | ");
  Serial.println(evaluation);

  if (fabs(dustDensity - prevDustDensity) >= CHANGE_TOLERANCE || prevDustDensity < 0) {
    drawPM25(dustDensity);
    prevDustDensity = dustDensity;
  }

  if (qualityNumber != prevQualityNumber || prevEvaluation != String(evaluation)) {
    drawAQI(qualityNumber, evaluation);

    // Example: Arrow at "Moderate" (index 3), Box at "Heavy" (index 4)
    int arrowIndex = 3;  // Moderate
    int boxIndex = 4;    // Heavy

    // Blink highlight for the chosen box
    for (int i = 0; i < 2; i++) {
      animateStatusBox(boxIndex, true);
      delay(150);
      animateStatusBox(boxIndex, false);
      delay(150);
    }

    // Draw arrow for the chosen arrow index
    drawArrow(arrowIndex);

    prevQualityNumber = qualityNumber;
    prevEvaluation = evaluation;
  }
  delay(500);
}
