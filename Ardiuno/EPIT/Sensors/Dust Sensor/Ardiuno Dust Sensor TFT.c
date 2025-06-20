#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS 53
#define TFT_DC 7
#define TFT_RST 8
#define TFT_BL 9

const int dustSensorPin = A3;

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

const float alpha = 0.85;
float filteredPPM = 0;

int lastRoundedPPM = -1;
String lastStatus = "";
bool wiringIssue = false;
int lastArrowIndex = -1;

const int barY = 130;
const int barHeight = 30;
const int barWidth = 320;
const int boxCount = 4;
const int boxWidth = barWidth / boxCount;

uint16_t statusColors[boxCount] = {
  ST77XX_GREEN, ST77XX_BLUE, ST77XX_YELLOW, ST77XX_RED
};

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  Serial.begin(9600);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  drawMainUI();
}

void drawMainUI() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(4);
  tft.setCursor(10, 20);
  tft.print("Dust Sensor");

  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(3);
  tft.setCursor(10, 60);
  tft.print("VALUE:");

  tft.setCursor(10, 90);
  tft.print("STATUS:");

  drawStatusBar();
}

void drawStatusBar() {
  for (int i = 0; i < boxCount; i++) {
    tft.fillRect(i * boxWidth, barY, boxWidth, barHeight, statusColors[i]);
    tft.drawRect(i * boxWidth, barY, boxWidth, barHeight, ST77XX_WHITE);

    String label = (i == 0) ? "Clear Air" : (i == 1) ? "Low"
                                          : (i == 2) ? "Moderate"
                                                     : "High";

    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(label, 0, 0, &x1, &y1, &w, &h);
    int labelX = i * boxWidth + (boxWidth - w) / 2;
    int labelY = barY + barHeight + 5;

    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(labelX, labelY);
    tft.print(label);
  }
}

String getStatus(float ppm) {
  if (ppm == 0) return "Clear Air";
  else if (ppm < 50) return "Low";
  else if (ppm < 150) return "Moderate";
  else return "High";
}

int getStatusIndex(String status) {
  if (status == "Clear Air") return 0;
  if (status == "Low") return 1;
  if (status == "Moderate") return 2;
  if (status == "High") return 3;
  return -1;
}

void showWiringIssue() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(3);
  tft.setCursor(40, 100);
  tft.print("WIRING ISSUE!");
  Serial.println("WIRING ISSUE DETECTED!");
}

void restoreNormalScreen() {
  drawMainUI();
  lastRoundedPPM = -1;
  lastStatus = "";
  lastArrowIndex = -1;
}

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

void updateDisplay(int roundedPPM, String statusText) {
  if (roundedPPM != lastRoundedPPM) {
    tft.fillRect(120, 60, 200, 30, ST77XX_BLACK);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(3);
    tft.setCursor(120, 60);
    tft.print(roundedPPM);
    tft.print(" ug/m3");
    lastRoundedPPM = roundedPPM;
  }

  if (statusText != lastStatus) {
    tft.fillRect(120, 90, 200, 30, ST77XX_BLACK);
    tft.setCursor(120, 90);
    tft.setTextColor(ST77XX_MAGENTA);
    tft.setTextSize(3);
    tft.print(statusText);
    lastStatus = statusText;
  }
}

int readSensor() {
  long sumADC = 0;
  const int sampleCount = 50;
  const float baselineVoltage = 0.7;  // Adjust this based on your calibration

  for (int i = 0; i < sampleCount; i++) {
    digitalWrite(ledPin, LOW);
    delayMicroseconds(280);
    sumADC += analogRead(dustSensorPin);
    delayMicroseconds(40);
    digitalWrite(ledPin, HIGH);
    delayMicroseconds(9680);
  }

  float avgADC = sumADC / (float)sampleCount;
  float voltage = avgADC * (5.0 / 1023.0);

  float dustDensity = 0;
  if (voltage > baselineVoltage) {
    dustDensity = (voltage - baselineVoltage) * 1000.0 / 0.5;
  } else {
    dustDensity = 0;
  }

  filteredPPM = alpha * filteredPPM + (1 - alpha) * dustDensity;

  Serial.print("Raw ADC: ");
  Serial.print(avgADC);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V | Dust Density (PPM): ");
  Serial.println((int)(filteredPPM + 0.5));

  return (int)(filteredPPM + 0.5);
}

void loop() {
  int roundedPPM = readSensor();

  if (roundedPPM > 800) {
    if (!wiringIssue) {
      showWiringIssue();
      wiringIssue = true;
    }
    delay(1000);
    return;
  }

  if (wiringIssue && roundedPPM <= 500) {
    wiringIssue = false;
    restoreNormalScreen();
  }

  String statusText = getStatus(roundedPPM);
  int statusIndex = getStatusIndex(statusText);

  updateDisplay(roundedPPM, statusText);

  static bool toggle = false;
  static unsigned long lastToggleTime = 0;
  if (millis() - lastToggleTime > 500) {
    toggle = !toggle;
    lastToggleTime = millis();
    animateStatusBox(statusIndex, toggle);
  }

  if (statusIndex != lastArrowIndex) {
    drawArrow(statusIndex);
  }

  Serial.print("Dust Value: ");
  Serial.print(roundedPPM);
  Serial.print(" | Status: ");
  Serial.println(statusText);

  delay(500);
}
