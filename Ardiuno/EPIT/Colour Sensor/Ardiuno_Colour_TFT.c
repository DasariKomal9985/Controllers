#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS   53
#define TFT_RST  8
#define TFT_DC   7
#define TFT_BL   9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

const int S0 = 26;
const int S1 = 27;
const int S2 = 28;
const int S3 = 29;
const int sensorOut = 30;
const int ledPin = 31;

unsigned long redFrequency = 0;
unsigned long greenFrequency = 0;
unsigned long blueFrequency = 0;

String lastColor = "";
String lastCircleText = "";
unsigned long lastRed = 0, lastGreen = 0, lastBlue = 0;

void setup() {
  Serial.begin(9600);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  pinMode(ledPin, OUTPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  digitalWrite(ledPin, LOW);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  drawUI();
}

void loop() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW);

  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);

  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW);

  String colorName = detectColor(redFrequency, greenFrequency, blueFrequency);

  Serial.print("R: "); Serial.print(redFrequency);
  Serial.print(" | G: "); Serial.print(greenFrequency);
  Serial.print(" | B: "); Serial.print(blueFrequency);
  Serial.print(" | Detected: "); Serial.println(colorName);

  if (colorName != lastColor) {
    lastColor = colorName;

    tft.fillRect(0, 90, 320, 30, ST77XX_BLACK);
    tft.drawRect(0, 90, 320, 30, ST77XX_GREEN);
    tft.setCursor(40, 97);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.print(colorName);
  }

  if (redFrequency != lastRed) {
    lastRed = redFrequency;
    tft.fillRect(0, 130, 160, 30, ST77XX_RED);
    tft.setCursor(15, 135);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.print("R: ");
    tft.print(redFrequency);
  }

  if (greenFrequency != lastGreen) {
    lastGreen = greenFrequency;
    tft.fillRect(0, 170, 160, 30, ST77XX_GREEN);
    tft.setCursor(15, 175);
    tft.setTextColor(ST77XX_BLACK);
    tft.setTextSize(2);
    tft.print("G: ");
    tft.print(greenFrequency);
  }

  if (blueFrequency != lastBlue) {
    lastBlue = blueFrequency;
    tft.fillRect(0, 210, 160, 30, ST77XX_BLUE);
    tft.setCursor(15, 215);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.print("B: ");
    tft.print(blueFrequency);
  }

  if (colorName != lastCircleText) {
    lastCircleText = colorName;
    uint16_t colorCode = getColorCode(colorName);

    int circleX = 260;
    int circleY = 180;
    int radius = 45;

    tft.fillRect(circleX - radius - 2, circleY - radius - 2, radius * 2 + 4, radius * 2 + 4, ST77XX_BLACK);

    if (colorName == "UNKNOWN") {
      tft.drawCircle(circleX, circleY, radius, ST77XX_WHITE);
    } else {
      tft.fillCircle(circleX, circleY, radius, colorCode);
    }

    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    int16_t x, y;
    uint16_t w, h;
    tft.getTextBounds(colorName, 0, 0, &x, &y, &w, &h);
    tft.setCursor(circleX - w / 2, circleY - h / 2);
    tft.print(colorName);
  }

  delay(1000);
}

void drawUI() {
  tft.fillRect(0, 0, 320, 30, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(2);
  tft.setCursor(30, 10);
  tft.print("Colour Sensor");

  tft.drawRect(0, 45, 320, 30, ST77XX_BLUE);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(30, 53);
  tft.print("Colour Status");
}

String detectColor(unsigned long r, unsigned long g, unsigned long b) {
  if (r > 120 && r < 150 && g > 1800 && g < 2000 && b > 950 && b < 2000) return "RED";
  if (r > 700 && r < 900 && g > 120 && g < 220 && b > 100 && b < 160) return "BLUE";
  if ((r > 180 && r < 250 && g > 120 && g < 170 && b > 280 && b < 400) || (r > 1000 && r < 3000 && g > 500 && g < 800 && b > 1000 && b < 3000)) return "GREEN";
  if (r > 20 && r < 90 && g > 110 && g < 160 && b > 200 && b < 300) return "YELLOW";
  if (r > 220 && r < 290 && g > 350 && g < 430 && b > 350 && b < 450) return "PINK";
  if (r > 200 && r < 250 && g > 240 && g < 300 && b > 260 && b < 310) return "WHITE";
  return "UNKNOWN";
}

uint16_t getColorCode(String colorName) {
  if (colorName == "RED") return ST77XX_RED;
  if (colorName == "GREEN") return ST77XX_GREEN;
  if (colorName == "BLUE") return ST77XX_BLUE;
  if (colorName == "YELLOW") return ST77XX_YELLOW;
  if (colorName == "WHITE") return ST77XX_WHITE;
  if (colorName == "PINK") return ST77XX_MAGENTA;
  if (colorName == "BLACK") return ST77XX_BLACK;
  return ST77XX_BLACK;
}
