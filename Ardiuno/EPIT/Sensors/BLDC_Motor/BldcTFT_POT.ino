#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9

#define POT_PIN A8
#define ESC_PIN 2
#define BUTTON_PIN 46

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Servo esc;

bool buttonState = false;
bool lastButtonReading = LOW;
bool toggledState = true;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 5;

int lastMappedVal = -1;
bool lastToggledState = false;

int dummySpeed = 0;

void drawStaticLayout();
void updateDisplay(int mappedVal, bool state);

void setup() {
  Serial.begin(9600);

  esc.attach(ESC_PIN);
  esc.writeMicroseconds(1000);
  delay(2000);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init(240, 320);
  tft.setRotation(1);
  drawStaticLayout();
}

void loop() {
  bool currentReading = digitalRead(BUTTON_PIN);

  if (currentReading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (currentReading == LOW && !buttonState) {
      toggledState = !toggledState;
      Serial.print("Button Pressed → ");
      Serial.println(toggledState ? "ON" : "OFF");
      buttonState = true;
    } else if (currentReading == HIGH) {
      buttonState = false;
    }
  }

  lastButtonReading = currentReading;

  int potVal = analogRead(POT_PIN);
  int mappedVal = map(potVal, 0, 840, 0, 100);
  int escSpeed = map(potVal, 0, 840, 1000, 2000);

  if (toggledState) {
    esc.writeMicroseconds(escSpeed);
  } else {
    esc.writeMicroseconds(1000);
  }

  Serial.print("Raw: ");
  Serial.print(potVal);
  Serial.print(" | Mapped (0–100): ");
  Serial.print(mappedVal);
  Serial.print(" | ESC PWM: ");
  Serial.println(toggledState ? escSpeed : 1000);

  updateDisplay(mappedVal, toggledState);
  delay(100);
}

void drawStaticLayout() {
  tft.fillScreen(ST77XX_BLACK);

  tft.fillRect(0, 0, 320, 50, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(3);
  tft.setCursor(50, 7);
  tft.print("BLDC Motor ");

  tft.drawRect(0, 55, 320, 50, ST77XX_WHITE);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(5, 65);
  tft.print("Push Button for ON and OFF");
  tft.setCursor(5, 85);
  tft.print("Toggle Switch for Clock and Anticlock wise Direction");

  tft.drawRect(0, 110, 320, 40, ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(15, 125);
  tft.print("Speed");

  tft.drawRect(0, 155, 150, 40, ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(15, 170);
  tft.print("Status:");
}

void updateDisplay(int mappedVal, bool motorState) {
  if (mappedVal != lastMappedVal) {
    int fillW = map(mappedVal, 0, 100, 0, 300);
    int barY = 200, barH = 40, barX = 0, barW = 300;

    for (int x = 0; x < barW; x++) {
      float ratio = (float)x / barW;
      uint8_t r = ratio * 255;
      uint8_t g = 255 - r;
      uint8_t b = (1 - abs(0.5 - ratio) * 2) * 255;
      uint16_t color = tft.color565(r, g, b);

      for (int y = 0; y < barH; y++) {
        if (x < fillW)
          tft.drawPixel(barX + x, barY + y, color);
        else
          tft.drawPixel(barX + x, barY + y, ST77XX_BLACK);
      }
    }

    lastMappedVal = mappedVal;
  }

  if (motorState != lastToggledState) {
    tft.fillRect(160, 155, 130, 40, ST77XX_BLACK);
    tft.setCursor(175, 165);
    tft.setTextColor(motorState ? ST77XX_GREEN : ST77XX_RED);
    tft.setTextSize(2);
    tft.print(motorState ? "ON " : "OFF");
    lastToggledState = motorState;
  }

  tft.fillRect(100, 115, 100, 30, ST77XX_BLACK);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setCursor(110, 125);
  tft.print(mappedVal);
  tft.print("%");
}
