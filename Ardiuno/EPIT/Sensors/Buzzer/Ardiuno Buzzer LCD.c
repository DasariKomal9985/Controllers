#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define BUZZER_PIN 42

LiquidCrystal_I2C lcd(0x27, 20, 4);

unsigned long startTime = 0;
int currentStage = 0;

void setup() {
  Serial.begin(9600);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  lcd.init();
  lcd.backlight();

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
    clearBeepLine();
  }
}

void drawStaticLayout() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Buzzer");

  lcd.setCursor(0, 1);
  lcd.print("Buzzer Sounds");
}

void clearBeepLine() {
  lcd.setCursor(0, 2);
  lcd.print("                    ");
  currentStage = 0;
}

void beepStage(int stage) {
  static unsigned long lastBeepTime = 0;
  static bool buzzerState = false;

  switch (stage) {
    case 1:
      if (currentStage != 1) {
        lcd.setCursor(0, 2);
        lcd.print("1 sec");
        Serial.println("Stage 1: 1s Continuous Beep");
        currentStage = 1;
      }
      digitalWrite(BUZZER_PIN, HIGH);
      break;

    case 2:
      if (currentStage != 2) {
        lcd.setCursor(0, 2);
        lcd.print("0.5 sec");
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
        lcd.setCursor(0, 2);
        lcd.print("0.05 sec");
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
