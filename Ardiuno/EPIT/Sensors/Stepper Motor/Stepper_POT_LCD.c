#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>

#define DIR_PIN 11
#define STEP_PIN 12
#define EN_PIN 10
#define POT_PIN A8
#define BUTTON_PIN 46

LiquidCrystal_I2C lcd(0x27, 20, 4);

volatile bool stepState = false;
volatile unsigned long stepDelay = 1000;

bool motorRunning = false;
bool directionClockwise = true;

bool buttonPressed = false;
unsigned long pressStartTime = 0;
int holdSeconds = 0;

int motorSpeed = 0;
int lastReportedHold = 0;

byte fullBlock[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

void setup() {
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, fullBlock);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  digitalWrite(EN_PIN, HIGH);
  digitalWrite(DIR_PIN, directionClockwise ? HIGH : LOW);

  Serial.begin(9600);

  Timer1.initialize(1000);
  Timer1.attachInterrupt(stepperISR);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Stepper Motor");
}

void loop() {
  handleButton();
  handlePotentiometer();

  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 200) {
    lastUpdate = millis();
    updateLCD();
  }
}

void handleButton() {
  bool state = digitalRead(BUTTON_PIN);

  if (state == HIGH && !buttonPressed) {
    buttonPressed = true;
    pressStartTime = millis();
    holdSeconds = 0;
    lastReportedHold = 0;
  }

  if (buttonPressed && state == HIGH) {
    unsigned long duration = millis() - pressStartTime;
    int currentHold = duration / 1000;

    if (currentHold > lastReportedHold) {
      lastReportedHold = currentHold;
      Serial.print("Holding for: ");
      Serial.print(currentHold);
      Serial.println(" sec");
    }
  }

  if (buttonPressed && state == LOW) {
    holdSeconds = (millis() - pressStartTime) / 1000;

    Serial.print("Button Released. Held for: ");
    Serial.print(holdSeconds);
    Serial.println(" sec");

    if (holdSeconds >= 5) {
      directionClockwise = !directionClockwise;
      digitalWrite(DIR_PIN, directionClockwise ? HIGH : LOW);
      Serial.print("Direction Changed to: ");
      Serial.println(directionClockwise ? "Clockwise" : "AntiClockwise");
    } else if (holdSeconds >= 2) {
      motorRunning = !motorRunning;
      digitalWrite(EN_PIN, motorRunning ? LOW : HIGH);
      Serial.print("Motor State: ");
      Serial.println(motorRunning ? "ON" : "OFF");
    } else {
      Serial.println("Ignored: Short press (<2s)");
    }

    buttonPressed = false;
    holdSeconds = 0;
    lastReportedHold = 0;
  }
}

void handlePotentiometer() {
  int potVal = analogRead(POT_PIN);
  int newSpeed = map(potVal, 0, 1023, 2000, 300);

  if (motorRunning && newSpeed != stepDelay) {
    stepDelay = newSpeed;
    Timer1.setPeriod(stepDelay);
  }
}

void stepperISR() {
  if (!motorRunning) return;

  digitalWrite(STEP_PIN, stepState);
  stepState = !stepState;
}

void updateLCD() {
  static int lastBars = -1;

  lcd.setCursor(0, 0);
  lcd.print("Stepper Motor      ");

  lcd.setCursor(0, 1);
  lcd.print("Hold: ");
  if (buttonPressed) {
    lcd.print(lastReportedHold);
    lcd.print(" sec       ");
  } else {
    lcd.print("-           ");
  }

  lcd.setCursor(0, 2);
  lcd.print("Dir: ");
  lcd.print(directionClockwise ? "Clockwise     " : "AntiClockwise ");

  lcd.setCursor(0, 3);

  if (!motorRunning) {
    lcd.print("     MOTOR OFF     ");
    lastBars = -1;
    return;
  }

  int potVal = analogRead(POT_PIN);
  motorSpeed = map(potVal, 0, 1023, 0, 255);

  if (motorSpeed >= 240) {
    if (lastBars != 255) {
      lcd.setCursor(0, 3);
      lcd.print(" Max Speed Reached ");
      lastBars = 255;
    }
  } else if (motorSpeed <= 30) {
    if (lastBars != 254) {
      lcd.setCursor(0, 3);
      lcd.print(" Low Limit Reached ");
      lastBars = 254;
    }
  } else {
    int bars = map(motorSpeed, 0, 255, 0, 20);
    if (bars != lastBars) {
      for (int i = 0; i < 20; i++) {
        lcd.setCursor(i, 3);
        if (i < bars)
          lcd.write(byte(0));
        else
          lcd.print(" ");
      }
      lastBars = bars;
    }
  }
}
