#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>

#define IN1 44
#define IN2 45
#define ENA 3
#define POT_PIN A8
#define BUTTON_PIN 46

LiquidCrystal_I2C lcd(0x27, 20, 4);

bool motorRunning = false;
bool directionClockwise = true;

bool buttonPressed = false;
unsigned long pressStartTime = 0;
int holdSeconds = 0;

int motorSpeed = 0;
int lastReportedHold = 0;
volatile bool lcdNeedsUpdate = false;

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

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  Serial.begin(9600);

  Timer1.initialize(200000);
  Timer1.attachInterrupt(triggerLCDUpdate);

  lcd.setCursor(0, 0);
  lcd.print("L298N Driver");
  updateLCD();
}

void loop() {
  handleButton();
  handlePotentiometer();

  if (lcdNeedsUpdate) {
    lcdNeedsUpdate = false;
    updateLCD();
  }
}

void triggerLCDUpdate() {
  lcdNeedsUpdate = true;
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
      Serial.print("Direction Changed to: ");
      Serial.println(directionClockwise ? "Clockwise" : "AntiClockwise");

      if (motorRunning) applyMotor();
    } else if (holdSeconds >= 2) {
      motorRunning = !motorRunning;
      Serial.print("Motor State: ");
      Serial.println(motorRunning ? "ON" : "OFF");
      applyMotor();
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
  int newSpeed = map(potVal, 0, 1023, 0, 255);

  if (motorRunning && newSpeed != motorSpeed) {
    motorSpeed = newSpeed;
    analogWrite(ENA, motorSpeed);
  } else if (!motorRunning) {
    analogWrite(ENA, 0);
  }
}

void applyMotor() {
  if (motorRunning) {
    digitalWrite(IN1, directionClockwise ? HIGH : LOW);
    digitalWrite(IN2, directionClockwise ? LOW : HIGH);
    analogWrite(ENA, motorSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("L298N Driver       ");

  lcd.setCursor(0, 1);
  lcd.print("Holding: ");
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
    lcd.print("                    ");
    return;
  }

  if (motorSpeed >= 240) {
    lcd.print("Max Speed Reached ");
  } else if (motorSpeed <= 30) {
    lcd.print("Low Limit Reached ");
  } else {
    int bars = map(motorSpeed, 0, 255, 0, 20);
    for (int i = 0; i < 20; i++) {
      lcd.setCursor(i, 3);
      if (i < bars) lcd.write(byte(0));
      else lcd.print(" ");
    }
  }
}
