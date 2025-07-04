#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>

#define DIR_PIN 11
#define STEP_PIN 12
#define EN_PIN 10
#define PCF_ADDR 0x20

LiquidCrystal_I2C lcd(0x27, 20, 4);

char keys[4][4] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

String inputSpeed = "";
volatile unsigned long stepDelay = 1000;
volatile bool stepPinState = LOW;
volatile unsigned long lastToggleMicros = 0;

int speedValue = 0;
int lastBars = -1;  
int lastSpeedValue = -1;
String direction = "Stopped";
String lastDirection = "";
bool motorEnabled = false;
bool dirState = true;

unsigned long stepCount = 0;
unsigned long lastLCDUpdate = 0;

void setup() {
  Wire.begin();
  lcd.init();
  lcd.backlight();

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, HIGH);
  digitalWrite(DIR_PIN, dirState);

  lcd.setCursor(0, 0);
  lcd.print("Stepper & Driver");

  Timer1.initialize(100);
  Timer1.attachInterrupt(stepperISR);
}

void loop() {
  char key = scanKeypad();
  if (key != 0) {
    if (key >= '0' && key <= '9') {
      if (inputSpeed.length() < 3) inputSpeed += key;
    } else if (key == '*') {
      direction = "Clockwise";
      dirState = true;
      applySpeed();
    } else if (key == '#') {
      direction = "AntiClockwise";
      dirState = false;
      applySpeed();
    } else if (key == 'A') {
      direction = "Stopped";
      motorEnabled = false;
      digitalWrite(EN_PIN, HIGH);
    } else if (key == 'C') {
      inputSpeed = "";
      direction = "Stopped";
      motorEnabled = false;
      digitalWrite(EN_PIN, HIGH);
    }

    digitalWrite(DIR_PIN, dirState);
    updateLCD(true);  // 
    delay(200);
  }

  if (millis() - lastLCDUpdate > 300) {
    lastLCDUpdate = millis();
    updateLCD(false);  // 
  }
}

void applySpeed() {
  speedValue = inputSpeed.toInt();
  if (speedValue > 300) speedValue = 300;

  if (speedValue < 20) {
    direction = "Stopped";
    motorEnabled = false;
    digitalWrite(EN_PIN, HIGH);
  } else {
    stepDelay = map(speedValue, 20, 300, 2000, 300);
    motorEnabled = true;
    digitalWrite(EN_PIN, LOW);
  }
  inputSpeed = "";
}

void updateLCD(bool force) {
  if (force || direction != lastDirection) {
    lcd.setCursor(0, 1);
    lcd.print("Dir: ");
    lcd.print(direction);
    lcd.print("        ");
    lastDirection = direction;
  }

  if (force || speedValue != lastSpeedValue || inputSpeed.length() > 0) {
    lcd.setCursor(0, 2);
    lcd.print("Speed: ");
    if (inputSpeed.length() > 0) {
      lcd.print("[" + inputSpeed + "]     ");
    } else {
      lcd.print(speedValue);
      lcd.print("       ");
    }
    lastSpeedValue = speedValue;
  }

  if (direction == "Stopped") {
    lcd.setCursor(0, 3);
    lcd.print("   MOTOR STOPPED    ");
    lastBars = -1; 
  } else {
    int bars = map(speedValue, 0, 300, 0, 20);
    if (bars != lastBars || force) {
      lcd.setCursor(0, 3);
      for (int i = 0; i < 20; i++) {
        lcd.write(i < bars ? byte(255) : ' ');
      }
      lastBars = bars;
    }
  }
}

void stepperISR() {
  if (!motorEnabled) return;

  unsigned long now = micros();
  if (now - lastToggleMicros >= stepDelay) {
    lastToggleMicros = now;
    stepPinState = !stepPinState;
    digitalWrite(STEP_PIN, stepPinState);
    if (stepPinState) stepCount++;
  }
}

char scanKeypad() {
  for (int row = 0; row < 4; row++) {
    byte out = 0xFF;
    out &= ~(1 << (row + 4));
    writePCF(out);
    delay(2);
    byte in = readPCF();
    for (int col = 0; col < 4; col++) {
      if ((in & (1 << col)) == 0) {
        return keys[row][col];
      }
    }
  }
  return 0;
}

void writePCF(byte val) {
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(val);
  Wire.endTransmission();
}

byte readPCF() {
  Wire.requestFrom(PCF_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}
