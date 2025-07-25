#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define ESC_PIN 2
Servo esc;

#define PCF_ADDR 0x20

char keys[4][4] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

String inputSpeed = "";
bool motorRunning = false;
int percentSpeed = 0;
int lastSpeed = 0;

LiquidCrystal_I2C lcd(0x27, 20, 4);

bool prevMotorState = false;
int prevDisplayedSpeed = -1;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  esc.attach(ESC_PIN);
  esc.writeMicroseconds(1000);
  delay(2000);
  lcd.init();
  lcd.backlight();

  lastSpeed = EEPROM.read(0);
  if (lastSpeed > 100) lastSpeed = 0;
  percentSpeed = 0;
  motorRunning = false;

  lcd.setCursor(0, 0);
  lcd.print("BLDC Motor Keypad");
  lcd.setCursor(0, 1);
  lcd.print("Speed: ");
  lcd.setCursor(0, 2);
  lcd.print("Motor: OFF");
}

void loop() {
  handleKeypad();
  updateLCD();
}

void handleKeypad() {
  char key = scanKeypad();
  if (key != 0) {
    Serial.print("Key Pressed: ");
    Serial.println(key);

    if (key >= '0' && key <= '9') {
      inputSpeed += key;
      int temp = inputSpeed.toInt();
      if (temp > 100) {
        inputSpeed = "";
        Serial.println("Invalid input > 100%. Reset.");
        return;
      }
      Serial.print("Input: ");
      Serial.println(inputSpeed);
    }

    else if (key == 'A') {
      if (inputSpeed.length() > 0) {
        int val = inputSpeed.toInt();
        if (val >= 0 && val <= 100) {
          lastSpeed = val;
          percentSpeed = lastSpeed;
          EEPROM.write(0, percentSpeed);
          motorRunning = true;
          applyESC(percentSpeed);
        }
        inputSpeed = "";
      } else {
        motorRunning = true;
        percentSpeed = lastSpeed;
        applyESC(percentSpeed);
      }
    }

    else if (key == 'B') {
      motorRunning = false;
      esc.writeMicroseconds(1000);
      percentSpeed = 0;
      inputSpeed = "";
    }

    else if (key == 'C') {
      inputSpeed = "";
    }

    delay(250);
  }
}

void applyESC(int percent) {
  int pulse = map(percent, 0, 100, 1000, 2000);
  esc.writeMicroseconds(pulse);
  Serial.print("ESC Pulse: ");
  Serial.print(pulse);
  Serial.println(" µs");
}

void updateLCD() {
  int displaySpeed = (inputSpeed.length() > 0) ? inputSpeed.toInt() : percentSpeed;

  if (displaySpeed != prevDisplayedSpeed) {
    lcd.setCursor(7, 1);
    lcd.print("     ");
    lcd.setCursor(7, 1);
    lcd.print(String(displaySpeed) + "%");
    prevDisplayedSpeed = displaySpeed;
  }

  if (motorRunning != prevMotorState) {
    lcd.setCursor(7, 2);
    lcd.print("     ");
    lcd.setCursor(7, 2);
    lcd.print(motorRunning ? "ON " : "OFF");
    prevMotorState = motorRunning;
  }

  int barLen = map(motorRunning ? percentSpeed : 0, 0, 100, 0, 20);
  static int prevBarLen = -1;
  if (barLen != prevBarLen) {
    for (int i = 0; i < 20; i++) {
      lcd.setCursor(i, 3);
      if (i < barLen)
        lcd.write(255);
      else
        lcd.print(" ");
    }
    prevBarLen = barLen;
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
