#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define PCF_ADDR 0x20
LiquidCrystal_I2C lcd(0x27, 20, 4);

char keys[4][4] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

#define IN1 44
#define IN2 45
#define ENA 3
int rampStep = 5;
int rampDelay = 100;
int speedValue = 0;
int lastSpeed = 50;
String direction = "Stopped";
String inputSpeed = "";
bool showMaxSpeed = false;
bool lastDirState = true;
String lastDirection = "Clockwise";
void setup() {
  Wire.begin();
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  lcd.setCursor(0, 0);
  lcd.print("L298N Driver Control");
  updateLCD();
}

void loop() {
  char key = scanKeypad();
  if (key != 0) {
    Serial.print("Key Pressed: ");
    Serial.println(key);

    if (key >= '0' && key <= '9') {
      inputSpeed += key;
      if (inputSpeed.toInt() > 999) inputSpeed = "";
    }

    else if (key == 'A') {
      if (speedValue == 0) {
        speedValue = 50;
        direction = lastDirection;

        if (lastDirState) {
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
        } else {
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, HIGH);
        }

        for (int s = 50; s <= lastSpeed; s += rampStep) {
          analogWrite(ENA, s);
          speedValue = s;
          updateLCD();
          delay(rampDelay);
        }

        showMaxSpeed = (lastSpeed >= 240);
      } else {
        direction = lastDirection;

        if (lastDirState) {
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
        } else {
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, HIGH);
        }

        analogWrite(ENA, speedValue);
      }
    }



    else if (key == 'B') {
     
      lastDirState = (direction == "Clockwise");
      lastDirection = direction;

      speedValue = 0;
      direction = "Stopped";
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
      showMaxSpeed = false;
    }

    else if (key == '*') {
      if (speedValue > 0) {
        direction = "Clockwise";
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        lastDirState = true;
        lastDirection = direction;
      }
    }

    else if (key == '#') {
      if (speedValue > 0) {
        direction = "AntiClock";
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        lastDirState = false;
        lastDirection = direction;
      }
    }

    else if (key == 'C') {
      if (inputSpeed.length() > 0) {
        int newSpeed = inputSpeed.toInt();
        if (newSpeed > 240) newSpeed = 240;
        speedValue = newSpeed;
        lastSpeed = newSpeed;
        analogWrite(ENA, speedValue);
        showMaxSpeed = (speedValue >= 240);
      }
      inputSpeed = "";
    }

    updateLCD();
    delay(200);
  }
}

void updateLCD() {
  lcd.setCursor(0, 1);
  lcd.print("Direction: ");
  lcd.print(direction);
  lcd.print("     ");

  lcd.setCursor(0, 2);
  lcd.print("Speed: ");
  if (inputSpeed.length() > 0) {
    lcd.print(inputSpeed.toInt());
  } else {
    lcd.print(speedValue);
  }
  lcd.print("     ");

  lcd.setCursor(0, 3);
  if (showMaxSpeed) {
    lcd.print("Max Speed Reached   ");
  } else {
    int displaySpeed = (inputSpeed.length() > 0) ? inputSpeed.toInt() : speedValue;
    int bars = map(displaySpeed, 0, 255, 0, 20);
    for (int i = 0; i < bars; i++) lcd.write(byte(255));
    for (int i = bars; i < 20; i++) lcd.print(" ");
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
