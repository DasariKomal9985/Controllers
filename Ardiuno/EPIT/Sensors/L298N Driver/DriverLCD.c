#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

#define IN1 44
#define IN2 45
#define IN3 46
#define IN4 47

#define ENA 3
#define ENB 5

int speedValue = 150;
String direction = "Stopped";

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  applySpeed();

  lcd.init();
  lcd.backlight();

  Serial.begin(9600);
  Serial.println("Control with:");
  Serial.println("f = forward, b = backward");
  Serial.println("l = left, r = right, s = stop");
  Serial.println("+= increase speed, -= decrease speed");

  updateLCD();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'f': moveForward(); break;
      case 'b': moveBackward(); break;
      case 'l': turnLeft(); break;
      case 'r': turnRight(); break;
      case 's': stopMotors(); break;
      case '+':
      case '=':
        speedValue += 25;
        if (speedValue > 255) speedValue = 255;
        applySpeed();
        break;
      case '-':
        speedValue -= 25;
        if (speedValue < 0) speedValue = 0;
        applySpeed();
        break;
      default:
        Serial.println("Invalid input");
        break;
    }

    updateLCD();
  }
}

void moveForward() {
  direction = "Forward";
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void moveBackward() {
  direction = "Backward";
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void turnLeft() {
  direction = "Left";
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnRight() {
  direction = "Right";
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void stopMotors() {
  direction = "Stopped";
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void applySpeed() {
  analogWrite(ENA, speedValue);
  analogWrite(ENB, speedValue);
  Serial.print("Speed set to: "); Serial.println(speedValue);
}

void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("L298N Motor Driver  ");

  lcd.setCursor(0, 1);
  lcd.print("Direction: ");
  lcd.print(direction);
  lcd.print("     ");

  lcd.setCursor(0, 2);
  lcd.print("Speed: ");
  lcd.print(speedValue);
  lcd.print("     ");

  lcd.setCursor(0, 3);
  int bars = map(speedValue, 0, 255, 0, 20);
  for (int i = 0; i < bars; i++) {
    lcd.write(byte(255));
  }
  for (int i = bars; i < 20; i++) {
    lcd.print(" ");
  }
}
