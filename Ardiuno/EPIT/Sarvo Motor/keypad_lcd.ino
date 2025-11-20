#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);
Servo myServo;
int currentAngle = 90;

#define PCF_ADDR 0x20

char keys[4][4] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

String inputStr = "";
String lastCommand = "";

void setup() {
  Wire.begin();
  lcd.init();
  lcd.backlight();

  myServo.attach(13);
  myServo.write(currentAngle);

  lcd.setCursor(0, 0);
  lcd.print("Servo Control");
  updateLCD();
}

void loop() {
  char key = scanKeypad();

  if (key != 0) {
    if (key == 'C') {
      // confirm entered angle
      int angle = inputStr.toInt();
      if (angle >= 0 && angle <= 180) {
        if (angle != currentAngle) {
          currentAngle = angle;
          myServo.write(currentAngle);
          lastCommand = "Moved to " + String(angle);
        } else {
          lastCommand = "Same angle " + String(angle);
        }
      } else {
        lastCommand = "Invalid Angle";
      }
      inputStr = "";
    } else if (key == 'A' || key == 'B' || key == 'D') {
      // ignore other special keys
    } else if (key >= '0' && key <= '9') {
      inputStr += key;
    }
    updateLCD();
    delay(200);
  }
}

void updateLCD() {
  lcd.setCursor(0, 1);
  lcd.print("Enter: ");
  lcd.print(inputStr);
  lcd.print("       ");

  lcd.setCursor(0, 2);
  lcd.print("Last: ");
  lcd.print(lastCommand);
  lcd.print("        ");

  lcd.setCursor(0, 3);
  int blocks = map(currentAngle, 0, 180, 0, 20);
  for (int i = 0; i < 20; i++) {
    if (i < blocks)
      lcd.write(byte(255));
    else
      lcd.print(" ");
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
