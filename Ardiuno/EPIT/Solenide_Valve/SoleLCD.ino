#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

#define PCF_ADDR 0x20
#define RELAY_PIN 50

char keys[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

String enteredCode = "";
String storedCode = "";
bool codeSet = false;
String doorStatus = "Door Closed";
String messageStatus = "";

void setup() {
  Wire.begin();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  updateLCD();
}

void loop() {
  char key = scanKeypad();

  if (key != 0) {
    Serial.print("Key Pressed: ");
    Serial.println(key);

    if (key >= '0' && key <= '9') {
      if (enteredCode.length() < 4) {
        enteredCode += key;
        updateLCD();
      }
    }
    else if (key == 'C') {
      enteredCode = "";
      messageStatus = "Code Cleared";
      updateLCD();
    }
    else if (key == 'D') {
      if (enteredCode.length() == 4) {
        storedCode = enteredCode;
        codeSet = true;
        enteredCode = "";
        messageStatus = "Passcode Changed";
        Serial.println("Passcode set");
        updateLCD();
      } else {
        messageStatus = "Enter 4 Digits";
        Serial.println("Passcode must be 4 digits");
        updateLCD();
      }
    }
    else if (key == 'A') {
      if (codeSet && enteredCode == storedCode) {
        Serial.println("Correct passcode - Relay ON");
        digitalWrite(RELAY_PIN, LOW);
        doorStatus = "Door Open";
        messageStatus = "Correct Passcode";
        updateLCD();
        delay(5000);
        digitalWrite(RELAY_PIN, HIGH);
        doorStatus = "Door Closed";
        updateLCD();
      } else {
        messageStatus = "Passcode Wrong";
        Serial.println("Wrong Passcode");
        updateLCD();
      }
      enteredCode = "";
    }

    while (scanKeypad() != 0);
    delay(100);
  }
}

void writePCF(byte data) {
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(data);
  Wire.endTransmission();
}

byte readPCF() {
  Wire.requestFrom(PCF_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
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

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Selenide Valve");

  lcd.setCursor(0, 1);
  lcd.print("Enter Passcode: ");
  for (int i = 0; i < 4; i++) {
    if (i < enteredCode.length()) lcd.print(enteredCode[i]);
    else lcd.print('_');
  }

  lcd.setCursor(0, 2);
  lcd.print(messageStatus);

  lcd.setCursor(0, 3);
  lcd.print("Status : ");
  lcd.print(doorStatus);
}
