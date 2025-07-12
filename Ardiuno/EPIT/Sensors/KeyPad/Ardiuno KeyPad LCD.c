#include <Wire.h>
#include <LiquidCrystal_I2C.h>




#define PCF_ADDR 0x20




LiquidCrystal_I2C lcd(0x27, 20, 4);


char keys[4][4] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};


String typedText = "";


byte arrowChar[8] = {
  0b00100,
  0b01110,
  0b11111,
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b00000
};




const char* keyLine = "123456789 ABCD *# 0";


void setup() {
  Wire.begin();
  Serial.begin(9600);
 
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, arrowChar);


  lcd.setCursor(0, 0); lcd.print("KeyPad");
  lcd.setCursor(0, 1); lcd.print(keyLine);
  drawArrow(' ');
  updateTypedLine();
}


void loop() {
  char key = scanKeypad();
  if (key != 0) {
    Serial.print("Key Pressed: ");
    Serial.println(key);


    typedText += key;
    drawArrow(key);
    updateTypedLine();
    delay(200);
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


void drawArrow(char key) {
  lcd.setCursor(0, 2);
  lcd.print("                    ");


  int pos = getKeyPosition(key);
  if (pos >= 0 && pos < 20) {
    lcd.setCursor(pos, 2);
    lcd.write(byte(0));
  }
}


void updateTypedLine() {
  lcd.setCursor(0, 3);
  lcd.print("                    ");


  int len = typedText.length();
  if (len > 20) {
    lcd.setCursor(0, 3);
    lcd.print(typedText.substring(len - 20));
  } else {
    lcd.setCursor(0, 3);
    lcd.print(typedText);
  }
}


int getKeyPosition(char key) {
  for (int i = 0; i < 20; i++) {
    if (keyLine[i] == key) {
      return i;
    }
  }
  return -1;
}
