#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

byte rowPins[ROWS] = { 37, 38, 39, 40 };
byte colPins[COLS] = { 33, 34, 35, 36 };

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

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
  char key = keypad.getKey();

  if (key != NO_KEY) {
    Serial.print("Key Pressed: ");
    Serial.println(key);

    typedText += key;
    drawArrow(key);
    updateTypedLine();
    delay(150);
  }
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
