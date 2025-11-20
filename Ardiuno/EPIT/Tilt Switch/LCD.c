#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

const int tiltPin = 2;
int boxPos = 0;

byte boxChar[8] = {
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
  pinMode(tiltPin, INPUT_PULLUP);
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.createChar(0, boxChar);

  lcd.setCursor(0, 0);
  lcd.print("Tilt Switch");

  lcd.setCursor(0, 1);
  lcd.print("Status:");
}

void loop() {
  int state = digitalRead(tiltPin);

  lcd.setCursor(0, 2);
  lcd.print("                ");
  lcd.setCursor(0, 2);
  if (state == LOW) {
    Serial.println("Tilted");
    lcd.print("Tilted");
    if (boxPos < 19) boxPos++;
  } else {
    Serial.println("Not Tilted");
    lcd.print("Not Tilted");
    if (boxPos > 0) boxPos--;
  }

  lcd.setCursor(0, 3);
  lcd.print("                    ");
  lcd.setCursor(boxPos, 3);
  lcd.write(byte(0));

  delay(200);
}
