#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define JOYSTICK_X A0
#define JOYSTICK_Y A1

#define X_MIN 320
#define X_CENTER 640
#define X_MAX 1000

#define Y_MIN 320
#define Y_CENTER 650
#define Y_MAX 1000

#define TOLERANCE 5

LiquidCrystal_I2C lcd(0x27, 20, 4);

int prevX = 999;
int prevY = 999;
String prevDirection = "";

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("   Joystick Module");
}

void loop() {
  int rawY = analogRead(JOYSTICK_X);
  int rawX = analogRead(JOYSTICK_Y);

  int mappedX = (rawX < X_CENTER)
                  ? map(rawX, X_MIN, X_CENTER, -50, 0)
                  : map(rawX, X_CENTER, X_MAX, 0, 50);

  int mappedY = (rawY < Y_CENTER)
                  ? map(rawY, Y_MIN, Y_CENTER, -50, 0)
                  : map(rawY, Y_CENTER, Y_MAX, 0, 50);

  if (abs(mappedX) < TOLERANCE) mappedX = 0;
  if (abs(mappedY) < TOLERANCE) mappedY = 0;

  String direction = "Stable";
  if (mappedY > 10) direction = "FORWARD";
  else if (mappedY < -10) direction = "BACKWARD";
  else if (mappedX > 10) direction = "RIGHT";
  else if (mappedX < -10) direction = "LEFT";

  if (abs(mappedX - prevX) > TOLERANCE) {
    lcd.setCursor(0, 1);
    lcd.print("X: ");
    lcd.print(mappedX);
    lcd.print("     ");
    prevX = mappedX;
  }

  if (abs(mappedY - prevY) > TOLERANCE) {
    lcd.setCursor(0, 2);
    lcd.print("Y: ");
    lcd.print(mappedY);
    lcd.print("     ");
    prevY = mappedY;
  }

  if (direction != prevDirection) {
    lcd.setCursor(0, 3);
    lcd.print("Status: ");
    lcd.print(direction);
    lcd.print("       ");
    prevDirection = direction;
  }

  delay(100);
}
