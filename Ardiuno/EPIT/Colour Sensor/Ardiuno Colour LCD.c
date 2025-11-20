#include <Wire.h>
#include <LiquidCrystal_I2C.h>
const int S0 = 26;
const int S1 = 27;
const int S2 = 28;
const int S3 = 29;
const int sensorOut = 30;
const int ledPin = 31;
LiquidCrystal_I2C lcd(0x27, 20, 4);
unsigned long redFrequency = 0;
unsigned long greenFrequency = 0;
unsigned long blueFrequency = 0;
unsigned long lastRed = 0;
unsigned long lastGreen = 0;
unsigned long lastBlue = 0;
String lastColor = "";
void setup() {
  Serial.begin(9600);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  digitalWrite(ledPin, HIGH);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Colour Sensor");
}
void loop() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW);
  String colorName = detectColor(redFrequency, greenFrequency, blueFrequency);
  if (redFrequency != lastRed || greenFrequency != lastGreen || blueFrequency != lastBlue) {
    lastRed = redFrequency;
    lastGreen = greenFrequency;
    lastBlue = blueFrequency;
    lcd.setCursor(0, 1);
    lcd.print("R:");
    lcd.print(redFrequency);
    lcd.print(" G:");
    lcd.print(greenFrequency);
    lcd.print(" B:");
    lcd.print(blueFrequency);
    lcd.print("   ");
  }
  if (colorName != lastColor) {
    lastColor = colorName;
    lcd.setCursor(0, 2);
    lcd.print("Colour           ");
    lcd.setCursor(0, 3);
    lcd.print("                ");
    lcd.setCursor(6, 3);
    lcd.print(colorName);
  }
  Serial.print("R: ");
  Serial.print(redFrequency);
  Serial.print(" | G: ");
  Serial.print(greenFrequency);
  Serial.print(" | B: ");
  Serial.print(blueFrequency);
  Serial.print(" | Detected: ");
  Serial.println(colorName);
  delay(1000);
}
String detectColor(unsigned long r, unsigned long g, unsigned long b) {
  if (r > 120 && r < 150 && g > 1800 && g < 2000 && b > 950 && b < 2000) return "RED";
  if (r > 300 && r < 400 && g > 180 && g < 300 && b > 50 && b < 100) return "BLUE";
  if ((r > 180 && r < 250 && g > 120 && g < 170 && b > 280 && b < 400) || (r > 1000 && r < 3000 && g > 500 && g < 800 && b > 1000 && b < 3000)) return "GREEN";
  if (r > 20 && r < 90 && g > 110 && g < 160 && b > 200 && b < 300) return "YELLOW";
  if (r > 220 && r < 290 && g > 350 && g < 430 && b > 350 && b < 450) return "PINK";
  if (r > 200 && r < 250 && g > 240 && g < 300 && b > 260 && b < 310) return "WHITE";
  return "UNKNOWN";
}
