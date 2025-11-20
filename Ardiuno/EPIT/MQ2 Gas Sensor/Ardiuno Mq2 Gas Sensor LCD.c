#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

const int mq2Pin = A2;
const int threshold = 5;

int rawValue = 0;
int gasLevel = 0;
bool isDetected = false;
int lastLevel = -1;

void setup() {
  initializeSystem();
  lcd.setCursor(0, 0);
  lcd.print("MQ2 Gas Sensor");
}

void loop() {
  rawValue = analogRead(mq2Pin);
  gasLevel = map(rawValue, 0, 1023, 0, 100);
  gasLevel = constrain(gasLevel, 0, 100);
  isDetected = gasLevel > threshold;

  logToSerial();
  updateGasDisplay();
  delay(1000);
}

void initializeSystem() {
  Wire.begin();
  lcd.init();
  lcd.backlight();
  pinMode(mq2Pin, INPUT);
  Serial.begin(9600);
}

void updateGasDisplay() {
  if (gasLevel != lastLevel) {
    lcd.setCursor(0, 1);
    lcd.print("Gas Level : ");
    lcd.print("     ");
    lcd.setCursor(13, 1);
    lcd.print(gasLevel);
    lcd.print("%");

    lcd.setCursor(0, 2);
    lcd.print("Status :");
    
    lcd.setCursor(0, 3);
    lcd.print("                    "); 
    
    lcd.setCursor(7, 3);
    lcd.print(isDetected ? "Detected" : "Not Detected");

    lastLevel = gasLevel;
  }
}

void logToSerial() {
  Serial.print("Gas Level: ");
  Serial.print(gasLevel);
  Serial.println(" %");
}
