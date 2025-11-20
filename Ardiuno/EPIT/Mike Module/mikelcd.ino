
#include <Wire.h>
#include <LiquidCrystal_I2C.h> 
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);
int SoundSensor = A4;
const int sampleWindow = 50;
int sample;
byte musicNote[] = {
  B00100,
  B00110,
  B00101,
  B00101,
  B00100,
  B11100,
  B11100,
  B00000
};
byte soundWave1[] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
  B00000
};
byte soundWave2[] = {
  B00000,
  B00000,
  B11000,
  B11000,
  B11000,
  B11000,
  B11000,
  B00000
};
byte soundWave3[] = {
  B00000,
  B01100,
  B01100,
  B01100,
  B01100,
  B01100,
  B00000,
  B00000
};
byte soundWave4[] = {
  B00110,
  B00110,
  B00110,
  B00110,
  B00110,
  B00110,
  B00110,
  B00110
};
void setup() {
  Serial.begin(9600);
  pinMode(SoundSensor, INPUT);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.createChar(0, musicNote);
  lcd.createChar(1, soundWave1);
  lcd.createChar(2, soundWave2);
  lcd.createChar(3, soundWave3);
  lcd.createChar(4, soundWave4);
}
void loop() {
  unsigned long startMillis = millis();
  float peakToPeak = 0;
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
  while (millis() - startMillis < sampleWindow) {
    sample = analogRead(SoundSensor);
    if (sample < 1024) {
      if (sample > signalMax) signalMax = sample;
      if (sample < signalMin) signalMin = sample;
    }
  }
  peakToPeak = signalMax - signalMin;
  int16_t db = map(peakToPeak, 20, 600, 50, 90);
  db = constrain(db, 50, 90);
  if (db <= 55) {
    db = 52;
    lcd.setCursor(0, 0);
    lcd.print("Loudness: ");
    lcd.print(db);
    lcd.print("dB   "); 
    lcd.setCursor(0, 1);
    lcd.print("Level: Quiet     ");
  } else if (db > 57 && db < 80) {
    lcd.setCursor(0, 0);
    lcd.print("Loudness: ");
    lcd.print(db);
    lcd.print("dB   "); 
    lcd.setCursor(0, 1);
    lcd.print("Level: Moderate  ");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Loudness: ");
    lcd.print(db);
    lcd.print("dB   "); 
    lcd.setCursor(0, 1);
    lcd.print("Level: High      ");
  }
  int numElements = map(db, 50, 90, 1, 18);
  numElements = constrain(numElements, 1, 18);
  lcd.setCursor(0, 2);
  lcd.write(byte(0));
  for (int i = 1; i <= 18; i++) {
    lcd.setCursor(i, 3);
    if (i <= numElements) {
      int waveType;
      if (i % 4 == 0) waveType = 4;
      else if (i % 3 == 0) waveType = 3;
      else if (i % 2 == 0) waveType = 2;
      else waveType = 1;
      lcd.write(byte(waveType));
    } else {
      lcd.print(" ");
    }
  }
  delay(100);
}
