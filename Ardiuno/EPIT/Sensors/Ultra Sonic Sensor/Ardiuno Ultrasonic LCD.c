#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LiquidCrystal_I2C.h>


#define TRIG_PIN 23
#define ECHO_PIN 22


LiquidCrystal_I2C lcd(0x27, 20, 4);
int distance = 100;




byte sensorTopLeft[8] = {
  0b00011, 0b00011, 0b00011, 0b00011,
  0b00011, 0b00011, 0b00011, 0b00011
};
byte sensorTopRight[8] = {
  0b11000, 0b11100, 0b11110, 0b11111,
  0b11111, 0b11110, 0b11100, 0b11000
};
byte sensorBottomLeft[8] = {
  0b00011, 0b00011, 0b00011, 0b00011,
  0b00011, 0b00011, 0b00011, 0b00011
};
byte sensorBottomRight[8] = {
  0b11000, 0b11100, 0b11110, 0b11111,
  0b11111, 0b11110, 0b11100, 0b11000
};


byte objectTopLeft[8] = {
  0b00000, 0b00000, 0b00000, 0b00000,
  0b00100, 0b01110, 0b11111, 0b11111
};


byte objectBottomLeft[8] = {
  0b11111, 0b11111, 0b01110, 0b00100,
  0b00000, 0b00000, 0b00000, 0b00000
};




void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, sensorTopLeft);
  lcd.createChar(1, sensorTopRight);
  lcd.createChar(2, sensorBottomLeft);
  lcd.createChar(3, sensorBottomRight);
  lcd.createChar(4, objectTopLeft);
  lcd.createChar(6, objectBottomLeft);
  lcd.clear();
}


void loop() {
  distance = Read_Distance_with_Ultrasonic_LCD();
  Serial_Monitor_Output(distance);
  Pixels_On_LCD(distance);
  delay(100);
}


int Read_Distance_with_Ultrasonic_LCD() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  int dist = duration * 0.0343 / 2;
  return constrain(dist, 0, 100);
}


void Serial_Monitor_Output(int distance) {
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}




void defineLCDChars() {
  lcd.setCursor(0, 2);
  lcd.write(byte(0));
  lcd.setCursor(1, 2);
  lcd.write(byte(1));
  lcd.setCursor(0, 3);
  lcd.write(byte(2));
  lcd.setCursor(1, 3);
  lcd.write(byte(3));


  int objCol = map(distance, 0, 100, 3, 19);


  for (int i = 3; i < 20; i++) {
    lcd.setCursor(i, 2);
    lcd.print(" ");
    lcd.setCursor(i, 3);
    lcd.print(" ");
  }


  if (objCol < 20) {
    lcd.setCursor(objCol, 2);
    lcd.write(byte(4));


    lcd.setCursor(objCol, 3);
    lcd.write(byte(6));
  }


  delay(300);
}
void Pixels_On_LCD(int distance) {
  lcd.setCursor(0, 0);
  lcd.print("UltraSonic Sensor");
  lcd.setCursor(0, 1);
  lcd.print("Distance = ");
  lcd.print(distance);
  lcd.print(" cm     ");
  defineLCDChars();
}


