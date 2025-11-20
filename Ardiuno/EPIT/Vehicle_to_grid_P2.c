
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include "DHT.h"

#include <EmonLib.h> // Include EmonLib for current sensing

SoftwareSerial bluetooth(2, 3); // RX, TX
EnergyMonitor emon1;
const float currentThreshold = 0.9; // Set threshold current in Amps (adjust as needed)


const int motorA_in1 = 4;   
const int motorA_in2 = 5;   
const int motorB_in3 = 6;   
const int motorB_in4 = 7;   
const int motorA_enable = 9; 
const int motorB_enable = 10; 

// const int voltagePin = A0;  
const int voltageSensorPin = A0;     // Voltage sensor analog output

const int dhtPin = A1;     
 float motorVoltage ;
// Calibration constants for voltage sensor
const float voltageSensorScale = 5.0 / 1023.0; // Adjust if your sensor differs
const float voltageDividerRatio = 9; // Adjust based on sensor specifications

#define DHTTYPE DHT11  
DHT dht(dhtPin, DHTTYPE);

LiquidCrystal_I2C lcd(0x27, 16, 2);

float voltage = 0;
float current = 0;
float temperature = 0;
float humidity = 0;
int motorSpeedForwardBackward = 70;  // Speed for forward/backward
int motorSpeedRightLeft = 110;        // Speed for turning right/left
// const float voltageDividerRatio = 10000.0/(100000.0 + 10000.0) ;
unsigned long previousMillis = 0;
const long interval = 2000;
  double current_A;
void moveForward();
void moveBackward();
void moveLeft();
void moveRight();
void stopMotors();
void updateLCD();
void readSensors();

void setup() {
  pinMode(motorA_in1, OUTPUT);
  pinMode(motorA_in2, OUTPUT);
  pinMode(motorB_in3, OUTPUT);
  pinMode(motorB_in4, OUTPUT);
  pinMode(motorA_enable, OUTPUT);
  pinMode(motorB_enable, OUTPUT); 
  
  emon1.current(A2, 5); // A2 is the input pin; 26 is the calibration constant

  stopMotors();
  bluetooth.begin(9600);
  Serial.begin(9600);
  Serial.println("Bluetooth Ready! Waiting for commands...");
  lcd.init();
  lcd.backlight();
  lcd.print("System  Ready");
  delay(1000);
  lcd.clear();
  dht.begin();
}

void loop() {
  
  if (bluetooth.available()) {
    char command = bluetooth.read(); 

    switch (command) {
      case 'F':  
        moveForward();
        bluetooth.println("Moving Forward");
        Serial.println("Moving Forward");
        break;
      case 'B': 
        moveBackward();
        bluetooth.println("Moving Backward");
        Serial.println("Moving Backward");
        break;
      case 'R': 
        moveRight();
        bluetooth.println("Turning Right");
        Serial.println("Turning Right");
        break;
      case 'L': 
        moveLeft();
        bluetooth.println("Turning Left");
        Serial.println("Turning Left");
        break;
      case 'S': 
        stopMotors();
        bluetooth.println("Motors Stopped");
        Serial.println("Motors Stopped");
        break;
      default:
        bluetooth.println("Unknown Command");
        Serial.println("Unknown Command");
        break;
    }
  }
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    readSensors();
    updateLCD();
  }
}

void moveForward() {
  analogWrite(motorA_enable, motorSpeedForwardBackward);  // Set speed for forward
  analogWrite(motorB_enable, motorSpeedForwardBackward);  // Set speed for forward
  digitalWrite(motorA_in1, HIGH);
  digitalWrite(motorA_in2, LOW);
  digitalWrite(motorB_in3, HIGH);
  digitalWrite(motorB_in4, LOW);
}

void moveBackward() {
  analogWrite(motorA_enable, motorSpeedForwardBackward);  // Set speed for backward
  analogWrite(motorB_enable, motorSpeedForwardBackward);  // Set speed for backward
  digitalWrite(motorA_in1, LOW);
  digitalWrite(motorA_in2, HIGH);
  digitalWrite(motorB_in3, LOW);
  digitalWrite(motorB_in4, HIGH);
}

void moveRight() {
  analogWrite(motorA_enable, motorSpeedRightLeft);  // Set speed for turning
  analogWrite(motorB_enable, motorSpeedRightLeft);  // Set speed for turning
  digitalWrite(motorA_in1, HIGH);
  digitalWrite(motorA_in2, LOW);
  digitalWrite(motorB_in3, LOW);
  digitalWrite(motorB_in4, HIGH);
}

void moveLeft() {
  analogWrite(motorA_enable, motorSpeedRightLeft);  // Set speed for turning
  analogWrite(motorB_enable, motorSpeedRightLeft);  // Set speed for turning
  digitalWrite(motorA_in1, LOW);
  digitalWrite(motorA_in2, HIGH);
  digitalWrite(motorB_in3, HIGH);
  digitalWrite(motorB_in4, LOW);
}

void stopMotors() {
  analogWrite(motorA_enable, 0);  // Stop motor A
  analogWrite(motorB_enable, 0);  // Stop motor B
  digitalWrite(motorA_in1, LOW);
  digitalWrite(motorA_in2, LOW);
  digitalWrite(motorB_in3, LOW);
  digitalWrite(motorB_in4, LOW);
}

void readSensors() {
  // int voltageValue = analogRead(voltagePin);

    int voltageSensorValue = analogRead(voltageSensorPin);
  motorVoltage = voltageSensorValue * voltageSensorScale * voltageDividerRatio;

  // int currentValue = analogRead(currentPin); 
    current_A = emon1.calcIrms(1480); // Use recommended 1480 samples
  current_A = max(0.0, current_A -0.28);  // Offset correction and ensure no negative values


  // voltage = (voltageValue / 1023.0) * 3.1 * voltageDividerRatio; 
  // current = (currentValue / 1023.0) * 5.0;       

  temperature = dht.readTemperature();
  
  if (isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
  }
}

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("V:");
  lcd.print(motorVoltage, 1);
  lcd.print("V C:");
  lcd.print(current_A, 2);
  lcd.print("A");
  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(temperature, 1);
}