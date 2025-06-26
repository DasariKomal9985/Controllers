
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EmonLib.h>
#include <WiFi.h>
#include <FirebaseESP32.h>

// Motor control pins
const int motorA_in1 = 4;
const int motorA_in2 = 5;
const int motorB_in3 = 14;
const int motorB_in4 = 27;
const int motorA_enable = 12;
const int motorB_enable = 13;

// Voltage and Current Calibration
EnergyMonitor emon1;
const int voltageSensorPin = 35;
const int currentSensorPin = 34;  // ACS712 sensor pin
float motorVoltage;
const float voltageSensorScale = 3.3 / 4095.0;
const float voltageDividerRatio = 5.4;
unsigned long previousMillis = 0;
double current_A;

//Firebase
#define WIFI_SSID "OTA"
#define WIFI_PASSWORD "135792468"
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#define API_KEY "AIzaSyAasvBq72dZRJy9ARtKGigchSyKrGJjBKc"
#define DATABASE_URL "https://vehicle-to-grid-09-march-2025-default-rtdb.asia-southeast1.firebasedatabase.app/"  //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app
#define USER_EMAIL "vehicletogrid_09_march_2025@gmail.com"
#define USER_PASSWORD "123456789"
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long sendDataPrevMillis = 0;
int count = 0;

// LCD initialization
LiquidCrystal_I2C lcd(0x27, 16, 2);
char lastCommand = 'S';
const long interval = 1000;

// ACS712 Calibration
const float ACS712_SENSITIVITY = 0.185; // 185mV/A for ACS712-5A (use 0.1 for 20A, 0.066 for 30A)
const float ACS712_ZERO_OFFSET = 2.5;   // 2.5V offset for bidirectional measurement

void setup() {
  pinMode(motorA_in1, OUTPUT);
  pinMode(motorA_in2, OUTPUT);
  pinMode(motorB_in3, OUTPUT);
  pinMode(motorB_in4, OUTPUT);
  pinMode(motorA_enable, OUTPUT);
  pinMode(motorB_enable, OUTPUT);
  
  emon1.current(currentSensorPin, 2);
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  lcd.init();
  lcd.backlight();

  // Display welcome message for 5 seconds
  lcd.setCursor(0, 0);
  lcd.print("Welcome To");
  lcd.setCursor(0, 1);
  lcd.print("Vehicle_to_grid");
  delay(5000);

  // Switch to standard display
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Vehicle_to_grid");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Waiting");
    delay(300);
  }

  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the user sign in credentials */
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback;  //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
}

void loop() {
  if (Serial2.available()) {
    char command = Serial2.read();
    if (command == 'F' || command == 'B' || command == 'L' || command == 'R' || command == 'S') {
      lastCommand = command;
    }
  }

  switch (lastCommand) {
    case 'F': moveForward(); break;
    case 'B': moveBackward(); break;
    case 'R': moveRight(); break;
    case 'L': moveLeft(); break;
    case 'S': stopMotors(); break;
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    readSensors();
  }
  if (Firebase.ready() && (millis() - sendDataPrevMillis > 1000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();
    FirebaseJson json;
   // json.add("Temperature", temperature);
    json.add("Voltage", motorVoltage);
    json.add("Current", current_A);
   // json.set("Temperature", temperature);
    json.set("Voltage", motorVoltage);
    json.set("Current", current_A);
    Serial.printf("Update json... %s\n\n", Firebase.updateNode(fbdo, String("/Vehicle_to_Grid/Data/" + fbdo.pushName()), json) ? "ok" : fbdo.errorReason().c_str());
  }
}

void readSensors() {
  // Read Voltage
  int voltageSensorValue = analogRead(voltageSensorPin);
  motorVoltage = (voltageSensorValue * voltageSensorScale) * voltageDividerRatio;

  // Read Current from ACS712
  int rawADC = analogRead(currentSensorPin);
  float sensorVoltage = rawADC * (3.3 / 4095.0); // Convert ADC to voltage
  current_A = (sensorVoltage - ACS712_ZERO_OFFSET) / ACS712_SENSITIVITY; // Convert to Amps

  // Ensure current does not go negative
  if (current_A < 0) {
    current_A = 0;
  }

  // Display on LCD
  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(motorVoltage, 2);
  lcd.print("V ");
  lcd.print("C:");
  lcd.print(current_A, 2);
  lcd.print("A   ");

  // Print to Serial Monitor
  Serial.print("Voltage: ");
  Serial.print(motorVoltage, 2);
  Serial.print(" V, Current: ");
  Serial.print(current_A, 2);
  Serial.println(" A");
}

void moveForward() {
  analogWrite(motorA_enable, 100);
  analogWrite(motorB_enable, 100);
  digitalWrite(motorA_in1, HIGH);
  digitalWrite(motorA_in2, LOW);
  digitalWrite(motorB_in3, HIGH);
  digitalWrite(motorB_in4, LOW);
}

void moveBackward() {
  analogWrite(motorA_enable, 100);
  analogWrite(motorB_enable, 100);
  digitalWrite(motorA_in1, LOW);
  digitalWrite(motorA_in2, HIGH);
  digitalWrite(motorB_in3, LOW);
  digitalWrite(motorB_in4, HIGH);
}

void moveRight() {
  analogWrite(motorA_enable, 150);
  analogWrite(motorB_enable, 150);
  digitalWrite(motorA_in1, HIGH);
  digitalWrite(motorA_in2, LOW);
  digitalWrite(motorB_in3, LOW);
  digitalWrite(motorB_in4, HIGH);
}

void moveLeft() {
  analogWrite(motorA_enable, 150);
  analogWrite(motorB_enable, 150);
  digitalWrite(motorA_in1, LOW);
  digitalWrite(motorA_in2, HIGH);
  digitalWrite(motorB_in3, HIGH);
  digitalWrite(motorB_in4, LOW);
}

void stopMotors() {
  analogWrite(motorA_enable, 0);
  analogWrite(motorB_enable, 0);
  digitalWrite(motorA_in1, LOW);
  digitalWrite(motorA_in2, LOW);
  digitalWrite(motorB_in3, LOW);
  digitalWrite(motorB_in4, LOW);
}
