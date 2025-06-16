#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// Motor driver pins
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5
#define ENA 6
#define ENB 10

// Bluetooth (TX=9, RX=8)
SoftwareSerial bluetooth(8, 9);

// LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);

String lastMessage = "";
unsigned long lastCommandTime = 0;
unsigned long actionStartTime = 0;
const unsigned long timeout = 3000;       // 3 seconds idle display
const unsigned long actionDuration = 5000; // 5 seconds motor action

char currentAction = 'S'; // 'F', 'B', 'L', 'R', 'S'

void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  analogWrite(ENA, 150);
  analogWrite(ENB, 150);

  bluetooth.begin(9600);
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Welcome to");
  lcd.setCursor(0, 1);
  lcd.print("Voice Ctrl Car");
  delay(2000);

  displayMessage("Waiting for", "voice commands");
}

void loop() {
  if (bluetooth.available()) {
    char command = bluetooth.read();
    Serial.print("Received via Bluetooth: ");
    Serial.println(command);
    lastCommandTime = millis();

    // Handle voice control (lowercase)
    switch (command) {
      case 'f': Voiceforward(); break;
      case 'b': Voicebackward(); break;
      case 'l': VoiceturnLeft(); break;
      case 'r': VoiceturnRight(); break;
      case 's': VoicestopCar(); break;

      // Handle manual control (uppercase)
      case 'F': Manualforward(); break;
      case 'B': Manualbackward(); break;
      case 'L': ManualturnLeft(); break;
      case 'R': ManualturnRight(); break;
      case 'S': ManualstopCar(); break;

      default: displayMessage("Unknown", "Command"); break;
    }
  }

  // Voice control: auto-stop after 5 seconds
  if (currentAction != 'S' && millis() - actionStartTime >= actionDuration) {
    stopCar(); // Stops after 5 sec in voice mode
  }

  // Idle display after 3 seconds
  if (currentAction == 'S' && millis() - lastCommandTime > timeout) {
    displayMessage("Waiting for", "voice commands");
  }
}

// === Display ===
void displayMessage(String top, String bottom) {
  String message = top + "|" + bottom;
  if (message != lastMessage) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(top);
    lcd.setCursor(0, 1);
    lcd.print(bottom);
    lastMessage = message;
  }
}

// === Shared Stop Function ===
void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  displayMessage("Car", "Stopped");
  currentAction = 'S';
}

// === Voice Control (5 sec) ===
void Voiceforward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  displayMessage("Voice", "Forward");
  actionStartTime = millis();
  currentAction = 'F';
}

void Voicebackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  displayMessage("Voice", "Backward");
  actionStartTime = millis();
  currentAction = 'B';
}

void VoiceturnLeft() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  displayMessage("Voice", "Left");
  actionStartTime = millis();
  currentAction = 'L';
}

void VoiceturnRight() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  displayMessage("Voice", "Right");
  actionStartTime = millis();
  currentAction = 'R';
}

void VoicestopCar() {
  stopCar(); // reuse shared stop
}

// === Manual Control (continuous) ===
void Manualforward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  displayMessage("Manual", "Forward");
  currentAction = 'S'; // No auto stop
}

void Manualbackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  displayMessage("Manual", "Backward");
  currentAction = 'S';
}

void ManualturnLeft() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  displayMessage("Manual", "Left");
  currentAction = 'S';
}

void ManualturnRight() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  displayMessage("Manual", "Right");
  currentAction = 'S';
}

void ManualstopCar() {
  stopCar(); // reuse shared stop
}
