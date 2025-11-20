

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// NRF24L01 Configuration
#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

// Joystick Inputs
#define JOY_X A1
#define JOY_Y A0
#define JOY_SW_PIN 2  // Joystick switch pin

// Switch and LED pins
#define SWITCH_PIN 5   // Switch pin
#define LED1_PIN 3     // Red LED
#define LED2_PIN 4     // Green LED

struct MotorData {
  int motor1;
  int motor2;
  int motor3;
  int motor4;
 // bool servoControl;   // Controls servo ON/OFF?
  int joystickToggle;  // Command to send based on switch (0 or 1)
  int specialCommand ;  // Toggles between 0 and 1 with each press
};

MotorData motorData;

bool joystickPressed = false;   // Toggle state variable (for joystick button)
bool currentJoystickState = HIGH;  // Current joystick button state
bool lastJoystickState = HIGH;  // Last state of joystick button
unsigned long lastDebounceTime = 0;  // Last debounce time
const unsigned long debounceDelay = 50;  // Debounce delay

void setup() {
  Serial.begin(9600);
  pinMode(JOY_SW_PIN, INPUT_PULLUP);  // Joystick switch pin with pull-up
  pinMode(SWITCH_PIN, INPUT_PULLUP);  // Switch with pull-up
  pinMode(LED1_PIN, OUTPUT);          // Red LED
  pinMode(LED2_PIN, OUTPUT);          // Green LED

  // Initialize NRF24L01
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();  // Set NRF24L01 to transmit mode

  Serial.println("Transmitter Initialized");
  digitalWrite(LED1_PIN, LOW);  // Ensure LEDs are off initially
  digitalWrite(LED2_PIN, LOW);
}

void loop() {
  // Debouncing the joystick button and toggling state on press
  currentJoystickState = digitalRead(JOY_SW_PIN);

  if (currentJoystickState == LOW && lastJoystickState == HIGH && (millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();  // Update debounce timer
    joystickPressed = !joystickPressed;  // Toggle the joystick state
    motorData.joystickToggle = joystickPressed ? 1 : 0;  // Alternate between 1 and 0
    Serial.print("Joystick Button Toggled: ");
    Serial.println(motorData.joystickToggle);
  }

  lastJoystickState = currentJoystickState;  // Update the last state

  // Read switch state and control LEDs based on the switch position
  bool switchState = digitalRead(SWITCH_PIN);
  if (switchState == LOW) {  // Switch ON (LOW due to pull-up)
    motorData.specialCommand = 0;  // Send 0
    digitalWrite(LED1_PIN, HIGH);  // Red LED ON
    digitalWrite(LED2_PIN, LOW);   // Green LED OFF
    Serial.println("Switch ON: Red LED ON, Command 0 Sent");
  } else {  // Switch OFF
    motorData.specialCommand = 1;  // Send 1
    digitalWrite(LED1_PIN, LOW);   // Red LED OFF
    digitalWrite(LED2_PIN, HIGH);  // Green LED ON
    Serial.println("Switch OFF: Green LED ON, Command 1 Sent");
  }

  // Motor control logic based on joystick position
  int xValue = analogRead(JOY_X);
  int yValue = analogRead(JOY_Y);

  if (yValue < 400) {
    motorData.motor1 = 1;
    motorData.motor2 = 1;
    motorData.motor3 = 1;
    motorData.motor4 = 1;
  } else if (yValue > 600) {
    motorData.motor1 = -1;
    motorData.motor2 = -1;
    motorData.motor3 = -1;
    motorData.motor4 = -1;
  } else if (xValue < 400) {
    motorData.motor1 = -1;
    motorData.motor2 = 1;
    motorData.motor3 = -1;
    motorData.motor4 = 1;
  } else if (xValue > 600) {
    motorData.motor1 = 1;
    motorData.motor2 = -1;
    motorData.motor3 = 1;
    motorData.motor4 = -1;
  } else {
    motorData.motor1 = 0;
    motorData.motor2 = 0;
    motorData.motor3 = 0;
    motorData.motor4 = 0;
  }

  // Debugging output for motor data and special command
  Serial.print("M1: "); Serial.print(motorData.motor1);
  Serial.print(" | M2: "); Serial.print(motorData.motor2);
  Serial.print(" | M3: "); Serial.print(motorData.motor3);
  Serial.print(" | M4: "); Serial.print(motorData.motor4);
  Serial.print(" | Special Command: "); Serial.print(motorData.specialCommand);
  Serial.print(" | Joystick Toggle: "); Serial.println(motorData.joystickToggle);

  // Send motor data to the receiver
  radio.write(&motorData, sizeof(motorData));

  delay(100);  // Transmit every 100 ms
}
