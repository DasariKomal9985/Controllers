#include <Wire.h>

#define PCF_ADDR 0x20
#define RELAY_PIN 50

char keys[4][4] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

String currentInput = "";
String storedPasscode = "";
bool passcodeSet = false;
bool relayActive = false;
unsigned long relayStart = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Relay OFF initially

  Serial.println("Enter 4 digit passcode:");
  Serial.println("Then press D to confirm.");
}

void loop() {
  char key = scanKeypad();

  if (key) {
    Serial.print("Key Pressed: ");
    Serial.println(key);

    if (key == 'C') {
      currentInput = "";
      Serial.println("Input Cleared.");
    }

    else if (key >= '0' && key <= '9') {
      if (currentInput.length() < 4) {
        currentInput += key;
        Serial.print("Current: ");
        Serial.println(currentInput);
      }
    }

    else if (key == 'D') {
      if (currentInput.length() == 4) {
        storedPasscode = currentInput;
        passcodeSet = true;
        currentInput = "";
        Serial.println("Passcode confirmed.");
        Serial.println("Now enter 4-digit passcode and press A to unlock.");
      } else {
        Serial.println("Enter 4 digits before pressing D.");
      }
    }

    else if (key == 'A') {
      if (!passcodeSet) {
        Serial.println("No passcode set. Enter 4 digits and press D.");
      }
      else if (currentInput.length() == 4) {
        if (currentInput == storedPasscode) {
          Serial.println("Correct passcode. Relay ON.");
          digitalWrite(RELAY_PIN, LOW);
          relayStart = millis();
          relayActive = true;
        } else {
          Serial.println("Incorrect passcode.");
        }
        currentInput = "";
      } else {
        Serial.println("Enter 4 digits before pressing A.");
      }
    }
  }

  if (relayActive && (millis() - relayStart >= 3000)) {
    digitalWrite(RELAY_PIN, HIGH); // Turn relay OFF
    Serial.println("Relay OFF after 3 seconds.");
    relayActive = false;
  }
}

void writePCF(byte data) {
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(data);
  Wire.endTransmission();
}

byte readPCF() {
  Wire.requestFrom(PCF_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}

char scanKeypad() {
  for (int row = 0; row < 4; row++) {
    byte out = 0xFF;
    out &= ~(1 << (row + 4));
    writePCF(out);
    delay(2);
    byte in = readPCF();
    for (int col = 0; col < 4; col++) {
      if ((in & (1 << col)) == 0) {
        while ((readPCF() & (1 << col)) == 0); // Wait for release
        return keys[row][col];
      }
    }
  }
  return 0;
}
