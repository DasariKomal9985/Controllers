int buttons[] = { 19, 26, 13, 18, 33, 14, 5, 32, 25 };
int buttonID[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
int total = sizeof(buttons) / sizeof(buttons[0]);

int lastStableState[20];

int readStable(int pin) {
  int a = digitalRead(pin);
  delay(5);
  int b = digitalRead(pin);
  delay(5);
  int c = digitalRead(pin);

  if (a == b && b == c)
    return a;
  return HIGH;
}

void setup() {
  Serial.begin(115200);

  Serial2.begin(115200, SERIAL_8N1, 16, 17);  // RX=16, TX=17

  for (int i = 0; i < total; i++) {
    pinMode(buttons[i], INPUT_PULLUP);
    lastStableState[i] = HIGH;
  }

  Serial.println("Button Sender Ready...");
}

void loop() {

  // Read full message from STM32
  if (Serial2.available()) {
    String msg = Serial2.readStringUntil('\n');
    Serial.print("STM32: ");
    Serial.println(msg);
  }

  // Detect button presses and send to STM32
  for (int i = 0; i < total; i++) {

    int state = readStable(buttons[i]);

    if (lastStableState[i] == HIGH && state == LOW) {
      Serial.print("Pressed: ");
      Serial.println(buttonID[i]);

      Serial2.print("BTN:");
      Serial2.println(buttonID[i]);
    }

    lastStableState[i] = state;
  }
}
