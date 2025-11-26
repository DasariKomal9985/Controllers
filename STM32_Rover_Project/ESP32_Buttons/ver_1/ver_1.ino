int buttons[]  = {19, 26, 13, 18, 33, 14, 5, 32, 25};
int buttonID[] = { 1,  2,  3,  4,  5,  6, 7,  8,  9};
int total = sizeof(buttons) / sizeof(buttons[0]);

int lastStableState[20];
unsigned long lastDebounceTime[20];

int readStable(int pin) {
  int a = digitalRead(pin);
  delay(5);
  int b = digitalRead(pin);
  delay(5);
  int c = digitalRead(pin);

  if (a == b && b == c)
    return a;
  return HIGH;  // default (not pressed)
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < total; i++) {
    pinMode(buttons[i], INPUT_PULLUP);
    lastStableState[i] = HIGH;   // initial not pressed
  }

  Serial.println("Stable Button Test Ready...");
}

void loop() {

  for (int i = 0; i < total; i++) {

    int pin = buttons[i];
    int stableState = readStable(pin);

    // Detect NEW PRESS (HIGH -> LOW)
    if (lastStableState[i] == HIGH && stableState == LOW) {
      Serial.print("Button ID: ");
      Serial.println(buttonID[i]);
    }

    // Save state
    lastStableState[i] = stableState;
  }
}
