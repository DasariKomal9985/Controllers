#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define RELAY_PIN 43

LiquidCrystal_I2C lcd(0x27, 20, 4);

bool relayState = false;
unsigned long lastToggleTime = 0;
const unsigned long toggleInterval = 2000;

void setup() {
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Relay Module");

  lcd.setCursor(0, 1);
  lcd.print("Status");

  updateRelayDisplay();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastToggleTime >= toggleInterval) {
    relayState = !relayState;
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
    lastToggleTime = currentMillis;

    updateRelayDisplay();
  }

  delay(100);
}

void updateRelayDisplay() {
  lcd.setCursor(0, 2);
  lcd.print("                  ");
  lcd.setCursor(0, 2);
  lcd.print(relayState ? "ON" : "OFF");

  lcd.setCursor(0, 3);
  lcd.print("[ ]              [ ]");

  if (relayState) {
    lcd.setCursor(18, 3);
    lcd.print("#");
  } else {
    lcd.setCursor(1, 3);
    lcd.print("#");
  }

  Serial.println(relayState ? "Relay ON" : "Relay OFF");
}
