#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define SWITCH_PIN 41
#define LED_PIN 33

LiquidCrystal_I2C lcd(0x27, 20, 4);

bool lastSwitchState = HIGH;

void setup() {
  Serial.begin(9600);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  lcd.init();
  lcd.backlight();
  drawStaticLayout();
  Serial.println("System Initialized. Waiting for switch input...");
}

void loop() {
  bool switchState = digitalRead(SWITCH_PIN);

  if (switchState != lastSwitchState) {
    lastSwitchState = switchState;
    String status = switchState ? "OFF" : "ON";
    Serial.print("Switch State Changed: ");
    Serial.println(status);
    updateStatus(status);
    digitalWrite(LED_PIN, switchState ? LOW : HIGH);
  }

  delay(100);
}

void drawStaticLayout() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Rocker Switch");
  lcd.setCursor(0, 1);
  lcd.print("Status");
  updateStatus("OFF");
}

void updateStatus(String statusText) {
  lcd.setCursor(0, 2);
  lcd.print("                    ");
  lcd.setCursor(0, 2);
  lcd.print(statusText);

  lcd.setCursor(0, 3);
  lcd.print("[");
  lcd.setCursor(2, 3);
  lcd.print("                ");
  lcd.setCursor(2, 3);
  lcd.print(statusText == "ON" ? "LED ON" : "LED OFF");
  lcd.setCursor(19, 3);
  lcd.print("]");
}
