#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <Keypad.h>

#define ENA 10
#define DIR 11
#define PUL 12

hd44780_I2Cexp lcd;

char spinner[4] = {'|', '/', '-', '\\'};
int spinIndex = 0;

const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

byte rowPins[ROWS] = { 37, 38, 39, 40 };
byte colPins[COLS] = { 33, 34, 35, 36 };

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  digitalWrite(ENA, LOW);

  int status = lcd.begin(20, 4);
  if (status) while (1);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Microstep Driver");
  lcd.setCursor(0, 1);
  lcd.print("Waiting for input...");
}

void loop() {
  char key = keypad.getKey();

  if (key) {
    int steps = 0;
    bool dir = true;
    int speedDelay = 1000;

    switch (key) {
      case '1': steps = 100; dir = true; break;
      case '2': steps = 200; dir = true; break;
      case '3': steps = 300; dir = true; break;
      case '4': steps = 400; dir = true; break;
      case '5': steps = 500; dir = true; break;
      case '6': steps = 600; dir = true; break;
      case '7': steps = 700; dir = true; break;
      case '8': steps = 800; dir = true; break;
      case '9': steps = 900; dir = true; break;
      case 'A': steps = 1000; dir = true; break;
      case '0': steps = 1600; dir = true; break;
      case '*': steps = 1600; dir = false; break;
      case 'B': steps = 100; dir = false; break;
      case 'C': steps = 500; dir = false; break;
      case 'D': steps = 1000; dir = false; break;
      case '#':
        lcd.setCursor(0, 1);
        lcd.print("Status: STOPPED     ");
        lcd.setCursor(0, 2);
        lcd.print("                    ");
        lcd.setCursor(0, 3);
        lcd.print("                    ");
        return;
      default: return;
    }

    digitalWrite(DIR, dir ? HIGH : LOW);

    lcd.setCursor(0, 1);
    lcd.print("Status: Rotating ");
    lcd.print(dir ? "CW " : "CCW");

    for (int i = 0; i < steps; i++) {
      digitalWrite(PUL, HIGH);
      delayMicroseconds(speedDelay);
      digitalWrite(PUL, LOW);
      delayMicroseconds(speedDelay);

      if (i % 10 == 0) {
        lcd.setCursor(0, 2);
        lcd.print(dir ? "CW  Steps: " : "CCW Steps: ");
        lcd.print(i);
        lcd.print("     ");

        lcd.setCursor(0, 3);
        lcd.print("Spin: ");
        lcd.print(spinner[spinIndex]);
        spinIndex = (spinIndex + 1) % 4;
      }
    }

    lcd.setCursor(0, 1);
    lcd.print("Status: Done        ");
  }
}
