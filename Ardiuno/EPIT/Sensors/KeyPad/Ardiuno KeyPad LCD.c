#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define PCF_ADDR 0x20

LiquidCrystal_I2C lcd(0x27, 20, 4);

char keys[4][4] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

String typedText = "";
String resultText = "";
unsigned long starPressedTime = 0;
bool starBeingHeld = false;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();

  showIntroAnimation();
  showCalculatorInstructions();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Interfacing KeyPad");
  lcd.setCursor(0, 1);
  lcd.print("A=+ B=- C=* D=/");

  updateDisplay();
}

void loop() {
  char key = scanKeypad();

  if (key != 0) {
    Serial.print("Key Pressed: ");
    Serial.println(key);

    if (key == '*') {
      if (!starBeingHeld) {
        starPressedTime = millis();
        starBeingHeld = true;
      }
    } else {
      if (starBeingHeld) {
        unsigned long pressDuration = millis() - starPressedTime;
        if (pressDuration < 1000) {
          if (typedText.length() > 0) {
            typedText.remove(typedText.length() - 1);
          }
        } else {
          typedText = "";
          resultText = "";
        }
        starBeingHeld = false;
        updateDisplay();
      }

      if (key == '#') {
        resultText = evaluateExpression(typedText);
      } else {
        if (isOperator(key)) {
          if (typedText.length() == 0 || isOperator(typedText.charAt(typedText.length() - 1))) {
          } else {
            typedText += key;
          }
        } else {
          typedText += key;
        }
      }

      updateDisplay();
      delay(200);
    }
  }

  if (starBeingHeld && millis() - starPressedTime >= 1000) {
    typedText = "";
    resultText = "";
    starBeingHeld = false;
    updateDisplay();
  }
}

bool isOperator(char c) {
  return (c == 'A' || c == 'B' || c == 'C' || c == 'D');
}

void updateDisplay() {
  lcd.setCursor(0, 2);
  lcd.print("                    ");

  String displayText = typedText;
  displayText.replace("A", "+");
  displayText.replace("B", "-");
  displayText.replace("C", "x");
  displayText.replace("D", "/");

  if (displayText.length() > 20) {
    lcd.setCursor(0, 2);
    lcd.print(displayText.substring(displayText.length() - 20));
  } else {
    lcd.setCursor(0, 2);
    lcd.print(displayText);
  }

  lcd.setCursor(0, 3);
  lcd.print("                    ");
  if (resultText.length() > 0) {
    lcd.setCursor(0, 3);
    lcd.print("= ");
    lcd.print(resultText);
  }
}

String evaluateExpression(String input) {
  if (input.length() < 3) return "Err";
  if (isOperator(input.charAt(input.length() - 1))) return "Err";

  char op = 0;
  String currentNumber = "";
  int result = 0;
  bool firstNumber = true;
  bool divByZero = false;

  for (int i = 0; i < input.length(); i++) {
    char c = input[i];
    if (isDigit(c)) {
      currentNumber += c;
    } else if (isOperator(c)) {
      if (currentNumber == "") return "Err";
      int num = currentNumber.toInt();
      if (firstNumber) {
        result = num;
        firstNumber = false;
      } else {
        if (op == 'D' && num == 0) {
          divByZero = true;
          break;
        }
        result = applyOperation(result, num, op);
      }
      op = c;
      currentNumber = "";
    }
  }

  if (!divByZero) {
    if (currentNumber != "") {
      int num = currentNumber.toInt();
      if (op == 'D' && num == 0) {
        divByZero = true;
      } else {
        result = applyOperation(result, num, op);
      }
    } else {
      return "Err";
    }
  }

  if (divByZero) {
    return "undefined";
  }

  return String(result);
}

int applyOperation(int a, int b, char op) {
  switch (op) {
    case 'A': return a + b;
    case 'B': return a - b;
    case 'C': return a * b;
    case 'D': return a / b;
    default: return b;
  }
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
        return keys[row][col];
      }
    }
  }
  return 0;
}

void writePCF(byte val) {
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(val);
  Wire.endTransmission();
}

byte readPCF() {
  Wire.requestFrom(PCF_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}

void showIntroAnimation() {
  lcd.clear();
  const char* line1 = "Interface with";
  const char* line2 = "Keypad";
  const char* line3 = "Lets do calculator";

  lcd.setCursor(0, 0);
  for (int i = 0; line1[i] != '\0'; i++) {
    lcd.print(line1[i]);
    delay(80);
  }

  lcd.setCursor(0, 1);
  for (int i = 0; line2[i] != '\0'; i++) {
    lcd.print(line2[i]);
    delay(80);
  }

  lcd.setCursor(0, 2);
  for (int i = 0; line3[i] != '\0'; i++) {
    lcd.print(line3[i]);
    delay(80);
  }

  delay(2000);
  lcd.clear();
}

void showCalculatorInstructions() {
  const char* lines[] = {
    "A:Add  B:Sub",
    "C:Mul  D:Div",
    "#=Enter *=Clear",
    "*=Long press: Clear All"
  };

  for (int i = 0; i < 4; i++) {
    lcd.setCursor(0, i);
    for (int j = 0; lines[i][j] != '\0'; j++) {
      lcd.print(lines[i][j]);
      delay(60);
    }
  }

  delay(2500);
  lcd.clear();
}