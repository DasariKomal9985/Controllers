#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Wire.h>
#include <SPI.h>

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

#define PCF_ADDR 0x20

char keys[4][4] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

String typedText = "";
String operand1 = "", operand2 = "";
char operation = 0;
bool secondOperand = false;
unsigned long starPressedTime = 0;
bool starBeingHeld = false;


int currentResult = 0;
bool hasResult = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  introAnimation();
  showCalculatorHelp();

  drawHeading();
  drawKeypad();
  drawTypedBox();
}

void loop() {
  char key = scanKeypad();


  if (starBeingHeld && millis() - starPressedTime >= 1000) {

    operand1 = operand2 = "";
    operation = 0;
    typedText = "";
    secondOperand = false;
    hasResult = false;
    currentResult = 0;
    updateTypedBox();
    starBeingHeld = false;
    delay(300);
    drawKeypad();
    return;
  }

  if (key != 0) {
    highlightKey(key);
    handleKey(key);
    updateTypedBox();
    delay(150);
    drawKeypad();
  }
}


void animateText(String text, int x, int y, int size, uint16_t color) {
  tft.setTextSize(size);
  tft.setTextColor(color);
  tft.setCursor(x, y);
  for (int i = 0; i < text.length(); i++) {
    tft.print(text[i]);
    delay(80);
  }
}

void introAnimation() {
  tft.fillScreen(ST77XX_BLACK);
  animateText("Interface with Keypad", 10, 80, 2, ST77XX_YELLOW);
  delay(500);
  animateText("Let's do Calculator", 20, 120, 2, ST77XX_CYAN);
  delay(2000);
  tft.fillScreen(ST77XX_BLACK);
}

void showCalculatorHelp() {

  tft.setCursor(20, 40);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.println("A - Addition");
  delay(300);
  tft.setCursor(20, 65);
  tft.println("B - Subtraction");
  delay(300);
  tft.setCursor(20, 90);
  tft.println("C - Multiply");
  delay(300);
  tft.setCursor(20, 120);
  tft.println("D - Division");
  delay(300);
  tft.setCursor(20, 150);
  tft.println("# - Enter");
  delay(300);
  tft.setCursor(20, 175);
  tft.println("* - Clear");
  delay(300);
  tft.setCursor(20, 200);
  tft.println("* - Longpress - Clear All");
  delay(2000);
  tft.fillScreen(ST77XX_BLACK);
}

void drawHeading() {
  tft.fillRect(0, 10, 320, 40, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(3);
  tft.setCursor(80, 20);
  tft.print("KeyBoard");
}

void drawKeypad() {
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      drawKeyBox(row, col, false);
    }
  }
}

void highlightKey(char key) {
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      drawKeyBox(row, col, keys[row][col] == key);
    }
  }
}

void drawKeyBox(int row, int col, bool active) {
  int x = 5 + col * 75;
  int y = 60 + row * 35;
  int w = 70;
  int h = 30;

  uint16_t fillColor = ST77XX_BLACK;
  uint16_t textColor = ST77XX_WHITE;

  if (active) {
    switch (keys[row][col]) {
      case '1' ... '3': fillColor = ST77XX_RED; break;
      case '4' ... '6': fillColor = ST77XX_ORANGE; break;
      case '7' ... '9': fillColor = ST77XX_YELLOW; break;
      case 'A' ... 'D': fillColor = ST77XX_BLUE; break;
      case '*': fillColor = ST77XX_GREEN; break;
      case '0': fillColor = ST77XX_MAGENTA; break;
      case '#': fillColor = ST77XX_CYAN; break;
    }
  }

  tft.fillRect(x, y, w, h, fillColor);
  tft.drawRect(x, y, w, h, ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setTextColor(textColor);
  tft.setCursor(x + 25, y + 7);
  char label = keys[row][col];
  if (label == 'A') label = '+';
  else if (label == 'B') label = '-';
  else if (label == 'C') label = 'X';
  else if (label == 'D') label = '/';
  tft.print(label);
}

void drawTypedBox() {
  tft.drawRect(0, 200, 320, 35, ST77XX_WHITE);
  updateTypedBox();
}

void updateTypedBox() {
  tft.fillRect(10, 201, 298, 33, ST77XX_BLACK);
  tft.setCursor(15, 210);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  int len = typedText.length();
  if (len > 20) {
    tft.print(typedText.substring(len - 20));
  } else {
    tft.print(typedText);
  }
}



void handleKey(char key) {
  if (key >= '0' && key <= '9') {
    if (!secondOperand) {
      operand1 += key;
    } else {
      operand2 += key;
    }
    typedText += key;
  } else if (key == 'A' || key == 'B' || key == 'C' || key == 'D') {
    if (!secondOperand && operand1 != "") {
      if (hasResult) {
        operand1 = String(currentResult);
        operand2 = "";
      }
      operation = key;
      secondOperand = true;
      typedText += operatorSymbol(key);
    } else if (secondOperand && operand2 != "") {
      currentResult = calculateResult();
      operand1 = String(currentResult);
      operand2 = "";
      operation = key;
      typedText += operatorSymbol(key);
      hasResult = true;
    }
  } else if (key == '#') {
    if (operand1 != "" && operand2 != "" && operation != 0) {
      currentResult = calculateResult();
      typedText += "=" + String(currentResult);
      Serial.print("Result: ");
      Serial.println(currentResult);
      operand1 = String(currentResult);
      operand2 = "";
      operation = 0;
      secondOperand = false;
      hasResult = true;
    }
  } else if (key == '*') {
    if (starBeingHeld && millis() - starPressedTime >= 1000) {

      operand1 = operand2 = "";
      operation = 0;
      typedText = "";
      secondOperand = false;
      hasResult = false;
      currentResult = 0;
    } else {

      if (typedText.length() > 0) {
        char lastChar = typedText.charAt(typedText.length() - 1);
        typedText.remove(typedText.length() - 1);
        if (isDigit(lastChar)) {
          if (secondOperand && operand2.length() > 0) {
            operand2.remove(operand2.length() - 1);
          } else if (!secondOperand && operand1.length() > 0) {
            operand1.remove(operand1.length() - 1);
          }
        } else if (lastChar == '+' || lastChar == '-' || lastChar == '*' || lastChar == '/') {
          operation = 0;
          secondOperand = false;
        }
      }
    }
  }
}




String operatorSymbol(char op) {
  switch (op) {
    case 'A': return "+";
    case 'B': return "-";
    case 'C': return "*";
    case 'D': return "/";
    default: return "";
  }
}

int calculateResult() {
  int num1 = operand1.toInt();
  int num2 = operand2.toInt();
  switch (operation) {
    case 'A': return num1 + num2;
    case 'B': return num1 - num2;
    case 'C': return num1 * num2;
    case 'D': return num2 != 0 ? num1 / num2 : 0;
  }
  return 0;
}

char scanKeypad() {
  static char lastKey = 0;
  static unsigned long lastDebounceTime = 0;

  char keyPressed = 0;

  for (int row = 0; row < 4; row++) {
    byte out = 0xFF;
    out &= ~(1 << (row + 4));
    writePCF(out);
    delay(2);

    byte input = readPCF();
    for (int col = 0; col < 4; col++) {
      if ((input & (1 << col)) == 0) {
        keyPressed = keys[row][col];
        break;
      }
    }

    if (keyPressed) break;
  }


  if (keyPressed == '*') {
    if (!starBeingHeld) {
      starPressedTime = millis();
      starBeingHeld = true;
    }
  } else {
    if (starBeingHeld) {
      starBeingHeld = false;
    }
  }


  if (keyPressed != 0 && keyPressed != lastKey) {
    lastKey = keyPressed;
    lastDebounceTime = millis();
    return keyPressed;
  }

  if (keyPressed == 0 && lastKey != 0) {
    lastKey = 0;
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