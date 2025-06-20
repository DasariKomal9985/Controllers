#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Keypad.h>

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

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

char lastKey = NO_KEY;
String typedText = "";

void setup() {
  Serial.begin(9600);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  drawHeading();
  drawKeypad();
  drawTypedBox();
}

void loop() {
  char key = keypad.getKey();
  if (key != NO_KEY) {
    highlightKey(key);
    typedText += key;
    updateTypedBox();
    Serial.print("Key Pressed: ");
    Serial.println(key);
    delay(150);
    drawKeypad();
  }
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
  tft.print(keys[row][col]);
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
