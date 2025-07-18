#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <DHT.h>

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 7
#define TFT_BL 9

#define DHTPIN 35
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define GRAY 0x8410

float lastTemperature = -999;
float lastHumidity = -999;
bool firstDraw = true;

float dropletWave = 0;

void setup() {
  Serial.begin(9600);
  dht.begin();
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(BLACK);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
}

void loop() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Sensor read failed!");
    tft.fillScreen(BLACK);
    tft.setCursor(20, 120);
    tft.setTextColor(RED);
    tft.setTextSize(2);
    tft.print("Sensor Read Error!");
    delay(2000);
    return;
  }

  if (firstDraw) {
    tft.fillScreen(BLACK);
    tft.fillRect(10, 5, 310, 40, WHITE);
    tft.setCursor(60, 15);
    tft.setTextColor(BLACK);
    tft.setTextSize(3);
    tft.print("AM2305 Sensor");
    tft.drawRect(100, 75, 180, 40, YELLOW);
    tft.drawRect(100, 175, 180, 40, GREEN);
    drawAnimatedThermometer(40, 135, 70, temperature);
    drawAnimatedHumidityDrop(40, 210, 40, humidity);
    drawTemperatureText(temperature);
    drawHumidityText(humidity);
    lastTemperature = temperature;
    lastHumidity = humidity;
    firstDraw = false;
  }

  if (abs(temperature - lastTemperature) >= 0.1) {
    drawAnimatedThermometer(40, 135, 70, temperature);
    drawTemperatureText(temperature);
    lastTemperature = temperature;
  }

  if (abs(humidity - lastHumidity) >= 1.0) {
    drawAnimatedHumidityDrop(40, 210, 40, humidity);
    drawHumidityText(humidity);
    lastHumidity = humidity;
  }

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  delay(300);
}

void drawTemperatureText(float temperature) {
  static float lastDrawnTemp = -999;
  if (abs(temperature - lastDrawnTemp) < 0.1) return;
  tft.fillRect(120, 80, 100, 20, BLACK);
  tft.setCursor(120, 85);
  tft.setTextColor(YELLOW);
  tft.setTextSize(2);
  tft.print("Temp: ");
  tft.setTextColor(WHITE);
  tft.print(temperature, 1);
  tft.write(247);
  tft.print("C");
  lastDrawnTemp = temperature;
}

void drawHumidityText(float humidity) {
  static float lastDrawnHum = -999;
  if (abs(humidity - lastDrawnHum) < 1.0) return;
  tft.fillRect(120, 180, 100, 20, BLACK);
  tft.setCursor(120, 185);
  tft.setTextColor(GREEN);
  tft.setTextSize(2);
  tft.print("Humi: ");
  tft.setTextColor(WHITE);
  tft.print(humidity, 1);
  tft.print(" %");
  lastDrawnHum = humidity;
}

void drawAnimatedThermometer(int x, int y, int height, float temp) {
  int level = map((int)temp, 0, 50, 0, height - 20);
  tft.fillRect(x - 20, y - height - 20, 50, height + 40, BLACK);
  int tubeWidth = 14;
  int tubeHeight = height - 15;
  tft.fillRoundRect(x - tubeWidth / 2, y - tubeHeight, tubeWidth, tubeHeight, 6, 0xE71C);
  tft.drawRoundRect(x - tubeWidth / 2, y - tubeHeight, tubeWidth, tubeHeight, 6, GRAY);
  if (level > 0) {
    tft.fillRoundRect(x - 5, y - level, 10, level, 3, RED);
  }
  tft.fillCircle(x, y + 8, 12, RED);
  tft.drawCircle(x, y + 8, 12, 0x8000);
}

void drawAnimatedHumidityDrop(int x, int y, int height, float hum) {
  int level = map((int)hum, 0, 100, 0, height - 10);
  tft.fillRect(x - 30, y - height - 10, 60, height + 25, BLACK);
  drawTeardropOutline(x, y, 22, BLUE);
  tft.fillCircle(x, y - 5, 18, 0x5E7F);
  if (level > 0) {
    dropletWave += 3;
    if (dropletWave >= 360) dropletWave = 0;
    int waveOffset = sin(dropletWave * PI / 180.0) * 2;
    for (int i = 0; i < level; i++) {
      int waterY = y - i;
      int radius = sqrt(324 - (waterY - y) * (waterY - y));
      if (radius > 0 && waterY <= y) {
        uint16_t waterColor = 0x001F + (i * 8);
        if (waterColor > 0x5E7F) waterColor = 0x5E7F;
        tft.drawLine(x - radius, waterY + waveOffset, x + radius, waterY + waveOffset, waterColor);
      }
    }
  }
  drawPercentageSymbol(x, y - 3);
}

void drawTeardropOutline(int x, int y, int radius, uint16_t color) {
  tft.drawCircle(x, y, radius, color);
  tft.drawCircle(x, y, radius + 1, color);
  int topHeight = 15;
  int topWidth = 10;
  for (int i = 0; i <= topWidth; i++) {
    int h = topHeight * sqrt(1 - (i * i) / float(topWidth * topWidth));
    tft.drawPixel(x + i, y - radius - h, color);
    tft.drawPixel(x - i, y - radius - h, color);
  }
  for (int i = 0; i < topHeight; i++) {
    int w = topWidth * sqrt(1 - (i * i) / float(topHeight * topHeight));
    tft.drawLine(x - w, y - radius - i, x + w, y - radius - i, 0x5E7F);
  }
}

void drawPercentageSymbol(int x, int y) {
  uint16_t symbolColor = BLUE;
  tft.fillCircle(x - 5, y - 5, 3, symbolColor);
  tft.fillCircle(x + 5, y + 5, 3, symbolColor);
  for (int i = -1; i <= 1; i++) {
    tft.drawLine(x - 8 + i, y + 8, x + 8 + i, y - 8, symbolColor);
  }
}
