#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD setup for 20x4 (change address to 0x3F if needed)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Pins
#define SENSOR_PIN A3    // Analog input from sensor AOUT
#define LED_PIN 6        // Digital output to sensor ILED

// AQI thresholds
struct AQILevel {
  float minPM;
  float maxPM;
  int qualityNumber;     // 1–6 numeric level
  const char* evaluation;
} AQI_TABLE[] = {
  {0,   35,   1, "Excellent"},
  {35,  79,   2, "Average"},
  {80,  115,  3, "Light Pol"},
  {115, 150,  4, "Moderate Pol"},
  {150, 250,  5, "Heavy Pol"},
  {250, 500,  6, "Serious Pol"}
};

// Custom character for bar (full block)
byte barChar[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

// Previous values for change detection
float prevDustDensity = -1;
int prevQualityNumber = -1;
String prevEvaluation = "";
int prevBarLength = -1;

// Change tolerance in μg/m³
#define CHANGE_TOLERANCE 3.0

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  // Create custom bar character
  lcd.createChar(0, barChar);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Static title (doesn't change)
  lcd.setCursor(0, 0);
  lcd.print("   Dust Sensor   ");
}

void loop() {
  // Turn on the sensor LED
  digitalWrite(LED_PIN, HIGH);
  delayMicroseconds(280);

  int rawValue = analogRead(SENSOR_PIN);
  digitalWrite(LED_PIN, LOW);

  // Convert ADC value to voltage (5V, 10-bit ADC)
  float voltage = (rawValue / 1023.0) * 5.0;

  // μg/m³ calculation: 0.5V = 100 μg/m³ → 1V = 200 μg/m³
  float dustDensity = (voltage / 0.5) * 100.0;

  // Clamp values
  if (dustDensity < 0) dustDensity = 0;
  if (dustDensity > 500) dustDensity = 500;

  // Determine AQI
  int qualityNumber = 0;
  const char* evaluation = "";
  for (int i = 0; i < 6; i++) {
    if (dustDensity >= AQI_TABLE[i].minPM && dustDensity <= AQI_TABLE[i].maxPM) {
      qualityNumber = AQI_TABLE[i].qualityNumber;
      evaluation = AQI_TABLE[i].evaluation;
      break;
    }
  }

  // Serial output
  Serial.print("PM2.5: ");
  Serial.print(dustDensity);
  Serial.print(" ug/m3 | AQI: ");
  Serial.print(qualityNumber);
  Serial.print(" | ");
  Serial.println(evaluation);

  // Update PM2.5 value only if change exceeds tolerance
  if (fabs(dustDensity - prevDustDensity) >= CHANGE_TOLERANCE || prevDustDensity < 0) {
    lcd.setCursor(0, 1);
    lcd.print("PM2.5:           "); // clear old text
    lcd.setCursor(7, 1);
    lcd.print(dustDensity, 1);
    lcd.print(" ug/m3");
    prevDustDensity = dustDensity;
  }

  // Update AQI only if it changes
  if (qualityNumber != prevQualityNumber || prevEvaluation != String(evaluation)) {
    lcd.setCursor(0, 2);
    lcd.print("AQI:             "); // clear old text
    lcd.setCursor(5, 2);
    lcd.print(qualityNumber);
    lcd.print(" ");
    lcd.print(evaluation);
    prevQualityNumber = qualityNumber;
    prevEvaluation = evaluation;
  }

  // Update bar graph only if AQI level changes
  int barLength = map(qualityNumber, 1, 6, 3, 20); // min 3 blocks
  if (barLength != prevBarLength) {
    lcd.setCursor(0, 3);
    for (int i = 0; i < barLength; i++) {
      lcd.write(byte(0)); // full block char
    }
    for (int i = barLength; i < 20; i++) {
      lcd.print(" "); // empty space
    }
    prevBarLength = barLength;
  }

  delay(500);
}
