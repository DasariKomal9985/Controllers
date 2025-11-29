#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

MAX30105 sensor;
hd44780_I2Cexp lcd;

// Buffer length required by the SparkFun SPO2 algorithm
#define SAMPLES 100

uint32_t irBuffer[SAMPLES];
uint32_t redBuffer[SAMPLES];

int32_t spo2;          // SPO2 result
int8_t  spo2Valid;     // SPO2 valid flag

int32_t heartRate;     // BPM result
int8_t  hrValid;       // BPM valid flag

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  lcd.begin(16, 2);
  lcd.print("Initializing...");

  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("Sensor not detected!");
    lcd.clear();
    lcd.print("Sensor ERROR!");
    while (1);
  }

  // Your exact configuration
  sensor.setup(0x7F, 4, 3, 100, 411, 16384);
  sensor.setPulseAmplitudeRed(0xFF);
  sensor.setPulseAmplitudeIR(0xFF);
  sensor.setPulseAmplitudeGreen(0);

  lcd.clear();
  lcd.print("Sensor OK");
  delay(500);

  lcd.clear();
  lcd.print("Reading...");
}

void loop() {

  // ---- RAW values (your original code) ----
  uint32_t ir = sensor.getIR();
  uint32_t red = sensor.getRed();

  Serial.print("IR=");
  Serial.print(ir);
  Serial.print("  RED=");
  Serial.println(red);
  //-------------------------------------------


  // ---- Fill SPO2 algorithm buffer ----
  for (int i = 0; i < SAMPLES; i++) {

    // ensure finger present
    while (sensor.getIR() < 50000) {
      delay(5);
    }

    irBuffer[i] = sensor.getIR();
    redBuffer[i] = sensor.getRed();

    delay(10);
  }


  // ---- Calculate BPM & SPO2 ----
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer,
    SAMPLES,
    redBuffer,
    &spo2,
    &spo2Valid,
    &heartRate,
    &hrValid
  );

  // ---- Serial output ----
  Serial.print("BPM = ");
  Serial.print(heartRate);
  Serial.print("  SpO2 = ");
  Serial.print(spo2);
  Serial.print("  HR_valid=");
  Serial.print(hrValid);
  Serial.print("  SPO2_valid=");
  Serial.println(spo2Valid);


  // ---- LCD output ----
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BPM:");
  lcd.print(heartRate);

  lcd.setCursor(0, 1);
  lcd.print("SpO2:");
  lcd.print(spo2);
  lcd.print("%");

  delay(300);
}
