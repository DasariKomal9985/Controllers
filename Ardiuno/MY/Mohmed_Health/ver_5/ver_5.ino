#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MLX90614.h>
#include <RTClib.h>
#include <HardwareSerial.h>

#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

// ------------------ LCD 20x2 ------------------
LiquidCrystal_I2C lcd(0x27, 20, 2);

// ------------------ MLX90614 ------------------
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// ------------------ ECG ------------------
#define ECG_PIN 34
int ecgValue = 0;

// ------------------ RTC ------------------
RTC_DS3231 rtc;

// ------------------ SIM900A ------------------
HardwareSerial sim900(1);
String phoneNumbers[] = { "9984798499", "9985798499", "9986798499" };

// ------------------ MAX30102 ------------------
MAX30105 sensor;
#define SAMPLES 100
uint32_t irBuffer[SAMPLES];
uint32_t redBuffer[SAMPLES];
int32_t spo2;
int8_t spo2Valid;
int32_t heartRate;
int8_t hrValid;

// ------------------ SMS function ------------------
void sendSMS(String number, String message) {
  sim900.print("AT+CMGF=1\r");  // Text mode
  delay(100);
  sim900.print("AT+CMGS=\"" + number + "\"\r");
  delay(100);
  sim900.print(message);
  delay(100);
  sim900.write(26);  // Ctrl+Z
  delay(2000);
}

void setup() {
  Serial.begin(115200);

  // Explicit I2C for ESP32
  Wire.begin(21, 22);
  delay(50);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Initializing...");

  // MLX90614
  int retries = 0;
  while (!mlx.begin()) {
    lcd.clear();
    lcd.print("Temp Sensor Err");
    Serial.println("MLX90614 not detected! Retrying...");
    retries++;
    if (retries > 5) {
      lcd.print("Check wiring");
      while (1)
        ;
    }
    delay(500);
  }
  delay(100);

  // ECG
  pinMode(ECG_PIN, INPUT);

  // RTC
  // RTC
  if (!rtc.begin()) {
    lcd.clear();
    lcd.print("RTC Error!");
    Serial.println("RTC not detected!");
    while (1)
      ;
  }

  // Set RTC time (only on first upload / or you can comment out after first run)
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  Serial.println("RTC time set automatically");


  // SIM900
  sim900.begin(9600, SERIAL_8N1, 16, 17);

  // MAX30102
  if (!sensor.begin(Wire, I2C_SPEED_STANDARD)) {
    lcd.clear();
    lcd.print("MAX30102 ERROR");
    Serial.println("MAX30102 not detected!");
    while (1)
      ;
  }
  sensor.setup(0x7F, 4, 3, 100, 411, 16384);
  sensor.setPulseAmplitudeRed(0xFF);
  sensor.setPulseAmplitudeIR(0xFF);
  sensor.setPulseAmplitudeGreen(0);

  // LoRa screen placeholder
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LoRa Transmitter");
  lcd.setCursor(0, 1);
  lcd.print("will be heading");
  delay(5000);
  lcd.clear();
}

void loop() {
  // ----------- 1. Temperature Screen ----------
  double objectTemp = mlx.readObjectTempC();
  double ambientTemp = mlx.readAmbientTempC();

  // Retry if NaN
  int tries = 0;
  while (isnan(objectTemp) || isnan(ambientTemp)) {
    objectTemp = mlx.readObjectTempC();
    ambientTemp = mlx.readAmbientTempC();
    tries++;
    if (tries > 5) break;
    delay(200);
  }

  lcd.setCursor(0, 0);
  lcd.print("Ambient:");
  lcd.print(ambientTemp, 2);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Object: ");
  lcd.print(objectTemp, 2);
  lcd.print((char)223);
  lcd.print("C");
  delay(3000);
  lcd.clear();

  // ----------- 2. ECG Screen ----------
  int rawECG = analogRead(ECG_PIN);
  ecgValue = map(rawECG, 0, 4095, 0, 100);

  lcd.setCursor(0, 0);
  lcd.print("ECG Raw: ");
  lcd.print(rawECG);
  lcd.setCursor(0, 1);
  lcd.print("Mapped: ");
  lcd.print(ecgValue);
  delay(3000);
  lcd.clear();

  // ----------- 3. MAX30102 Heart + SpO2 ----------
  lcd.setCursor(0, 0);
  lcd.print("Reading...");
  for (int i = 0; i < SAMPLES; i++) {
    while (sensor.getIR() < 50000) { delay(5); }  // ensure finger
    irBuffer[i] = sensor.getIR();
    redBuffer[i] = sensor.getRed();
    delay(10);
  }
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, SAMPLES,
    redBuffer,
    &spo2, &spo2Valid,
    &heartRate, &hrValid);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BPM:");
  lcd.print(heartRate);
  lcd.setCursor(0, 1);
  lcd.print("SpO2:");
  lcd.print(spo2);
  lcd.print("%");
  delay(3000);
  lcd.clear();

  // ----------- 4. RTC Screen ----------
  DateTime now = rtc.now();
  lcd.setCursor(0, 0);
  lcd.print("Date: ");
  lcd.print(now.day() < 10 ? "0" : "");
  lcd.print(now.day());
  lcd.print("/");
  lcd.print(now.month() < 10 ? "0" : "");
  lcd.print(now.month());
  lcd.print("/");
  lcd.print(now.year());

  lcd.setCursor(0, 1);
  lcd.print("Time: ");
  lcd.print(now.hour() < 10 ? "0" : "");
  lcd.print(now.hour());
  lcd.print(":");
  lcd.print(now.minute() < 10 ? "0" : "");
  lcd.print(now.minute());
  lcd.print(":");
  lcd.print(now.second() < 10 ? "0" : "");
  lcd.print(now.second());
  delay(3000);
  lcd.clear();

  // ----------- 5. SMS Screen (send all sensor data) ----------
  String msg = "Ambient:" + String(ambientTemp, 2) + "C, Object:" + String(objectTemp, 2) + "C";
  msg += ", ECG:" + String(ecgValue);
  msg += ", Date:" + String(now.day()) + "/" + String(now.month()) + "/" + String(now.year());
  msg += ", Time:" + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
  msg += ", BPM:" + String(heartRate) + ", SpO2:" + String(spo2) + "%";

  lcd.setCursor(0, 0);
  lcd.print("Sending Data...");
  for (int i = 0; i < 3; i++) {
    sendSMS(phoneNumbers[i], msg);
  }
  lcd.setCursor(0, 1);
  lcd.print("SMS Sent!");
  delay(3000);
  lcd.clear();
}
