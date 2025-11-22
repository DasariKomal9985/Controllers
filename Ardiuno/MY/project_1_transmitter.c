#include <Wire.h>
#include <MAX30105.h>
#include "heartRate.h"
#include "spo2_algorithm.h"

#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <Adafruit_MLX90614.h>

#include <SPI.h>
#include <LoRa.h>

// -------- AD8232 ECG PIN --------
#define ECG_PIN 34

// -------- LoRa Pins -------- 
#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 2

// -------- GSM (SIM900A) PINS --------
#define GSM_RX 16   // ESP32 RX2
#define GSM_TX 17   // ESP32 TX2
HardwareSerial GSM(2);  // UART2

// Objects
MAX30105 sensor;
hd44780_I2Cexp lcd;
Adafruit_MLX90614 mlx;

// Buffers
uint32_t irBuffer[100];
uint32_t redBuffer[100];
int sampleIndex = 0;

int32_t spo2, heartRate;
int8_t validSPO2, validHeartRate;

unsigned long lastTempRead = 0;
unsigned long lastLCD = 0;
unsigned long lastSample = 0;
unsigned long lastLoRa = 0;
unsigned long lastSMS = 0;

float objectT = 0;
int ecgValue = 0;

// -------- PHONE NUMBERS --------
String phone1 = "9985798499";
String phone2 = "9876543210";
String phone3 = "9123456789";

void sendSMS(String number, String msg) {
  GSM.println("AT+CMGF=1");
  delay(500);
  GSM.print("AT+CMGS=\"");
  GSM.print(number);
  GSM.println("\"");
  delay(500);
  GSM.print(msg);
  delay(500);
  GSM.write(26);  // CTRL+Z
  delay(2000);
}

void setup() {

  Serial.begin(9600);
  Wire.begin(21, 22);
  delay(500);

  // ECG
  pinMode(ECG_PIN, INPUT);

  // MLX90614
  mlx.begin();

  // LCD
  lcd.begin(20, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();

  // MAX30102
  if (!sensor.begin(Wire)) {
    Serial.println("MAX30102 fail!");
  }

  sensor.setup();
  sensor.setPulseAmplitudeRed(0x1F);
  sensor.setPulseAmplitudeIR(0x1F);

  // LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  LoRa.begin(915E6);
  Serial.println("LoRa Ready");

  // -------- GSM (SIM900A) --------
  GSM.begin(9600, SERIAL_8N1, GSM_RX, GSM_TX);
  delay(1000);
  GSM.println("AT");
  delay(500);
  GSM.println("AT+CMGF=1");  // Text mode
  delay(500);

  Serial.println("GSM Ready");
}

void loop() {

  unsigned long now = millis();

  // ---------------- ECG ----------------
  int rawECG = analogRead(ECG_PIN);
  ecgValue = map(rawECG, 0, 4095, 0, 100);

  // ---------------- TEMP ----------------
  if (now - lastTempRead >= 300) {
    lastTempRead = now;
    objectT = mlx.readObjectTempC();
  }

  // ---------------- MAX30102 ----------------
  if (now - lastSample >= 5) {
    lastSample = now;

    if (sensor.available()) {
      irBuffer[sampleIndex] = sensor.getIR();
      redBuffer[sampleIndex] = sensor.getRed();
      sensor.nextSample();
      sampleIndex++;

      if (sampleIndex >= 100) {
        sampleIndex = 0;

        maxim_heart_rate_and_oxygen_saturation(
          irBuffer, 100,
          redBuffer,
          &spo2, &validSPO2,
          &heartRate, &validHeartRate);
      }
    }
    sensor.check();
  }

  // ---------------- LCD ----------------
  if (now - lastLCD >= 300) {
    lastLCD = now;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(objectT, 0);
    lcd.print(" BPM:");
    lcd.print(validHeartRate ? heartRate : -1);

    lcd.setCursor(0, 1);
    lcd.print("ECG:");
    lcd.print(ecgValue);
    lcd.print(" SpO2:");
    lcd.print(validSPO2 ? spo2 : -1);
  }

  // ---------------- LoRa ----------------
  if (now - lastLoRa >= 500) {
    lastLoRa = now;

    LoRa.beginPacket();
    LoRa.print("T:");
    LoRa.print(objectT);
    LoRa.print(",BPM:");
    LoRa.print(validHeartRate ? heartRate : -1);
    LoRa.print(",SPO2:");
    LoRa.print(validSPO2 ? spo2 : -1);
    LoRa.print(",ECG:");
    LoRa.print(ecgValue);
    LoRa.endPacket();
  }

  // ---------------- GSM SMS Every 5 Seconds ----------------
  if (now - lastSMS >= 5000) {
    lastSMS = now;

    String message = 
      "T:" + String(objectT) +
      " BPM:" + String(validHeartRate ? heartRate : -1) +
      " SpO2:" + String(validSPO2 ? spo2 : -1) +
      " ECG:" + String(ecgValue);

    sendSMS(phone1, message);
    sendSMS(phone2, message);
    sendSMS(phone3, message);

    Serial.println("SMS sent to all numbers");
  }
}
