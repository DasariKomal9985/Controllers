#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <MAX30105.h>
#include "heartRate.h"  // <-- for BPM
#include "spo2_algorithm.h" // <-- for SpO2

#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 2

MAX30105 sensor;

// Buffers for SpO2 & BPM algorithm
uint32_t irBuffer[100];
uint32_t redBuffer[100];
int32_t spo2; 
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

void setup() {
  Serial.begin(9600);
  Wire.begin(21, 22);  // SDA, SCL

  // ---- LoRa Setup ----
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa init failed!");
    while (1);
  }
  Serial.println("LoRa Transmitter Started");

  // ---- MAX30102 Setup ----
  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found!");
    while (1);
  }

  sensor.setup();
  sensor.setPulseAmplitudeRed(0x1F);
  sensor.setPulseAmplitudeIR(0x1F);

  Serial.println("MAX30102 Started");
}

void loop() {

  // ---- Step 1: Fill 100 samples ----
  for (int i = 0; i < 100; i++) {
    while (!sensor.available()) sensor.check();  // Wait for new data
    
    irBuffer[i] = sensor.getIR();
    redBuffer[i] = sensor.getRed();

    sensor.nextSample(); 
  }

  // ---- Step 2: Run Maxim SpO2 + BPM algorithm ----
  maxim_heart_rate_and_oxygen_saturation(
      irBuffer, 100,
      redBuffer,
      &spo2, &validSPO2,
      &heartRate, &validHeartRate
  );

  // ---- Step 3: Serial Output ----
  Serial.print("IR=");  Serial.print(irBuffer[50]);
  Serial.print(" RED="); Serial.print(redBuffer[50]);
  Serial.print(" BPM="); Serial.print(heartRate);
  Serial.print(" SpO2="); Serial.println(spo2);

  // ---- Step 4: LoRa Transmit ----
  LoRa.beginPacket();
  LoRa.print("IR:");
  LoRa.print(irBuffer[50]);
  LoRa.print(",RED:");
  LoRa.print(redBuffer[50]);
  LoRa.print(",BPM:");
  LoRa.print(heartRate);
  LoRa.print(",SPO2:");
  LoRa.print(spo2);
  LoRa.endPacket();

  delay(500);
}
