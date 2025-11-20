#include <SPI.h>
#include <LoRa.h>

#define LORA_SS 15      // D8
#define LORA_RST 16     // D0
#define LORA_DIO0 5     // D1

void setup() {
  Serial.begin(9600);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa failed!");
    while (1);
  }

  Serial.println("LoRa Receiver Started");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Received: ");

    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    Serial.print(" | RSSI: ");
    Serial.println(LoRa.packetRssi());
  }
}
