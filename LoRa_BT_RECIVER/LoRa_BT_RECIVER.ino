#include <SPI.h>
#include <LoRa.h>
#include "BluetoothSerial.h"

// Create a Bluetooth Serial object
BluetoothSerial BTSerial;

// Define LoRa pins
#define ss 5
#define rst 14
#define dio0 2

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa + Bluetooth Receiver");

  // Start Bluetooth Serial with a name
  BTSerial.begin("ESP32_LoRa_BT");  // You can change the name shown on Bluetooth
  Serial.println("Bluetooth started. Pair and connect.");

  // Setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);

  while (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    delay(500);
  }

  LoRa.setSyncWord(0xAB); // Must match the sender
  Serial.println("LoRa Initializing OK!");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Received packet: ");
    BTSerial.print("Received packet: ");

    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      Serial.print(LoRaData);
      BTSerial.print(LoRaData);
    }

    Serial.print(" with RSSI ");
    Serial.println(LoRa.packetRssi());
    BTSerial.print(" with RSSI ");
    BTSerial.println(LoRa.packetRssi());
  }
}
