#include <SPI.h>
#include <LoRa.h>

#define ss 5
#define rst 14
#define dio0 2

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 2000; // 2 seconds
int counter = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender & Receiver");

  LoRa.setPins(ss, rst, dio0);
  while (!LoRa.begin(433E6)) {
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0xFF);
  Serial.println("LoRa Initialized Successfully!");
}

void loop() {
  // Send data every sendInterval milliseconds
  if (millis() - lastSendTime > sendInterval) {
    sendMessage();
    lastSendTime = millis();
  }
  
  receiveMessage();
}

void sendMessage() {
  Serial.print("Sending packet: ");
  Serial.println(counter);
  
  LoRa.beginPacket();
  LoRa.print("");
  LoRa.print(counter);
  LoRa.endPacket();
  
  counter++;
}

void receiveMessage() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Received packet: ");
    String LoRaData = "";
    while (LoRa.available()) {
      LoRaData += (char)LoRa.read();
    }
    Serial.println(LoRaData);
    Serial.print("RSSI: ");
    Serial.println(LoRa.packetRssi());
  }
}
