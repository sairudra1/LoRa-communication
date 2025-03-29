/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Modified from the examples of the Arduino LoRa library
  More resources: https://RandomNerdTutorials.com/esp32-lora-rfm95-transceiver-arduino-ide/
*********/

#include <SPI.h>
#include <LoRa.h>

//define the pins used by the transceiver module
#define ss 10
#define rst 9
#define dio0 2

#define rx_sync_word 0xAB
#define tx_sync_word 0xFF
String LoRaData = "";
int counter = 0;

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);

  //replace the LoRa.begin(---E-) argument with your location's frequency
  //433E6 for Asia
  //868E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(433E6)) {
    Serial.println(".");
    delay(500);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  // LoRa.setSyncWord(0xFF);
  // Serial.println("LoRa Initializing OK!");
}

void loop() {

  LoRa.setSyncWord(rx_sync_word);
  //  Serial.println("LoRa Initializing OK!");

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      LoRaData = LoRa.readString();
      Serial.print(LoRaData);
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());

    LoRa.setSyncWord(tx_sync_word);
    Serial.println("LoRa Initializing OK!");

    Serial.print("Sending packet: ");
    Serial.println(counter);

    //Send LoRa packet to receiver
    LoRa.beginPacket();
    LoRa.print(LoRaData);
    //LoRa.print(counter);
    LoRa.endPacket();

    counter++;

    delay(50);
  }
  LoRaData = "";
  ///////////////////////////////////////////
  LoRa.setSyncWord(tx_sync_word);
  //  Serial.println("LoRa Initializing OK!");

  packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      LoRaData = LoRa.readString();
      Serial.print(LoRaData);
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());

    LoRa.setSyncWord(rx_sync_word);
    Serial.println("LoRa Initializing OK!");

    Serial.print("Sending packet: ");
    Serial.println(counter);

    //Send LoRa packet to receiver
    LoRa.beginPacket();
    LoRa.print(LoRaData);
    LoRa.endPacket();

    counter++;

    delay(50);
  }
}
