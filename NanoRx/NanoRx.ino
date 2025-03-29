/*********
  Modified for Arduino Nano with LoRa RFM95
*********/

#include <SPI.h>
#include <LoRa.h>

// Define the pins used by the LoRa transceiver module
#define ss 10      // NSS (Chip Select)
#define rst 9      // Reset
#define dio0 2     // IRQ (Interrupt)

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 2000; // 2 seconds
int counter = 0;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender & Receiver - Arduino Nano");

  // Setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);

  // LoRa Frequency (433E6 for Asia, 868E6 for Europe, 915E6 for North America)
  while (!LoRa.begin(433E6)) {
    Serial.println("LoRa Initialization Failed!");
    delay(500);
  }

  // Set sync word to ensure communication with receiver
  LoRa.setSyncWord(0xAB);
  Serial.println("LoRa Initialization Successful!");
}

void loop() {
  // Send data every sendInterval milliseconds
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Received packet:   ");
    String LoRaData = "";
    while (LoRa.available()) {
      LoRaData += (char)LoRa.read();
    }
    Serial.println(LoRaData);
    
    if (LoRaData == "drop") {
      Serial.println("//////////////////////////////////////////////drop data packet received////////////////////////////////////////");
      delay(1500);
    }
    
    Serial.print("RSSI: ");
    Serial.println(LoRa.packetRssi());
  }
}
