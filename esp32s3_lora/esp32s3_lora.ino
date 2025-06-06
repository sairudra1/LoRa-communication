#include <SPI.h>
#include <LoRa.h>

// Define LoRa module connections
#define SS      5   // Chip select (NSS)
#define RST     14  // Reset pin
#define DIO0    2   // DIO0 interrupt pin

int counter = 1;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Sender - ESP32-S3");

  // Start custom SPI with chosen pins
  SPI.begin(18, 19, 17, SS); // SCK, MISO, MOSI, SS
  LoRa.setSPI(SPI);

  // Set LoRa pins
  LoRa.setPins(SS, RST, DIO0);

  // Initialize LoRa with 433 MHz (Asia)
  while (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    delay(1000);
  }

  LoRa.setSyncWord(0xAB); // Match this with receiver

  Serial.println("LoRa Initialized Successfully");
}

void loop() {
  Serial.print("Sending packet: ");
  
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();
  counter++;

  delay(2500); // Wait 10 seconds
}
