#include <SPI.h>
#include <LoRa.h>
#include <ESP32Servo.h>

static const int servoPin = 13;
Servo servo1;
int counter;

// Define the pins used by the transceiver module
#define ss 5
#define rst 14
#define dio0 2

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  servo1.attach(servoPin);
  while (!Serial);
  Serial.println("LoRa Sender");

  // Setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);

  while (!LoRa.begin(433E6)) {
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0xAB);
  Serial.println("LoRa Initializing OK!");
}

void loop() {
  // Check if there is data available from the Serial Monitor
  if (Serial.available()) {
    String inputString = Serial.readStringUntil('\n'); // Read input from Serial Monitor
    inputString.trim(); // Remove leading and trailing spaces
    
    Serial.print("Sending packet: ");
    Serial.println(inputString);

    // Send LoRa packet to receiver
    LoRa.beginPacket();
    LoRa.print(inputString);
    LoRa.endPacket();
  }
  /*
  // Check for received LoRa packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Received packet: ");
    
    String LoRaData = "";
    while (LoRa.available()) {
      LoRaData += (char)LoRa.read();
    }
    Serial.println(LoRaData);

    // Control servo if "90" is received
    if (LoRaData == "90") {
      for (int posDegrees = 0; posDegrees <= 180; posDegrees++) {
        servo1.write(posDegrees);
        Serial.println(posDegrees);
        delay(20);
      }
      for (int posDegrees = 180; posDegrees >= 0; posDegrees--) {
        servo1.write(posDegrees);
        Serial.println(posDegrees);
        delay(20);
      }
    }
    Serial.print("RSSI: ");
    Serial.println(LoRa.packetRssi());
  }*/
}
