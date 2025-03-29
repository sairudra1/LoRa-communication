#include <SPI.h>
#include <LoRa.h>

#include <ESP32Servo.h>
#define servoPin 13 // Servo control pin
#define ss 5
#define rst 14
#define dio0 2

Servo myServo;
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

  // Attach the servo to its pin
  myServo.attach(servoPin);
  myServo.write(0); // Set servo to initial position
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
  LoRa.print("hello ");
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
    
    if (LoRaData == "90") {
      Serial.println("90 data packet received");
    }
    
    if (LoRaData == "drop") {
      Serial.println("Drop command received. Moving servo...");
      myServo.write(90); // Move servo to 90 degrees
      delay(1000); // Hold position for 1 second
      myServo.write(0); // Return servo to initial position
    }
    
    Serial.print("RSSI: ");
    Serial.println(LoRa.packetRssi());
  }
}
