
// Define TX and RX pins for UART (change if needed)
#define TXD1 19
#define RXD1 21

// Use Serial1 for UART communication
HardwareSerial mySerial(2);

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, RXD1, TXD1);  // UART setup
  
  Serial.println("ESP32 UART Receiver");
}

void loop() {
  // Check if data is available to read
    // Read data and display it
    String message = mySerial.readStringUntil('\n');
    Serial.print(message);
}
