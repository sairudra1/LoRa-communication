#include <SPI.h>
#include <LoRa.h>
#include <HardwareSerial.h>
#include <MAVLink.h>
#include <ESP32Servo.h>

HardwareSerial mySerial(1);  // Use UART1 on ESP32 (GPIO16 RX, GPIO17 TX)


#define servoPin 13 // Servo control pin
#define ss 5
#define rst 14
#define dio0 2

Servo myServo;
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 10000; // 10 seconds
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

  Serial.begin(115200);  // Debugging output to Serial Monitor
  mySerial.begin(57600, SERIAL_8N1, 16, 17);  // Pixhawk connection (adjust baud rate if needed)

  // Attach the servo to its pin
  myServo.attach(servoPin);
  myServo.write(0); // Set servo to initial position
}

void loop() {
  // Send data every sendInterval milliseconds/////////////////////////////////////////////////////////////////////////
  mavlink_message_t msg;
  mavlink_status_t status;
  //delay(1000);
  if (millis() - lastSendTime > sendInterval) {
    while (mySerial.available()) {
      uint8_t byte = mySerial.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
        switch (msg.msgid) {
          case MAVLINK_MSG_ID_HEARTBEAT: {
              mavlink_heartbeat_t heartbeat;
              mavlink_msg_heartbeat_decode(&msg, &heartbeat);
              Serial.print("System Status: ");
              Serial.println(heartbeat.system_status);
              Serial.print("Arm Status: ");
              Serial.println((heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? "Armed" : "Disarmed");

              LoRa.beginPacket();
              LoRa.print("System Status: ");  LoRa.print(" ");
              LoRa.print(heartbeat.system_status);  LoRa.print(" ");
              
              LoRa.endPacket();
              LoRa.beginPacket();
              
              LoRa.print("Arm Status: ");  LoRa.print(" ");
              LoRa.print((heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? "Armed" : "Disarmed");  LoRa.print(" ");
              LoRa.endPacket();
              break;
            }

          case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
              mavlink_global_position_int_t pos;
              mavlink_msg_global_position_int_decode(&msg, &pos);
              Serial.print("Latitude: "); Serial.println(pos.lat / 1E7, 7);
              Serial.print("Longitude: "); Serial.println(pos.lon / 1E7, 7);
              Serial.print("Altitude: "); Serial.println(pos.alt / 1000.0);
              Serial.print("Relative Altitude: "); Serial.println(pos.relative_alt / 1000.0);
              LoRa.beginPacket();
              LoRa.print("Latitude: ");  LoRa.print(" "); LoRa.print(pos.lat / 1E7, 7);  LoRa.print(" ");
              
              LoRa.endPacket();
              LoRa.beginPacket();
              
              LoRa.print("Longitude: ");  LoRa.print(" "); LoRa.print(pos.lon / 1E7, 7);  LoRa.print(" ");

              
              LoRa.endPacket();
              LoRa.beginPacket();
              
              LoRa.print("Altitude: ");  LoRa.print(" "); LoRa.print(pos.alt / 1000.0);  LoRa.print(" ");

              
              LoRa.endPacket();
              LoRa.beginPacket();
              
              LoRa.print("Relative Altitude: ");  LoRa.print(" "); LoRa.print(pos.relative_alt / 1000.0);  LoRa.print(" ");
              LoRa.endPacket();
              break;
            }

          case MAVLINK_MSG_ID_VFR_HUD: {
              mavlink_vfr_hud_t hud;
              mavlink_msg_vfr_hud_decode(&msg, &hud);
              Serial.print("Airspeed: "); Serial.println(hud.airspeed);
              Serial.print("Groundspeed: "); Serial.println(hud.groundspeed);
              Serial.print("Altitude: "); Serial.println(hud.alt);
              Serial.print("Heading: "); Serial.println(hud.heading);
              LoRa.beginPacket();
              LoRa.print("Airspeed: "); LoRa.print(" "); LoRa.print(hud.airspeed); LoRa.print(" ");
              
              LoRa.endPacket();
              LoRa.beginPacket();
              
              LoRa.print("Groundspeed: ");  LoRa.print(" "); LoRa.print(hud.groundspeed); LoRa.print(" ");
              
              LoRa.endPacket();
              LoRa.beginPacket();
              
              LoRa.print("Altitude: "); LoRa.print(" "); LoRa.print(hud.alt);  LoRa.print(" ");
              
              LoRa.endPacket();
              LoRa.beginPacket();
              
              LoRa.print("Heading: ");  LoRa.print(" "); LoRa.print(hud.heading);  LoRa.print(" ");
              LoRa.endPacket();
              break;
            }

          case MAVLINK_MSG_ID_SYS_STATUS: {
              mavlink_sys_status_t sys_status;
              mavlink_msg_sys_status_decode(&msg, &sys_status);
              Serial.print("Battery Voltage: "); Serial.println(sys_status.voltage_battery / 1000.0);
              Serial.print("Battery Current: "); Serial.println(sys_status.current_battery / 100.0);
              LoRa.beginPacket();

              LoRa.print("Battery Voltage: ");  LoRa.print(" "); LoRa.print(sys_status.voltage_battery / 1000.0);  LoRa.print(" ");
              
              LoRa.endPacket();
              LoRa.beginPacket();
              
              LoRa.print("Battery Current: ");  LoRa.print(" "); LoRa.print(sys_status.current_battery / 100.0); LoRa.print(" ");

              LoRa.endPacket();


              break;
            }

          case MAVLINK_MSG_ID_ATTITUDE: {
              mavlink_attitude_t attitude;
              mavlink_msg_attitude_decode(&msg, &attitude);
              Serial.print("Roll: "); Serial.println(attitude.roll * (180.0 / 3.14159));
              Serial.print("Pitch: "); Serial.println(attitude.pitch * (180.0 / 3.14159));
              Serial.print("Yaw: "); Serial.println(attitude.yaw * (180.0 / 3.14159));
              LoRa.beginPacket();

              LoRa.print("Roll: ");  LoRa.print(" "); LoRa.print(attitude.roll * (180.0 / 3.14159)); LoRa.print(" ");
              
              LoRa.endPacket();
              LoRa.beginPacket();
              
              LoRa.print("Pitch: "); LoRa.print(" "); LoRa.print(attitude.pitch * (180.0 / 3.14159)); LoRa.print(" ");

             
              LoRa.endPacket();
              LoRa.beginPacket();
              
              LoRa.print("Yaw: "); LoRa.print(" "); LoRa.print(attitude.yaw * (180.0 / 3.14159)); LoRa.print(" ");

              LoRa.endPacket();




              break;
            }
        }
      }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    sendMessage();
    lastSendTime = millis();
  }

  receiveMessage();
}

void sendMessage() {
  Serial.print("Sending packet: ");
  Serial.println(counter);
  /*
    LoRa.beginPacket();
    LoRa.print("hello ");
    LoRa.print(counter);
    LoRa.endPacket();
  */
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

 /*   if (LoRaData == "90") {
      Serial.println("90 data packet received");
    }*/

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
