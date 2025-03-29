#include <HardwareSerial.h>
#include <MAVLink.h>

HardwareSerial mySerial(1);  // Use UART1 on ESP32 (GPIO16 RX, GPIO17 TX)

void setup() {
    Serial.begin(115200);  // Debugging output to Serial Monitor
    mySerial.begin(57600, SERIAL_8N1, 16, 17);  // Pixhawk connection (adjust baud rate if needed)
}

void loop() {
    mavlink_message_t msg;
    mavlink_status_t status;
    //delay(1000);
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
                    break;
                }

                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                    mavlink_global_position_int_t pos;
                    mavlink_msg_global_position_int_decode(&msg, &pos);
                    Serial.print("Latitude: "); Serial.println(pos.lat / 1E7, 7);
                    Serial.print("Longitude: "); Serial.println(pos.lon / 1E7, 7);
                    Serial.print("Altitude: "); Serial.println(pos.alt / 1000.0);
                    Serial.print("Relative Altitude: "); Serial.println(pos.relative_alt / 1000.0);
                    break;
                }

                case MAVLINK_MSG_ID_VFR_HUD: {
                    mavlink_vfr_hud_t hud;
                    mavlink_msg_vfr_hud_decode(&msg, &hud);
                    Serial.print("Airspeed: "); Serial.println(hud.airspeed);
                    Serial.print("Groundspeed: "); Serial.println(hud.groundspeed);
                    Serial.print("Altitude: "); Serial.println(hud.alt);
                    Serial.print("Heading: "); Serial.println(hud.heading);
                    break;
                }

                case MAVLINK_MSG_ID_SYS_STATUS: {
                    mavlink_sys_status_t sys_status;
                    mavlink_msg_sys_status_decode(&msg, &sys_status);
                    Serial.print("Battery Voltage: "); Serial.println(sys_status.voltage_battery / 1000.0);
                    Serial.print("Battery Current: "); Serial.println(sys_status.current_battery / 100.0);
                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE: {
                    mavlink_attitude_t attitude;
                    mavlink_msg_attitude_decode(&msg, &attitude);
                    Serial.print("Roll: "); Serial.println(attitude.roll * (180.0 / 3.14159));
                    Serial.print("Pitch: "); Serial.println(attitude.pitch * (180.0 / 3.14159));
                    Serial.print("Yaw: "); Serial.println(attitude.yaw * (180.0 / 3.14159));
                    break;
                }
            }
        }
    }
}
