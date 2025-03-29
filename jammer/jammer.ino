#include "WiFi.h"
#include "esp_wifi.h"

void sendNoise() {
    uint8_t packet[128] = {0xFF};
    for (int i = 0; i < 128; i++) packet[i] = random(0, 256);
    esp_wifi_80211_tx(WIFI_IF_AP, packet, sizeof(packet), false);
}

void setup() {
    WiFi.mode(WIFI_MODE_AP);
    Serial.begin(115200);
    Serial.println("FlySky i6 Jammer Active");
}

void loop() {
    for (int ch = 2405; ch <= 2475; ch += 1) {
        esp_wifi_set_channel(ch % 14 + 1, WIFI_SECOND_CHAN_NONE);
        sendNoise();
        delay(5);  
    }
}
