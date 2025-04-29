#define BLYNK_TEMPLATE_ID "TMPL6R8kML26F"
#define BLYNK_TEMPLATE_NAME "nhung lam the"
#define BLYNK_AUTH_TOKEN "8lV30Ef0NO2Fp4_RikeukONTXXCnbFZc"

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <ESP8266HTTPClient.h>
#include <BlynkSimpleEsp8266.h>
#include "MAX30100_PulseOximeter.h"

// ESP8266 -> Arduino: From pin D3 to 6
// Arduino -> ESP8266: From pin 11 to D4

static const uint8_t SerialReceivePin = D5;
static const uint8_t SerialTransmitPin = D6;

PulseOximeter pox;
SoftwareSerial swSerial(SerialReceivePin, SerialTransmitPin, false);

void setup() {
    // Wire.begin();
    Serial.begin(9600);
    swSerial.begin(9600);

    Serial.println("PulseOximeter: Initializing.");
    if (!pox.begin()) {
        Serial.println("PulseOximeter: Initialization failed!");
        for (;;);
    }

    Serial.println("PulseOximeter: Initialized!");

    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

    pinMode(SerialReceivePin, INPUT);
    pinMode(SerialTransmitPin, OUTPUT);
}

static unsigned long reportTime;

void loop() {
    pox.update();

    if (millis() > reportTime) {
        const float heartRate = pox.getHeartRate();
        const uint8_t spO2 = std::clamp<uint8_t>(pox.getSpO2(), 0, 100);

        if (heartRate != 0 || spO2 != 0) {
            uint8_t buffer[9];
            memcpy(buffer, "HRIF", 4);
            *reinterpret_cast<float*>(buffer + 4) = heartRate;
            buffer[8] = spO2;

            swSerial.write(buffer, sizeof(buffer));
            Serial.println("Transmit health info: " + String(heartRate) + ", " + String(spO2));
        }

        reportTime = millis() + 1000;
    }
}

// const char* wifiSSID = "DESKTOP-5B623ID 9507";
// const char* wifiPassword = "1j63{11X";
//
// WiFiServer server(5912);
// SoftwareSerial swSerial(D2, -1);
//
// void setup() {
//     Serial.begin(9600);
//     swSerial.begin(9600);
//
//     pinMode(D0, OUTPUT);
//     pinMode(D1, OUTPUT);
//     pinMode(D2, INPUT);
//
//     Serial.println("Connecting to Blynk...");
//
//     Blynk.begin(BLYNK_AUTH_TOKEN, wifiSSID, wifiPassword);
//
//     Serial.println("Connected to Blynk.");
// }
//
// void loop() {
//     Blynk.run();
// }
//
// BLYNK_WRITE(V1) {
//     digitalWrite(D1, param.asInt());
// }