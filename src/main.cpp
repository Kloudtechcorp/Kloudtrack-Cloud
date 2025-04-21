// Remote Firmware Update Test

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <Update.h>
#include "secrets.h"
#include <ArduinoJson.h>

#define AWS_IOT_SUBSCRIBE_TOPIC "Kloudtrack/ota/cmd"
#define DEVICE_ID "Kloudtrack-1"

#define OTA_URL "https://kloudtrack-firmware-test.s3.ap-southeast-1.amazonaws.com/firmware.bin"
#define OTA_VERSION "1.0.0"

WiFiClientSecure net;
PubSubClient client(net);

void performOTAUpdate() {
    HTTPClient http;

    // Start the OTA update process
    Serial.println("Attempting to download firmware...");

    http.begin(net, OTA_URL);  // Specify the firmware URL

    // Specify the certificate if using HTTPS
    http.addHeader("Content-Type", "application/octet-stream");

    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
        WiFiClient* stream = http.getStreamPtr();

        // Prepare for OTA update
        if (Update.begin(UPDATE_SIZE_UNKNOWN)) {
            size_t written = 0;
            size_t total = http.getSize();

            // Read and write the stream
            while (http.connected() && (written < total)) {
                int len = stream->available();
                if (len > 0) {
                    uint8_t buffer[128];
                    int bytesRead = stream->read(buffer, sizeof(buffer));
                    written += Update.write(buffer, bytesRead);

                    // Log progress
                    Serial.printf("Written: %d bytes\n", written);
                    Serial.printf("Expected: %d bytes\n", total);
                }
            }

            // Check if the update is complete
            if (written == total) {
                Serial.println("Firmware download successful, updating...");
                if (Update.end()) {
                    if (Update.isFinished()) {
                        Serial.println("OTA Update completed, restarting...");
                        ESP.restart();
                    } else {
                        Serial.println("Error: OTA Update not finished.");
                    }
                } else {
                    Serial.println("Error during the update process");
                }
            } else {
                Serial.println("Error: Firmware size mismatch");
            }
        } else {
            Serial.println("Error: Not enough space for update");
        }
    } else {
        Serial.println("Error: Unable to download firmware");
    }

    http.end();  // End the HTTP request
}


void messageHandler(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message received on topic: ");
    Serial.println(topic);

    // Allocate JSON document
    StaticJsonDocument<256> doc;

    // Parse payload
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.f_str());
        return;
    }

    // Extract and act on the "message" field
    const char* command = doc["message"];
    if (command && String(command) == "start OTA") {
        Serial.println("Starting OTA update...");
        performOTAUpdate();
    } else {
        Serial.println("Invalid or no command received");
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 OTA Update Test");
    Serial.println("Firwmware Version: " + String(OTA_VERSION));

    // Connect to WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Connect to IoT Core
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    client.setServer(AWS_IOT_ENDPOINT, 8883);
    client.setCallback(messageHandler);

    while (!client.connected()) {
        Serial.println(".");
        if (client.connect(DEVICE_ID)) {
            Serial.println("Connected to AWS IoT");
            client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
        } else {
            Serial.print("Failed to connect, rc=");
            Serial.print(client.state());
            delay(2000);
        }
    }   
}

void loop() {
    client.loop();
}

