#include "OTAUpdater.h"
#include "MQTTClient.h" // For publishing status

OTAUpdater::OTAUpdater(Config* config, DeviceManager* deviceManager) :
    _config(config),
    _deviceManager(deviceManager) {
}

void OTAUpdater::handleUpdateCommand(const JsonDocument& doc, MQTTClient* mqttClient) {
    // Check if device is activated before proceeding
    if (!_deviceManager->isActivated()) {
        Serial.println("Update command rejected: Device not activated");
        mqttClient->publishStatus("Rejected", "Device not activated");
        return;
    }
    
    const char* version = doc["version"];
    const char* url = doc["url"];
    const char* checksum = doc["checksum"];
    bool force = doc["force"] | false;
    
    // Check if we need to update
    if (!version || !url) {
        mqttClient->publishStatus("Error", "Missing version or URL in update command");
        return;
    }
    
    // Compare versions
    if (strcmp(version, _config->getFirmwareVersion()) == 0 && !force) {
        Serial.println("Already running this version, update skipped");
        mqttClient->publishStatus("Skipped", "Already running the requested version");
        return;
    }
    
    Serial.printf("Starting update to version %s from %s\n", version, url);
    mqttClient->publishStatus("Started", "Starting firmware update");
    
    // Perform the update
    if (performOTAUpdate(url, checksum, mqttClient)) {
        ESP.restart();
    }
}

bool OTAUpdater::performOTAUpdate(const char* url, const char* expectedChecksum, MQTTClient* mqttClient) {
    // Check if device is activated
    if (!_deviceManager->isActivated()) {
        Serial.println("OTA update rejected: Device not activated");
        mqttClient->publishStatus("Rejected", "Device not activated");
        return false;
    }

    WiFiClientSecure client;
    HTTPClient http;
    bool success = false;
    
    // Skip certificate verification for firmware download
    client.setInsecure();
    
    // Start the OTA update process
    Serial.println("Attempting to download firmware...");
    mqttClient->publishStatus("Downloading", "Starting firmware download");

    http.begin(client, url);
    http.addHeader("Content-Type", "application/octet-stream");

    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
        WiFiClient* stream = http.getStreamPtr();
        size_t total = http.getSize();

        // Prepare for OTA update
        if (Update.begin(total)) {
            size_t written = 0;
            
            // Read and write the stream
            uint8_t buffer[1024];
            while (http.connected() && (written < total)) {
                size_t available = stream->available();
                if (available) {
                    size_t bytesRead = stream->readBytes(buffer, min(available, sizeof(buffer)));
                    size_t bytesWritten = Update.write(buffer, bytesRead);
                    if (bytesWritten > 0) {
                        written += bytesWritten;
                        // Log progress (every 10%)
                        if (written % (total / 10) < 1024) {
                            Serial.printf("Progress: %d%%\n", (written * 100) / total);
                            char progressMsg[32];
                            sprintf(progressMsg, "Progress: %d%%", (written * 100) / total);
                            mqttClient->publishStatus("Downloading", progressMsg);
                        }
                    } else {
                        Serial.println("Error writing update");
                        mqttClient->publishStatus("Failed", "Error writing update");
                        break;
                    }
                }
                delay(1);
            }

            // Check if the update is complete
            if (written == total) {
                Serial.println("Firmware download complete, verifying...");
                mqttClient->publishStatus("Verifying", "Firmware download complete, verifying");
                
                // Verify update before finalizing
                if (Update.end()) {
                    if (Update.isFinished()) {
                        Serial.println("OTA Update successful, restarting...");
                        mqttClient->publishStatus("Success", "Update successful, restarting");
                        success = true;
                        // Give MQTT message time to send before restart
                        delay(1000);
                    } else {
                        Serial.println("OTA Update not finished");
                        mqttClient->publishStatus("Failed", "Update not finished properly");
                    }
                } else {
                    Serial.printf("Error during update finalization: %d\n", Update.getError());
                    mqttClient->publishStatus("Failed", "Error during update finalization");
                }
            } else {
                Serial.println("Firmware size mismatch");
                mqttClient->publishStatus("Failed", "Firmware size mismatch");
                Update.abort();
            }
        } else {
            Serial.println("Not enough space for update");
            mqttClient->publishStatus("Failed", "Not enough space for update");
        }
    } else {
        Serial.printf("HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
        char errorMsg[64];
        sprintf(errorMsg, "HTTP error: %d", httpCode);
        mqttClient->publishStatus("Failed", errorMsg);
    }

    http.end();
    return success;
}