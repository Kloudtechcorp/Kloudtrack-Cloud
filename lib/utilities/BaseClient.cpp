#include "BaseClient.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>

#include "Config.h"

// Function to check if device is activated
bool BaseClient::isDeviceActivated()
{
    _preferences.begin("kloudtrack", false);
    bool activated = _preferences.getBool("activated", false);
    _preferences.end();
    return activated;
}

// Function to activate the device
void BaseClient::activateDevice(bool activate)
{
    _preferences.begin("kloudtrack", false);
    _preferences.putBool("activated", activate);
    _preferences.end();
    _isDeviceActivated = activate;

    updateDateTime();
    // Publish activation status
    StaticJsonDocument<200> doc;
    doc["device_id"] = _config.getDeviceId();
    doc["firmware_version"] = Config::FIRMWARE_VERSION;
    doc["activated"] = activate;
    doc["recorded_at"] = _dateTime.asStr();

    String jsonStr;
    serializeJson(doc, jsonStr);

    // Publish to device-specific activation topic
    _mqttClient.publish(_config.getAwsIotDeviceCommandTopic(), jsonStr.c_str());

    Serial.printf("Device %s\n", activate ? "ACTIVATED" : "DEACTIVATED");
}

// Sync offline data with cloud
bool BaseClient::syncOfflineData()
{
    if (!_sdCard.isAvailable())
    {
        Serial.println("SD card not available for syncing");
        return false;
    }

    if (!isNetworkConnected() || !_mqttClient.connected())
    {
        Serial.println("Network connection not available for syncing");
        return false;
    }

    Serial.println("Starting data synchronization...");

    int syncedCount = 0;
    File root = _sdCard.openDataDir();
    if (!root)
    {
        Serial.println("Failed to open data directory");
        return false;
    }

    if (!root.isDirectory())
    {
        Serial.println("Data path is not a directory");
        root.close();
        return false;
    }

    // Process up to 10 files at a time to avoid blocking the main loop for too long
    int processLimit = 10;
    File file = root.openNextFile();
    while (file && processLimit > 0)
    {
        processLimit--;

        if (!file.isDirectory())
        {
            // Read the file content
            String data = "";
            while (file.available())
            {
                data += (char)file.read();
            }
            file.close();

            // Publish the data
            if (_mqttClient.publish(_config.getAwsIotDeviceWeatherTopic(), data.c_str()))
            {
                // Delete the file after successful publish
                if (_sdCard.deleteDataFile(file.name()))
                {
                    syncedCount++;
                }
            }
            else
            {
                Serial.printf("Failed to publish data from file: %s\n", file.name());
            }
        }
        else
        {
            file.close();
        }

        // Get next file
        file = root.openNextFile();
    }

    root.close();
    Serial.printf("Synchronized %d files\n", syncedCount);

    return syncedCount > 0;
}

// Check connectivity and sync data when available
void BaseClient::checkAndSyncData()
{
    if (isNetworkConnected() && _mqttClient.connected() && _sdCard.getPendingRecords() > 0)
    {
        unsigned long currentMillis = millis();
        if (currentMillis - _lastSyncAttempt >= SYNC_INTERVAL)
        {
            _lastSyncAttempt = currentMillis;
            Serial.println("Auto-syncing offline data...");
            syncOfflineData();
        }
    }
}

BaseClient::BaseClient(Client &client) : _mqttClient(client)
{
}

void BaseClient::begin()
{
    _config.begin();

    Serial.println("\n---------------------------------");
    Serial.println("ESP32 Weather Station");
    Serial.printf("Device ID: %s\n", _config.getDeviceId());
    Serial.printf("Current Firmware Version: %s\n", Config::FIRMWARE_VERSION);
    Serial.println("---------------------------------");

    // Check activation status
    _isDeviceActivated = isDeviceActivated();
    Serial.printf("Device activation status: %s\n", _isDeviceActivated ? "ACTIVATED" : "NOT ACTIVATED");

    // Initialize SD card
    _sdCard.begin();
    Serial.printf("SD Card status: %s\n", _sdCard.isAvailable() ? "AVAILABLE" : "NOT AVAILABLE");

    // Connect to WiFi and AWS IoT
    connect();

    _lastWeatherPublish = millis();
    _lastSyncAttempt = millis();

    // Run subclass-specific initializations
    beginInner();
}
