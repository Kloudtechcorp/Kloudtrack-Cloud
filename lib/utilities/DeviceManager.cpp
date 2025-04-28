#include "DeviceManager.h"
#include "MQTTClient.h"

DeviceManager::DeviceManager(Config* config) : _config(config) {
    // Check activation status from persistent storage
    _preferences.begin("kloudtrack", false);
    _activated = _preferences.getBool("activated", false);
    _preferences.end();

    Serial.printf("Device activation status: %s\n", _activated ? "ACTIVATED" : "NOT ACTIVATED");
}

void DeviceManager::activateDevice(bool activate, MQTTClient* mqttClient) {
    _preferences.begin("kloudtrack", false);
    _preferences.putBool("activated", activate);
    _preferences.end();
    _activated = activate;
    
    if (mqttClient != nullptr) {
        // Publish activation status
        StaticJsonDocument<200> doc;
        doc["device_id"] = _config->getDeviceId();
        doc["firmware_version"] = _config->getFirmwareVersion();
        doc["activated"] = activate;
        
        char jsonBuffer[256];
        serializeJson(doc, jsonBuffer);
        
        mqttClient->publish(_config->getStatusTopic(), jsonBuffer);
    }
    
    Serial.printf("Device %s\n", activate ? "ACTIVATED" : "DEACTIVATED");
}

void DeviceManager::handleActivationCommand(const JsonDocument& doc, MQTTClient* mqttClient) {
    const char* action = doc["action"];
    const char* key = doc["key"];
    
    if (!action) {
        Serial.println("Invalid activation command: missing action");
        return;
    }
    
    if (strcmp(action, "activate") == 0) {
        // Check if key is correct
        if (!key || strcmp(key, _config->getActivationKey()) != 0) {
            Serial.println("Activation rejected: Invalid key");
            if (mqttClient != nullptr) {
                mqttClient->publishStatus("Access_denied", "Invalid activation key");
            }
            return;
        }
        
        activateDevice(true, mqttClient);
        Serial.println("Device activated successfully");
        if (mqttClient != nullptr) {
            mqttClient->publishStatus("Activated", "Device activated successfully");
        }
    } 
    else if (strcmp(action, "deactivate") == 0) {
        // Check if key is correct
        if (!key || strcmp(key, _config->getActivationKey()) != 0) {
            Serial.println("Deactivation rejected: Invalid key");
            if (mqttClient != nullptr) {
                mqttClient->publishStatus("Access_denied", "Invalid activation key");
            }
            return;
        }
        
        activateDevice(false, mqttClient);
        Serial.println("Device deactivated");
        if (mqttClient != nullptr) {
            mqttClient->publishStatus("Deactivated", "Device deactivated");
        }
    }
    else {
        Serial.println("Unknown activation action");
    }
}