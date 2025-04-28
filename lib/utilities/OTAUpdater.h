#ifndef OTA_UPDATER_H
#define OTA_UPDATER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Update.h>
#include "Config.h"
#include "DeviceManager.h"

class MQTTClient; // Forward declaration

class OTAUpdater {
public:
    OTAUpdater(Config* config, DeviceManager* deviceManager);
    
    void handleUpdateCommand(const JsonDocument& doc, MQTTClient* mqttClient);
    
private:
    Config* _config;
    DeviceManager* _deviceManager;
    
    bool performOTAUpdate(const char* url, const char* expectedChecksum, MQTTClient* mqttClient);
};

#endif // OTA_UPDATER_H