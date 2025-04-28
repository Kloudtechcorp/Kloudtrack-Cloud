#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H

#include <Arduino.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include "Config.h"

class MQTTClient;

class DeviceManager {
    public:
        DeviceManager(Config* config);

        // Activation management
        bool isActivated() const { return _activated; }
        void activateDevice(bool activate, MQTTClient* mqttClient);
        void handleActivationCommand(const JsonDocument& doc, MQTTClient* mqttClient);

    private:
        Config* _config;
        Preferences _preferences;
        bool _activated;
};

#endif