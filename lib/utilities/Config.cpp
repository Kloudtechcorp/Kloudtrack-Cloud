#include "Config.h"

Config::Config() {
    // Generate unique device ID based on MAC Address
    getMacAddress(_deviceId);

    // Configure device-specific topics
    sprintf(_deviceId, "Kloudtrack/device/%s/cmd", _deviceId);
    sprintf(_deviceWeatherTopic, "Kloudtrack/device/%s/weather", _deviceId);
}

void Config::getMacAddress(char* macStr) {
    uint64_t mac = ESP.getEfuseMac();
    sprintf(macStr, "KT-%04X%08X", (uint16_t)(mac >> 32), (uint32_t)mac);
}