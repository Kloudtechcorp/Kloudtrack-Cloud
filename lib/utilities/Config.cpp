#include "Config.h"

void Config::updateDeviceIdFromMac()
{
    uint64_t mac = ESP.getEfuseMac();
    sprintf(_deviceId, "KT-%04X%08X",
            (uint16_t)(mac >> 32),
            (uint32_t)mac);
}

void Config::updateAwsIotDeviceTopics()
{
    snprintf(_awsIotDeviceCommandTopic, 50, "kloudtrack/%s/command", _deviceId);
    snprintf(_awsIotDeviceWeatherTopic, 50, "kloudtrack/%s/data", _deviceId);
}

Config::Config()
{
}