#pragma once

#include <Arduino.h>

class Config
{
private:
    char _deviceId[20];
    char _awsIotDeviceCommandTopic[50];
    char _awsIotDeviceWeatherTopic[50];

    void updateDeviceIdFromMac();
    void updateAwsIotDeviceTopics();

public:
    static constexpr const char *FIRMWARE_VERSION = "2.3.3";

    Config();

    void begin()
    {
        // Get stored device credentials
        updateDeviceIdFromMac();

        // Configure device-specific topics using stored or default credentials
        updateAwsIotDeviceTopics();
    }

    inline const char *getDeviceId() const
    {
        return _deviceId;
    }

    inline const char *getAwsIotDeviceCommandTopic() const
    {
        return _awsIotDeviceCommandTopic;
    }

    inline const char *getAwsIotDeviceWeatherTopic() const
    {
        return _awsIotDeviceWeatherTopic;
    }
};
