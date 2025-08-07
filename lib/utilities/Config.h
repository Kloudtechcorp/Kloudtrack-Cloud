#pragma once

#include <Arduino.h>
#include <Preferences.h>

class Config
{
private:
    char _deviceId[20];
    char _awsIotDeviceCommandTopic[50];
    char _awsIotDeviceWeatherTopic[50];
    char _awsIotDeviceResponseTopic[50];
    
    // Credential storage
    char _wifiSsid[64];
    char _wifiPassword[64];
    char _apn[32];
    char _awsIotEndpoint[128];
    int _awsIotPort;
    
    Preferences _preferences;

    void updateDeviceIdFromMac();
    void updateAwsIotDeviceTopics();
    void loadCredentialsFromPreferences();
    void loadDefaultCredentials();

public:
    static constexpr const char *FIRMWARE_VERSION = "2.3.4";
    static constexpr const char *PREFERENCES_NAMESPACE = "credentials";

    Config();

    void begin()
    {
        // Initialize preferences
        _preferences.begin(PREFERENCES_NAMESPACE, false);
        
        // Load credentials from preferences (with fallback to defaults)
        loadCredentialsFromPreferences();
        
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

    inline const char *getAwsIotDeviceResponseTopic() const
    {
        return _awsIotDeviceResponseTopic;
    }
    
    // Credential getters
    inline const char *getWifiSsid() const { return _wifiSsid; }
    inline const char *getWifiPassword() const { return _wifiPassword; }
    inline const char *getApn() const { return _apn; }
    inline const char *getAwsIotEndpoint() const { return _awsIotEndpoint; }
    inline int getAwsIotPort() const { return _awsIotPort; }
    
    // Credential setters
    bool setWifiCredentials(const char* ssid, const char* password);
    bool setGsmCredentials(const char* apn);
    bool setAwsCredentials(const char* endpoint, int port);
    
    // Credential management
    bool hasWifiCredentials() const;
    bool hasGsmCredentials() const;
    bool hasAwsCredentials() const;
    void clearCredentials();
    void printCredentials();
};
