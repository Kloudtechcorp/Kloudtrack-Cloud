#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include "secrets.h"

// MQTT Topics
#define AWS_IOT_SUBSCRIBE_TOPIC "Kloudtrack/ota/cmd"
#define AWS_IOT_WEATHER_TOPIC "kloudtrack/weather/data"
#define AWS_IOT_STATUS_TOPIC "Kloudtrack/ota/status"
#define AWS_IOT_ACTIVATION_TOPIC "Kloudtrack/admin/activation"
#define AWS_IOT_ALL_TOPIC "Kloudtrack/all/cmd"

// Current firmware version
#define FIRMWARE_VERSION "1.2.0"

// Activation key that admin must provide (should be unique per device)
#define ACTIVATION_KEY "KT-SECURE-KEY-12345"

// Weather data parameters
#define WEATHER_PUBLISH_INTERVAL 60000  // Publish weather data every minute

// Weather data ranges
#define MIN_TEMPERATURE 15.0
#define MAX_TEMPERATURE 35.0
#define MIN_HUMIDITY 30.0
#define MAX_HUMIDITY 90.0
#define MIN_PRESSURE 980.0
#define MAX_PRESSURE 1040.0
#define MIN_WIND_SPEED 0.0
#define MAX_WIND_SPEED 20.0

class Config {
    public:
        Config();

        // Getters
        const char* getDeviceId() const { return _deviceId; }
        const char* getFirmwareVersion() const { return FIRMWARE_VERSION; }
        const char* getWiFiSSID() const { return WIFI_SSID; }
        const char* getWiFiPassword() const { return WIFI_PASSWORD; }
        const char* getAWSEndpoint() const { return AWS_IOT_ENDPOINT; }
        const char* getAWSCertCA() const { return AWS_CERT_CA; }
        const char* getAWSCertCRT() const { return AWS_CERT_CRT; }
        const char* getAWSCertPrivateKey() const { return AWS_CERT_PRIVATE; }
        const char* getActivationKey() const { return ACTIVATION_KEY; }
        
        // Topic getters
        const char* getSubscribeTopic() const { return AWS_IOT_SUBSCRIBE_TOPIC; }
        const char* getWeatherTopic() const { return AWS_IOT_WEATHER_TOPIC; }
        const char* getStatusTopic() const { return AWS_IOT_STATUS_TOPIC; }
        const char* getActivationTopic() const { return AWS_IOT_ACTIVATION_TOPIC; }
        const char* getAllTopic() const { return AWS_IOT_ALL_TOPIC; }
        const char* getDeviceTopic() const { return _deviceTopic; }
        const char* getDeviceWeatherTopic() const { return _deviceWeatherTopic; }
        
        // Weather parameter getters
        int getWeatherPublishInterval() const { return WEATHER_PUBLISH_INTERVAL; }
        float getMinTemperature() const { return MIN_TEMPERATURE; }
        float getMaxTemperature() const { return MAX_TEMPERATURE; }
        float getMinHumidity() const { return MIN_HUMIDITY; }
        float getMaxHumidity() const { return MAX_HUMIDITY; }
        float getMinPressure() const { return MIN_PRESSURE; }
        float getMaxPressure() const { return MAX_PRESSURE; }
        float getMinWindSpeed() const { return MIN_WIND_SPEED; }
        float getMaxWindSpeed() const { return MAX_WIND_SPEED; }
        
        // Connection parameters
        int getMaxRetries() const { return 5; }
        unsigned long getInitialReconnectDelay() const { return 1000; }
        unsigned long getMaxReconnectDelay() const { return 60000; }

    private:
        void getMacAddress(char* macStr);
        
        char _deviceId[20];
        char _deviceTopic[50];
        char _deviceWeatherTopic[50];
};

#endif