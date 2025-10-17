#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "../connectivity/GSM/GsmConnection.h"
#include <PubSubClient.h>
#include "../configuration/Config.h"
#include "../time/DateTime.h"
#include "../storage/SdCard.h"
#include "../sensors/TestSensorManager.h"
#include <SSLClient.h>
#include <Preferences.h>
#include <Update.h>
#include <ArduinoHttpClient.h>
#include <TinyGsmClient.h>
#include <esp_task_wdt.h>
#define SerialMon Serial

// Error logging levels
enum LogLevel {
    LOG_DEBUG = 0,
    LOG_INFO = 1,
    LOG_WARNING = 2,
    LOG_ERROR = 3,
    LOG_CRITICAL = 4
};

class MqttUtils {
    private:
        // Network health tracking
        struct NetworkHealth {
            unsigned long gsmUptime = 0;
            unsigned long mqttUptime = 0;
            unsigned long lastGsmDisconnect = 0;
            unsigned long lastMqttDisconnect = 0;
            int gsmReconnectCount = 0;
            int mqttReconnectCount = 0;
            int signalQuality = 0;
            bool networkHealthy = false;
        } _networkHealth;
        
        SSLClient* _sslClient;
        PubSubClient _mqttClient;
        Config _config;
        Preferences _preferences;
        DateTime _dateTime;
        SdCard _sdCard;
        TestSensorManager _testSensorManager;
        HardwareSerial _serialAT;
        TinyGsm* _modem;
        LogLevel _currentLogLevel;
        char *topic;
        byte *payload;
        unsigned int length;
        bool _isActivated;
        unsigned long _lastSyncAttempt = 0;
        static const unsigned long SYNC_INTERVAL = 300000;

        // Monitoring and logging
        void logMessage(LogLevel level, const char* module, const char* message);
        void publishUpdateStatus(const char *status, const char *message);

        // Publishing data
        void publishSensorStatus();
        void publishStatusReport();
        
        String generateSensorStatusJSON();
        String generateWeatherDataJson();
        String generateStatusInfoJSON();
        String generateNetworkHealthJson();

        // Syncing data
        bool syncOfflineData();

        // Device management
        void activateDevice(bool activate);

        // OTA 
        void parseURL(const String &url, String &host, int &port, String &path);
        bool testServerConnection(const String &host, int port);
        bool performOTAUpdate(const String &url);
        void handleUpdateCommand(const JsonDocument& doc);

    public:
        MqttUtils(SSLClient* sslClient, TinyGsm* modem);
        void messageHandler(char *topic, byte *payload, unsigned int length);
        void loop(); // Service MQTT client
        bool isDeviceActivated();
        void publishWeatherData();
        void updateNetworkHealth();
        void checkAndSyncData();
};