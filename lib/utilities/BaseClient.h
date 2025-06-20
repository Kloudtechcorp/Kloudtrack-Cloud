#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <PubSubClient.h>

#include "Config.h"
#include "DateTime.h"
#include "SdCard.h"
#include "SensorManager.h"

class BaseClient
{
protected:
    static constexpr const int MAX_CONNECT_RETRIES = 5;
    static constexpr const unsigned long MAX_RECONNECT_DELAY = 60000;

    static constexpr const unsigned long SYNC_INTERVAL = 300000; // Attempt to sync every 5 minutes (300,000 ms)
    static constexpr const unsigned long WEATHER_PUBLISH_INTERVAL = 60000;

    Config _config;
    SdCard _sdCard;
    SensorManager _sensorManager;
    Preferences _preferences;
    PubSubClient _mqttClient;

    int _mqttRetryCount = 0;
    unsigned long _reconnectDelay = 1000;

    unsigned long _lastSyncAttempt = 0;
    unsigned long _lastWeatherPublish = 0;

    // Initializes the client with a specific configuration.
    virtual void beginInner() = 0;

    DateTime _dateTime;
    virtual int updateDateTime() = 0;

    bool _isDeviceActivated = false;
    bool isDeviceActivated();
    void activateDevice(bool activate);

    bool syncOfflineData();
    void checkAndSyncData();

public:
    BaseClient(Client &client);

    // Initializes the client.
    void begin();

    // Executes repeating tasks.
    virtual void loop() = 0;

    // Connects to the server.
    virtual void connect() = 0;

    // Returns whether client is connected to the network.
    virtual bool isNetworkConnected() = 0;
};
