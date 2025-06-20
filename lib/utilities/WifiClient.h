#pragma once

#include <ArduinoJson.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>

#include "BaseClient.h"

class WifiClient : public BaseClient
{
private:
    static constexpr const int MAX_CONNECT_RETRIES = 20;

    WiFiClientSecure _wifiClient;
    WiFiUDP _ntpUdp;
    NTPClient _timeClient;

    int _wifiRetryCount = 0;

    void connectWifi();
    void connectToAws();

    void handleMqttMessage(char *topic, byte *payload, unsigned int length);
    void handleUpdateCommand(const JsonDocument &doc);

    bool performOTAUpdate(const String &url);

    String generateWeatherDataJson();

    void publishUpdateStatus(const char *status, const char *message);
    void publishWeatherData();
    void publishStatusReport();

protected:
    virtual int updateDateTime();

public:
    WifiClient(const char *ssid, const char *password);

    virtual void beginInner();

    inline virtual void connect()
    {
        connectWifi();
        connectToAws();
    }

    inline virtual bool isNetworkConnected()
    {
        return WiFi.isConnected();
    }

    virtual void loop();
};
