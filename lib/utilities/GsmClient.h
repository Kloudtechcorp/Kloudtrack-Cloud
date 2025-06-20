#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SSLClient.h>
#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>

#include "BaseClient.h"

class GsmClient : public BaseClient
{
private:
    static constexpr const int TIME_THRESHOLD = 120; // Allow up to 120s jump

    TinyGsm _modem;
    TinyGsmClient _gsmClient;
    SSLClient _sslClient;

    int _gsmRetryCount = 0;

    void updateBaudRate();
    void connectGsm();
    void connectToAws();

    void handleMqttMessage(char *topic, byte *payload, unsigned int length);
    void handleUpdateCommand(const JsonDocument &doc);

    bool testServerConnection(const String &host, int port);
    bool performOTAUpdate(const String &url);

    String generateSensorStatusJSON();
    String generateStatusInfoJSON();
    String generateWeatherDataJson();

    void publishUpdateStatus(const char *status, const char *message);
    void publishWeatherData();
    void publishStatusReport();
    void publishSensorStatus();

protected:
    virtual int updateDateTime();

public:
    GsmClient();

    virtual void beginInner();

    inline virtual void connect()
    {
        connectGsm();
        connectToAws();
    }

    inline virtual bool isNetworkConnected()
    {
        return _modem.isNetworkConnected();
    }

    virtual void loop();
};
