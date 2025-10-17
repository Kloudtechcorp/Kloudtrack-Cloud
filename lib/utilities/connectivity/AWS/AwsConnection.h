#pragma once

#include <Arduino.h>
#include "../GSM/GsmConnection.h"
#include "../../MQTT/MqttUtils.h"
#include <PubSubClient.h>
#include <SSLClient.h>
#include <TinyGsmClient.h>
#include "../../configuration/Config.h"
#include "../../time/DateTime.h"
#include <Preferences.h>
#include <esp_task_wdt.h>
#define SerialMon Serial

class AwsConnection {
    private:
        PubSubClient _mqttClient;
        bool _isConnected;
        const char *_iotEndpoint;
        int _iotPort;
        const char* _caCert;
        const char* _deviceCert;
        const char* _privateKey;

        // Add these missing members:
        SSLClient* _sslClient;
        TinyGsm* _modem;
        Config _config;

        void setUpAws();

    public:
        AwsConnection(SSLClient* sslClient, TinyGsm* modem);
        ~AwsConnection();

        void awsConnect();
        void awsDisconnect();
        bool isAwsDisconnected();
        bool isAwsConnected();
};