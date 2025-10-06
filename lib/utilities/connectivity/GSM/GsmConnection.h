#pragma once

#define SerialMon Serial
#define TINY_GSM_MODEM_SIM7600
#include <Arduino.h>
#include <TinyGsmClient.h>
#include <SSLClient.h>
#include <ArduinoHttpClient.h>
#include <esp_task_wdt.h>
#include "../../time/DateTime.h"
#include "../../configuration/Config.h"

#define UART_BAUD 115200
#define RESET_PIN 5
#define POWER_PIN 4
#define TX_PIN 26
#define RX_PIN 27

class GsmConnection {
    private:
        HardwareSerial SerialAT;
        TinyGsm _modem;
        TinyGsmClient _baseClient;
        SSLClient _sslClient;
        Config _config;
        bool _isConnected;

        void setupModem();
        void configureSSL();
        
    public:
        GsmConnection();
        ~GsmConnection();

        void gsmConnect();
        void gsmDisconnect();
        uint32_t AutoBaud();
        bool isGsmConnected();

        SSLClient* getSslClient() { return &_sslClient; }
        TinyGsm* getModem() { return &_modem; }
};