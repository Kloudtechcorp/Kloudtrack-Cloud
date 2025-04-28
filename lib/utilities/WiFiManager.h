#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include "Config.h"

class WiFiManager {
public:
    WiFiManager(Config* config);
    
    void connect();
    bool isConnected() { return WiFi.status() == WL_CONNECTED; }
    
private:
    Config* _config;
    int _retryCount;
    int _maxRetries;
};

#endif // WIFI_MANAGER_H