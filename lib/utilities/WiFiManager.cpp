#include "WiFiManager.h"

WiFiManager::WiFiManager(Config* config) : 
    _config(config), 
    _retryCount(0),
    _maxRetries(config->getMaxRetries()) {
}

void WiFiManager::connect() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Connecting to WiFi...");
        WiFi.begin(_config->getWiFiSSID(), _config->getWiFiPassword());
        
        unsigned long wifiStart = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart < 10000)) {
            delay(500);
            Serial.print(".");
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWiFi connected!");
            _retryCount = 0;
        } else {
            _retryCount++;
            Serial.printf("\nWiFi connection failed. Attempt %d/%d\n", _retryCount, _maxRetries);
        }
        
        if (_retryCount >= _maxRetries) {
            Serial.println("Maximum WiFi retries reached. Restarting ESP32.");
            ESP.restart();
        }
    }
}