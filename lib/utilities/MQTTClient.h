#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "Config.h"
#include "DeviceManager.h"
#include "OTAUpdater.h"
#include "WeatherSensor.h"

class MQTTClient {
public:
    MQTTClient(Config* config, DeviceManager* deviceManager, OTAUpdater* otaUpdater, WeatherSensor* weatherSensor);
    
    void connect();
    bool isConnected() { return _client.connected(); }
    void loop() { _client.loop(); }
    
    // Publishing methods
    bool publish(const char* topic, const char* payload);
    void publishStatus(const char* status, const char* message);
    
    // Message handling
    void messageHandler(char* topic, byte* payload, unsigned int length);
    
private:
    Config* _config;
    DeviceManager* _deviceManager;
    OTAUpdater* _otaUpdater;
    WeatherSensor* _weatherSensor;
    
    WiFiClientSecure _net;
    PubSubClient _client;
    
    int _retryCount;
    int _maxRetries;
    unsigned long _reconnectDelay;
    const unsigned long _maxReconnectDelay;
    
    void handleCommand(const JsonDocument& doc);
    
    static MQTTClient* _instance; // For static callback
    static void staticCallback(char* topic, byte* payload, unsigned int length);
};

#endif // MQTT_CLIENT_H