#ifndef WEATHER_SENSOR_H
#define WEATHER_SENSOR_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "Config.h"

class MQTTClient; // Forward declaration

class WeatherSensor {
public:
    WeatherSensor(Config* config);
    
    void update(MQTTClient* mqttClient, bool deviceActive);
    void publishData(MQTTClient* mqttClient);
    
private:
    Config* _config;
    unsigned long _lastPublishTime;
};

#endif // WEATHER_SENSOR_H