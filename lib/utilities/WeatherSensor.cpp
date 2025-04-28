#include "WeatherSensor.h"
#include "MQTTClient.h" // For publishing data

WeatherSensor::WeatherSensor(Config* config) :
    _config(config),
    _lastPublishTime(0) {
}

void WeatherSensor::update(MQTTClient* mqttClient, bool deviceActive) {
    unsigned long currentMillis = millis();
    
    // Publish weather data at regular intervals
    if (currentMillis - _lastPublishTime >= _config->getWeatherPublishInterval()) {
        _lastPublishTime = currentMillis;
        if (deviceActive) {
            publishData(mqttClient);
        }
    }
}

void WeatherSensor::publishData(MQTTClient* mqttClient) {
    // Generate random weather data
    float temperature = random(_config->getMinTemperature() * 100, _config->getMaxTemperature() * 100) / 100.0;
    float humidity = random(_config->getMinHumidity() * 100, _config->getMaxHumidity() * 100) / 100.0;
    float pressure = random(_config->getMinPressure() * 10, _config->getMaxPressure() * 10) / 10.0;
    float windSpeed = random(_config->getMinWindSpeed() * 100, _config->getMaxWindSpeed() * 100) / 100.0;
    
    // Get random wind direction (N, NE, E, SE, S, SW, W, NW)
    const char* windDirections[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    const char* windDirection = windDirections[random(0, 8)];
    
    // Get random weather condition
    const char* weatherConditions[] = {"Sunny", "Partly Cloudy", "Cloudy", "Rainy", "Thunderstorm", "Foggy", "Snowy"};
    const char* weatherCondition = weatherConditions[random(0, 7)];
    
    // Create JSON document
    StaticJsonDocument<256> doc;
    doc["device_id"] = _config->getDeviceId();
    doc["timestamp"] = millis();
    
    // Weather data
    JsonObject weather = doc.createNestedObject("weather");
    weather["temperature"] = temperature;
    weather["humidity"] = humidity;
    weather["pressure"] = pressure;
    weather["wind_speed"] = windSpeed;
    weather["wind_direction"] = windDirection;
    weather["condition"] = weatherCondition;
    
    // Device info
    JsonObject device = doc.createNestedObject("device");
    device["firmware"] = _config->getFirmwareVersion();
    device["activated"] = true;  // We only publish if activated
    
    // Serialize to JSON
    size_t msgLen = measureJson(doc) + 1;
    char* jsonBuffer = new char[msgLen];
    serializeJson(doc, jsonBuffer, msgLen);
    
    // Publish to both global weather topic and device-specific weather topic
    mqttClient->publish(_config->getWeatherTopic(), jsonBuffer);
    mqttClient->publish(_config->getDeviceWeatherTopic(), jsonBuffer);
    
    Serial.println("Published weather data:");
    Serial.println(jsonBuffer);
    delete[] jsonBuffer;
}