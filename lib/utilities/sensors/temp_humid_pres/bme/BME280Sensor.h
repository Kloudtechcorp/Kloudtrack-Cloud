#pragma once

#include <Arduino.h>
#include <Adafruit_BME280.h>

class BME280Sensor {
    private:
        Adafruit_BME280 _bme;
        float _temperature;
        float _humidity;
        float _pressure;
        bool _isValid;

    public:
        BME280Sensor();

        bool begin();
        bool update();

        float getTemperature() const { return _temperature; }
        float getHumidity() const { return _humidity; }
        float getPressure() const { return _pressure; }
        bool isValid() const { return _isValid; }
};