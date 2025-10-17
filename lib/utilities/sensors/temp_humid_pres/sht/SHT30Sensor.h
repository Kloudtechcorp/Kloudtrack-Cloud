#pragma once

#include <Arduino.h>
#include <SHT31.h>

class SHT30Sensor {
    private:
        SHT31 _sht;
        float _temperature;
        float _humidity;
        bool _isValid;

    public:
        SHT30Sensor();

        bool begin();
        bool update();

        float getTemperature() const { return _temperature; }
        float getHumidity() const { return _humidity; }
        bool isValid() const { return _isValid; }
};