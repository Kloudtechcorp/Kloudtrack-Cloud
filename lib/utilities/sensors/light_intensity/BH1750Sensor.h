#pragma once

#include <Arduino.h>
#include <BH1750.h>

class BH1750Sensor {
    private:
        BH1750 _light;
        float _lux;
        bool _isValid;

    public:
        BH1750Sensor();

        bool begin();
        bool update();

        float getLux() const { return _lux; }
        bool isValid() const { return _isValid; }
};