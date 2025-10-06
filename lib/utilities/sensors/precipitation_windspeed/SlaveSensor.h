#pragma once

#include <Arduino.h>
#include <Wire.h>

// Slave Sensor Configurations
#define SLAVE_ADDRESS 0x03

class SlaveSensor {
    private:
        static constexpr float rainGaugeTipValue = 0.1099;
        static constexpr float anemometerRadius = 0.075;
        static constexpr int period = 60;

        float _circumference;
        float _windSpeed;
        float _precipitation;
        bool _isValid;

    public:
        SlaveSensor();

        bool begin();
        bool update();

        float getWindSpeed() const { return _windSpeed; }
        float getPrecipitation() const { return _precipitation; }
        bool isValid() const { return _isValid; }
};