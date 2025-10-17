#pragma once

#include <Arduino.h>

#define UV_REF_VOLTAGE 3.3
#define UV_ADC_RESOLUTION 4095
#define UV_SENSOR_PIN 32

struct UVReference  {
    int uvIndex;
    int voltageMin_mV;
    int voltageMax_mV;
};

class GUVASensor {
    private:
        static const int BURST_SAMPLES = 10;
        float _uvIndex;
        bool _isValid;
        static const UVReference _uvTable[12];

        float readVoltageWithBurstSampling();
        float calculateUVIndex(float voltage_mV);

    public:
        GUVASensor();

        bool begin();
        bool update();
        float getUVIndex() const { return _uvIndex; }
        bool isValid() const { return _isValid; }
};