#include "GUVASensor.h"

const UVReference GUVASensor::_uvTable[12] = {
    {0, 0, 49},
    {1, 227, 317},
    {2, 318, 407},
    {3, 408, 502},
    {4, 503, 605},
    {5, 606, 695},
    {6, 696, 794},
    {7, 795, 880},
    {8, 881, 975},
    {9, 976, 1078},
    {10, 1079, 1169},
    {11, 1170, 9999}
};

GUVASensor::GUVASensor() : _uvIndex(NAN), _isValid(false) {
}

bool GUVASensor::begin() {
    pinMode(UV_SENSOR_PIN, INPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Allow ADC to stabilize
    delay(100);

    // Test if sensor is responding
    float testVoltage = readVoltageWithBurstSampling();
    if (isnan(testVoltage)) {
        _isValid = false;
        return false;
    }

    _isValid = true;
    return true;
}

float GUVASensor::readVoltageWithBurstSampling() {
    float totalVoltage = 0;
    int validReadings = 0;

    // Take multiple rapid readings for averaging
    for (int i = 0; i < BURST_SAMPLES; i++) {
        int sensorValue = analogRead(UV_SENSOR_PIN);

        // Validate reading is within expected ADC range
        if (sensorValue >= 0 && sensorValue <= UV_ADC_RESOLUTION) {
            float voltage_mV = (sensorValue / (float)UV_ADC_RESOLUTION) * UV_REF_VOLTAGE * 1000;
            totalVoltage += voltage_mV;
            validReadings++;
        }

        // Small delay between readings for stability
        delay(10);
    }

    // Return averaged result if we have valid readings
    if (validReadings > 0) {
        return totalVoltage / validReadings;
    }

    return NAN;
}


float GUVASensor::calculateUVIndex(const float voltage_mV) {
    if (voltage_mV < 0) {
        return 0.0;
    }
    
    for (int i = 0; i < 12; i++) {
        if (voltage_mV >= _uvTable[i].voltageMin_mV && voltage_mV <= _uvTable[i].voltageMax_mV) {
            return (float)_uvTable[i].uvIndex;
        }
    }
    
    if (voltage_mV > 1170) {
        return 11.0;
    }
    
    // Interpolation between table entries
    for (int i = 0; i < 11; i++) {
        if (voltage_mV > _uvTable[i].voltageMax_mV && voltage_mV < _uvTable[i+1].voltageMin_mV) {
            float denominator = _uvTable[i+1].voltageMin_mV - _uvTable[i].voltageMax_mV;
            if (denominator == 0) {
                return (float)_uvTable[i].uvIndex;  // Avoid division by zero
            }
            float ratio = (voltage_mV - _uvTable[i].voltageMax_mV) / denominator;
            return (float)_uvTable[i].uvIndex + ratio;
        }
    }
    
    return 0.0;
}

bool GUVASensor::update() {
    if (!_isValid) {
        if (!begin()) {
            _uvIndex = NAN;
            return false;
        }
    }

    // Get voltage reading using burst sampling
    float voltage = readVoltageWithBurstSampling();

    if (isnan(voltage)) {
        _uvIndex = NAN;
        _isValid = false;
        return false;
    }

    _uvIndex = calculateUVIndex(voltage);
    return true;
}