#include "SHT30Sensor.h"

SHT30Sensor::SHT30Sensor() : _temperature(NAN), _humidity(NAN), _isValid(false) {}

bool SHT30Sensor::begin() {
    if (!_sht.begin()) {
        _isValid = false;
        return false;
    }

    _isValid = true;
    return true;
}

bool SHT30Sensor::update() {
    if (!_isValid) {
        if (!begin()) {
            _temperature = NAN;
            _humidity = NAN;
            return false;
        }
    }

    uint32_t _start, _stop;
    _start = micros(); 
    _sht.read(); 
    _stop = micros();
    _temperature = _sht.getTemperature();
    _humidity = _sht.getHumidity();

    return true;
}