#include "BME280Sensor.h"

BME280Sensor::BME280Sensor() : _temperature(NAN), _humidity(NAN), _pressure(NAN), _isValid(false) {}

bool BME280Sensor::begin() {
    if (!_bme.begin(0x76)) {
        _isValid = false;
        return false;
    }

    _isValid = true;
    return true;
}

bool BME280Sensor::update() {
    if (!_isValid) {
        if (!begin()) {
            _temperature = NAN;
            _humidity = NAN;
            _pressure = NAN;
            return false;
        }
    }

    _temperature = _bme.readTemperature();
    _humidity = _bme.readHumidity();
    _pressure = _bme.readPressure() / 100.0F;

    return true;
}