#include "BMP180Sensor.h"

BMP180Sensor::BMP180Sensor() : _temperature(NAN), _pressure(NAN), _isValid(false) {
}

bool BMP180Sensor::begin() {
    if (!_bmp.begin()) {
        _isValid = false;
        return false;
    }
    
    _isValid = true;
    return true;
}

bool BMP180Sensor::update() {
    if (!_isValid) {
        if (!begin()) {
            _temperature = NAN;
            _pressure = NAN;
            return false;
        }
    }
    
    _temperature = _bmp.readTemperature();
    _pressure = _bmp.readPressure() / 100.0F;
    
    return true;
}