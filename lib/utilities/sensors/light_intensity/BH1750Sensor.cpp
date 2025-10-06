#include "BH1750Sensor.h"

BH1750Sensor::BH1750Sensor() : _light(0x23), _lux(NAN), _isValid(false) {}

bool BH1750Sensor::begin() {
    if (!_light.begin()) {
        _isValid = false;
        return false;
    }

    _isValid = true;
    return true;
}

bool BH1750Sensor::update() {
    if (!_isValid) {
        if (!begin()) {
            _lux = NAN;
            return false;
        }
    }

    _lux = _light.readLightLevel();
    return true;
}