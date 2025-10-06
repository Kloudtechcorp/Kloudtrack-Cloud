#include "SlaveSensor.h"

SlaveSensor::SlaveSensor() : _windSpeed(NAN), _precipitation(NAN), _isValid(false) {
    _circumference = 2 * PI * anemometerRadius;
}

bool SlaveSensor::begin() {
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write(0x00); // Dummy write to check if the device is present
    uint8_t error = Wire.endTransmission();

    if (error != 0) {
        _isValid = false;
        return false;
    }

    _isValid = true;
    return true;
}

bool SlaveSensor::update() {
    if (!_isValid) {
        if (!begin()) {
            _windSpeed = NAN;
            _precipitation = NAN;
            return false;
        }
    }

    Wire.requestFrom(SLAVE_ADDRESS, 4);

    byte precipitationMsb = Wire.read();
    byte precipitationLsb = Wire.read();
    byte windSpeedMsb = Wire.read();
    byte windSpeedLsb = Wire.read();

    uint16_t precipitationCount = ((uint16_t)precipitationMsb << 8) | precipitationLsb;
    uint16_t windSpeedCount = ((uint16_t)windSpeedMsb << 8) | windSpeedLsb;

    _precipitation = (float)precipitationCount * rainGaugeTipValue;
    _windSpeed = (_circumference * float(windSpeedCount * 3.6) / period);

    return true;
}