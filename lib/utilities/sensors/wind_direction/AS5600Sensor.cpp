#include "AS5600Sensor.h"

AS5600Sensor::AS5600Sensor() : 
    _windDirection(NAN), 
    _northOffset(0.0), 
    _isCalibrated(false), 
    _isDirectionReversed(false), 
    _isValid(false) 
{}

bool AS5600Sensor::begin() {
    loadCalibration();

    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(AS5600_ANGLE_REG);
    uint8_t error = Wire.endTransmission();

    if (error != 0) {
        _isValid = false;
        return false;
    }

    _isValid = true;
    return true;
}

bool AS5600Sensor::update() {
    if (!_isValid) {
        if (!begin()) {
            _windDirection = NAN;
            return false;
        }
    }

    _windDirection = calculateWindDirection();
    return true;
}

float AS5600Sensor::readAngle() {
    Wire.requestFrom(AS5600_ADDRESS, 2);
    
    byte highByte = Wire.read();
    byte lowByte = Wire.read();

    uint16_t rawValue = ((uint16_t)highByte << 8) | lowByte;
    float degrees = (rawValue * 360.0) / 4096.0;

    return degrees;
}

float AS5600Sensor::applyCalibration(float rawAngle) {
    float calibratedAngle = rawAngle;

    if (_isDirectionReversed) {
        calibratedAngle = 360.0 - calibratedAngle;
    }

    calibratedAngle -= _northOffset;

    while (calibratedAngle < 0) calibratedAngle += 360.0;
    while (calibratedAngle >= 360.0) calibratedAngle -= 360.0;

    return calibratedAngle;
}

float AS5600Sensor::calculateCircularAverage(const float *angles, int count) {
    float sumX = 0, sumY = 0;
    
    for (int i = 0; i < count; i++) {
        float radians = angles[i] * PI / 180.0;
        sumX += cos(radians);
        sumY += sin(radians);
    }

    float avgX = sumX / count;
    float avgY = sumY / count;
    float avgAngle = atan2(avgY, avgX) * 180.0 / PI;

    while (avgAngle < 0) avgAngle += 360.0;
    while (avgAngle >= 360.0) avgAngle -= 360.0;

    return avgAngle;
}

void AS5600Sensor::loadCalibration() {
    _preferences.begin("windvane", false);
    _isCalibrated = _preferences.getBool("calibrated", false);
    _northOffset = _preferences.getFloat("northOffset", 0.0);
    _isDirectionReversed = _preferences.getBool("reversed", false);
    _preferences.end();
}

void AS5600Sensor::saveCalibration() {
    _preferences.begin("windvane", false);
    _preferences.putBool("calibrated", _isCalibrated);
    _preferences.putFloat("northOffset", _northOffset);
    _preferences.putBool("reversed", _isDirectionReversed);
    _preferences.end();
}

float AS5600Sensor::calculateWindDirection() {
    float readings[AS5600_BURST_SAMPLES];
    int validReadings = 0;

    for (int i = 0; i <AS5600_BURST_SAMPLES; i++) {
        float rawAngle = readAngle();

        if (rawAngle >= 0 && validReadings < AS5600_BURST_SAMPLES) {
            readings[validReadings] = applyCalibration(rawAngle);
            validReadings++;
        }
        delay(AS5600_SAMPLE_INTERVAL);
    }
    if (validReadings < 5) {
        return NAN;
    }
    return calculateCircularAverage(readings, validReadings);
}

void AS5600Sensor::calibrateNorth() {
    while (Serial.available()) Serial.read();

    unsigned long startTime = millis();
    while (!Serial.available()) {
        if (millis() - startTime > 30000) {
            return;
        }
        delay(100);
    }

    while (Serial.available()) Serial.read();

    float totalAngle = 0;
    int validReadings = 0;;

    Serial.println("Calibrating...");
    for (int i = 0; i < 20; i++) {
        float angle = readAngle();
        if (angle >= 0) {
            totalAngle += angle;
            validReadings++;
        }
        delay(50);
    }

    if (validReadings > 10) {
        _northOffset = totalAngle / validReadings;
        _isCalibrated = true;
        
        saveCalibration();
        
        Serial.println("North calibrated and saved!");
        Serial.print("North offset: ");
        Serial.print(_northOffset, 2);
        Serial.println("°");
    } else {
        Serial.println("Calibration failed - try again");
    }
}

void AS5600Sensor::reverseDirection() {
    _isDirectionReversed = !_isDirectionReversed;
    saveCalibration();
    Serial.print("Direction reversed: ");
    Serial.println(_isDirectionReversed ? "YES" : "NO");
}