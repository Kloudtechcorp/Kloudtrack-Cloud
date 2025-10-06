#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

// AS5600 Sensor Configurations
#define AS5600_ADDRESS 0x36
#define AS5600_ANGLE_REG 0x0E
#define AS5600_BURST_SAMPLES 10
#define AS5600_SAMPLE_INTERVAL 50

class AS5600Sensor {
    private:
        Preferences _preferences;
        float _northOffset;
        bool _isCalibrated;
        bool _isDirectionReversed;
        float _windDirection;
        bool _isValid;

        float readAngle();
        float applyCalibration(float rawAngle);
        float calculateCircularAverage(const float *angles, int count);
        void loadCalibration();
        void saveCalibration();
        float calculateWindDirection();

    public:
        AS5600Sensor();

        bool begin();
        bool update();

        void calibrateNorth();
        void reverseDirection();

        float getWindDirection() const { return _windDirection; }
        bool isValid() const { return _isValid; }
        bool isCalibrated() const { return _isCalibrated; }
};