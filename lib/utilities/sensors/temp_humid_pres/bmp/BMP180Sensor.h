#pragma once

#include <Arduino.h>
#include <Adafruit_BMP085.h>

class BMP180Sensor {
private:
    Adafruit_BMP085 _bmp;
    float _temperature;
    float _pressure;
    bool _isValid;
    
public:
    BMP180Sensor();
    
    bool begin();
    bool update();
    
    float getTemperature() const { return _temperature; }
    float getPressure() const { return _pressure; }
    bool isValid() const { return _isValid; }
};