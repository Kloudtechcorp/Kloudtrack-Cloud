#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP085.h>
#include <SHT31.h>
#include <BH1750.h>
#include <Preferences.h>
#include <stdint.h>
#define UV_PIN 32
#define SLAVE_ADDRESS 0x03
#define AS5600_ADDRESS 0x36
#define AS5600_ANGLE_REG 0x0E

struct SensorReadings {
    float temperature;
    float humidity;
    float pressure;
    float lightIntensity;
    float uvIndex;
    float windDirection;
    float windSpeed;
    float precipitation;
};

struct SensorStatus {
    bool bmeAvailable = false;
    bool bmpAvailable = false;
    bool shtAvailable = false;
    bool lightAvailable = false;
    bool uvAvailable = false;
    bool directionAvailable = false;
    bool slaveAvailable = false;
};

class SensorManager {
    public: 
        SensorManager();
        void begin();
        SensorStatus getSensorStatus();
        SensorReadings readAllSensors();
    private:
        Adafruit_BME280 bme;
        Adafruit_BMP085 bmp;
        SHT31 sht;
        BH1750 light;
        Preferences preferences;

        // Sensor State
        bool bmeReady = false;
        bool bmpReady = false;
        bool shtReady = false;
        bool lightReady = false;
        bool uvReady = false;
        bool directionReady = false;
        bool slaveReady = false;

        // BME Parameters
        float bmeTemperature = NAN;
        float bmeHumidity = NAN;
        float bmePressure = NAN;

        // BMP Parameters
        float bmpTemperature = NAN;
        float bmpPressure = NAN;

        // SHT Parameters
        float shtTemperature = NAN;
        float shtHumidity = NAN;

        // Average for Temperature, Humidity, & Pressure
        float averageTemperature = NAN;
        float averageHumidity = NAN;
        float averagePressure = NAN;

        // GUVA Parameters
        float uvIntensity = NAN;
        int guvaUv = -1;

        // Sampling Configuration for Wind Direction
        #define BURST_SAMPLES 30
        #define SAMPLE_INTERVAL 100

        // AS5600 Parameters
        float northOffset = 0.0;
        bool isCalibrated = false;
        bool reverseDirection = false;
        float avgAngle = 0.0;
        float degrees = 0.0;
        float calibratedAngle = 0.0;
        float windDirection = NAN;

        // Slave Parameters
        const float tipValue = 0.1099f;
        float precipitation = NAN;
        uint16_t rainCount = 0;
        float windSpeed = NAN;
        float circumference;
        const float calibrationFactor = 2.4845;
        const float radius = 0.05f;
        const int period = 60;
        uint16_t windCount = 0;

        // Functions to read temperature, humidity, pressure and calculate averages
        float readBmeTemperature();
        float readBmeHumidity();
        float readBmePressure();
        float readBmpTemperature();
        float readBmpPressure();
        float readShtTemperature();
        float readShtHumidity();
        void updateSensorReadings();
        void calculateAverages();

        // Function to read light intensity and UV index
        float readLightIntensity();
        float readUvIntensity();
        int calculateUvIndex();

        // Function to read wind direction
        float calculateCircularAverage(float* angles, int count);
        void loadCalibration();
        void saveCalibration();
        float readAngle();
        float applyWindvaneCalibration(float rawAngle);
        void calibrateNorth();
        float performWindMeasurement();

        // Function to update slave readings
        void updateSlaveReadings();
};

#endif