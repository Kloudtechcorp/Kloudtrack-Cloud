#pragma once

#include <Arduino.h>
#include "temp_humid_pres/bme/BME280Sensor.h"
#include "temp_humid_pres/bmp/BMP180Sensor.h"
#include "temp_humid_pres/sht/SHT30Sensor.h"
#include "light_intensity/BH1750Sensor.h"
#include "wind_direction/AS5600Sensor.h"
#include "precipitation_windspeed/SlaveSensor.h"
#include "uv_index/GUVASensor.h"
#include "esp_task_wdt.h"
#define SerialMon Serial
const int SCL_PIN = 22;
const int SDA_PIN = 21;
const int I2C_FREQ = 100000;
const int I2C_TIMEOUT = 20;
const int SERIAL_BAUD = 115200;

struct SensorReadings {
    float temperature;
    float humidity;
    float pressure;
    float lightIntensity;
    float uvIndex;
    float windDirection;
    float windSpeed;
    float precipitation;
    float panelTemperature;
};

struct SensorStatus {
    bool bmeAvailable = false;
    bool bmpAvailable = false;
    bool shtAvailable = false;
    bool lightAvailable = false;
    bool guvaAvailable = false;
    bool asAvailable = false;
    bool slaveAvailable = false;
};

class TestSensorManager {
    private:
        BME280Sensor* _bmeSensor;
        BMP180Sensor* _bmpSensor;
        SHT30Sensor* _shtSensor;
        BH1750Sensor* _lightSensor;
        AS5600Sensor* _asSensor;
        SlaveSensor* _slaveSensor;
        GUVASensor* _guvaSensor;

        // BME Parameters
        float _bmeTemperature = NAN;
        float _bmeHumidity = NAN;
        float _bmePressure = NAN;

        // BMP Parameters
        float _bmpTemperature = NAN;
        float _bmpPressure = NAN;

        // SHT Parameters
        float _shtTemperature = NAN;
        float _shtHumidity = NAN;

        // Average for Temperature, Humidity, & Pressure
        float _averageTemperature = NAN;
        float _averageHumidity = NAN;
        float _averagePressure = NAN;

        // Light Intensity & UV Index Parameters
        float _lux = NAN;
        float _uvIndex = NAN;

        // Wind Direction, Wind Speed & Precipitation Parameters
        float _windDirection = NAN;
        float _windSpeed = NAN;
        float _precipitation = NAN;

        void updateTemperatureHumidityPressure();
        void updateLightIntensity();
        void updateWindDirection();
        void updateSlaveSensor();
        void updateUvIndex();

    public:
        TestSensorManager();
        ~TestSensorManager();
        void begin(); // Initialize sensors (call after Serial and I2C setup)
        SensorStatus getSensorStatus();

        float getAverageTemperature() { return _averageTemperature; }
        float getAverageHumidity() { return _averageHumidity; }
        float getAveragePressure() { return _averagePressure; }
        float getLux() { return _lux; }
        float getUvIndex() { return _uvIndex; }
        float getWindDirection() { return _windDirection; }
        float getWindSpeed() { return _windSpeed; }
        float getPrecipitation() { return _precipitation; }

        void updateAllSensors() {
            SerialMon.println("\n=================================================");
            SerialMon.println("Sensors Status");

            updateTemperatureHumidityPressure(); yield();
            updateLightIntensity(); yield();
            updateWindDirection(); yield();
            updateSlaveSensor(); yield();
            updateUvIndex(); yield();
        }
};