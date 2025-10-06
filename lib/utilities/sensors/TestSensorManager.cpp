#include "TestSensorManager.h"

TestSensorManager::TestSensorManager() {
    // Initialize pointers to nullptr - actual initialization happens in begin()
    _bmeSensor = nullptr;
    _bmpSensor = nullptr;
    _shtSensor = nullptr;
    _lightSensor = nullptr;
    _asSensor = nullptr;
    _slaveSensor = nullptr;
    _guvaSensor = nullptr;
}

void TestSensorManager::begin() {
    SerialMon.println("TestSensorManager: Initializing sensors...");

    _bmeSensor = new BME280Sensor();
    _bmpSensor = new BMP180Sensor();
    _shtSensor = new SHT30Sensor();
    _lightSensor = new BH1750Sensor();
    _asSensor = new AS5600Sensor();
    _slaveSensor = new SlaveSensor();
    _guvaSensor = new GUVASensor();

    // Check for allocation failures (rare on ESP32 but good practice)
    if (!_bmeSensor || !_bmpSensor || !_shtSensor || !_lightSensor || !_guvaSensor || !_asSensor || !_slaveSensor) {
        SerialMon.println("CRITICAL: Memory allocation failed for sensors!");
        return;
    }

    // Initialize I2C bus
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQ);
    delay(100); // Allow I2C bus to stabilize

    // Scan for I2C devices to diagnose multiplexer issue
    SerialMon.println("Scanning I2C bus for devices...");
    int deviceCount = 0;
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0) {
            SerialMon.print("I2C device found at address 0x");
            if (address < 16) SerialMon.print("0");
            SerialMon.print(address, HEX);
            if (address == 0x76) SerialMon.print(" (BME280)");
            if (address == 0x23) SerialMon.print(" (BH1750)");
            if (address == 0x36) SerialMon.print(" (AS5600)");
            if (address == 0x03) SerialMon.print(" (Slave Sensor)");
            if (address == 0x77) SerialMon.print(" (BMP180)");
            if (address == 0x44) SerialMon.print(" (SHT30)");
            SerialMon.println();
            deviceCount++;
        }
    }
    SerialMon.print("Found ");
    SerialMon.print(deviceCount);
    SerialMon.println(" I2C devices total");

    // Initialize sensors with error checking
    if (_bmeSensor && !_bmeSensor->begin()) {
        SerialMon.println("Warning: BME280 sensor 1 failed to initialize");
    }
    if (_bmpSensor && !_bmpSensor->begin()) {
        SerialMon.println("Warning: BMP180 sensor failed to initialize");
    }
    if (_shtSensor && !_shtSensor->begin()) {
        SerialMon.println("Warning: SHT30 sensor failed to initialize");
    }
    if (!_lightSensor->begin()) {
        SerialMon.println("Warning: BH1750 light sensor failed to initialize");
    }
    if (!_guvaSensor->begin()) {
        SerialMon.println("Warning: GUVA-S12SD sensor failed to initialize");
    }
    if (!_asSensor->begin()) {
        SerialMon.println("Warning: AS5600 sensor failed to initialize");
    }
    if (!_slaveSensor->begin()) {
        SerialMon.println("Warning: Slave sensor failed to initialize");
    }

    SerialMon.println("TestSensorManager: Initialization complete!");
}

TestSensorManager::~TestSensorManager()
{
    // Safe to delete nullptr in C++
    delete _bmeSensor;
    delete _bmpSensor;
    delete _shtSensor;
    delete _lightSensor;
    delete _guvaSensor;
    delete _asSensor;
    delete _slaveSensor;
}

void TestSensorManager::updateTemperatureHumidityPressure() {
    SerialMon.print("BME: \t\t");
    if (_bmeSensor->update()) SerialMon.println("OK");
    else SerialMon.println("Failed");
    _bmeTemperature = _bmeSensor->getTemperature();
    _bmeHumidity = _bmeSensor->getHumidity();
    _bmePressure = _bmeSensor->getPressure();
    delay(10);

    SerialMon.print("BMP: \t\t");
    if (_bmpSensor->update()) SerialMon.println("OK");
    else SerialMon.println("Failed");
    _bmpTemperature = _bmpSensor->getTemperature();
    _bmpPressure = _bmpSensor->getPressure();
    delay(10);

    SerialMon.print("SHT: \t\t");
    if (_shtSensor->update()) SerialMon.println("OK");
    else SerialMon.println("Failed");
    _shtTemperature = _shtSensor->getTemperature();
    _shtHumidity = _shtSensor->getHumidity();
    delay(10);

    // Average Temperature
    float tempSum = 0;
    int validTempCount = 0;

    if (!isnan(_bmeTemperature)) {
        tempSum += _bmeTemperature;
        validTempCount++;
    }
    if (!isnan(_shtTemperature)) {
        tempSum += _shtTemperature;
        validTempCount++;
    }

    _averageTemperature = (validTempCount > 0) ? tempSum / validTempCount : NAN;

    // Average Humidity
    float humiditySum = 0;
    int validHumidityCount = 0;

    if (!isnan(_bmeHumidity)) {
        humiditySum += _bmeHumidity;
        validHumidityCount++;
    }
    if (!isnan(_shtHumidity)) {
        humiditySum += _shtHumidity;
        validHumidityCount++;
    }

    _averageHumidity = (validHumidityCount > 0) ? humiditySum / validHumidityCount : NAN;

    // Average Pressure
    float pressureSum = 0;
    int validPressureCount = 0;

    if (!isnan(_bmePressure)) {
        pressureSum += _bmePressure;
        validPressureCount++;
    }
    if (!isnan(_bmpPressure)) {
        pressureSum += _bmpPressure;
        validPressureCount++;
    }

    _averagePressure = (validPressureCount > 0) ? pressureSum / validPressureCount : NAN;
}

void TestSensorManager::updateLightIntensity() {
    SerialMon.print("BH1750: \t");
    if (_lightSensor->update()) SerialMon.println("OK");
    else SerialMon.println("Failed");
    _lux = _lightSensor->getLux();
    delay(10);
}

void TestSensorManager::updateUvIndex() {
    SerialMon.print("GUVA-S12SD: \t");
    if (_guvaSensor->update()) SerialMon.println("OK");
    else SerialMon.println("Failed");
    _uvIndex = _guvaSensor->getUVIndex();
    delay(10);
}

void TestSensorManager::updateWindDirection() {
    SerialMon.print("AS5600: \t");
    if (_asSensor->update()) SerialMon.println("OK");
    else SerialMon.println("Failed");
    _windDirection= _asSensor->getWindDirection();
    delay(10);
}

void TestSensorManager::updateSlaveSensor() {
    SerialMon.print("Slave: \t\t");
    if (_slaveSensor->update()) SerialMon.println("OK");
    else SerialMon.println("Failed");
    _windSpeed = _slaveSensor->getWindSpeed();
    _precipitation = _slaveSensor->getPrecipitation();
    delay(10);
}

SensorStatus TestSensorManager::getSensorStatus() {
    SensorStatus status;

    status.bmeAvailable = _bmeSensor->isValid();
    status.bmpAvailable = _bmpSensor->isValid();
    status.shtAvailable = _shtSensor->isValid();
    status.lightAvailable = _lightSensor->isValid();
    status.guvaAvailable = _guvaSensor->isValid();
    status.asAvailable = _asSensor->isValid();
    status.slaveAvailable = _slaveSensor->isValid();

    return status;
};