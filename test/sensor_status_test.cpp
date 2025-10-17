#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SHT31.h>
#include "sensors/SensorManager.h"

SensorManager sensorManager;

// Individual sensor objects for detailed readings
Adafruit_BME280 bme;
SHT31 sht;
bool bmeAvailable = false;
bool shtAvailable = false;

void scanI2CDevices() {
    Serial.println("\n========================================");
    Serial.println("          I2C DEVICE SCANNER           ");
    Serial.println("========================================");
    Serial.println("Scanning I2C bus...\n");

    // Initialize I2C if not already initialized
    Wire.begin();
    Wire.setClock(100000); // Set I2C frequency to 100kHz
    delay(100);

    byte error, address;
    int deviceCount = 0;

    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.printf("I2C device found at address 0x%02X  ", address);

            // Identify known devices
            switch(address) {
                case 0x03:
                    Serial.println("(Slave Sensor - Wind/Rain)");
                    break;
                case 0x23:
                    Serial.println("(BH1750 - Light Sensor)");
                    break;
                case 0x36:
                    Serial.println("(AS5600 - Wind Direction)");
                    break;
                case 0x44:
                    Serial.println("(SHT30 - Temp/Humidity)");
                    break;
                case 0x76:
                    Serial.println("(BME280 - Temp/Humidity/Pressure)");
                    break;
                case 0x77:
                    Serial.println("(BMP180 - Temp/Pressure)");
                    break;
                default:
                    Serial.println("(Unknown Device)");
                    break;
            }
            deviceCount++;
        } else if (error == 4) {
            Serial.printf("Unknown error at address 0x%02X\n", address);
        }
    }

    if (deviceCount == 0) {
        Serial.println("No I2C devices found!\n");
    } else {
        Serial.printf("\nTotal I2C devices found: %d\n", deviceCount);
    }
    Serial.println("========================================\n");
    Wire.end();
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("========================================");
    Serial.println("    SENSOR STATUS AND READINGS TEST    ");
    Serial.println("========================================");

    // Run I2C scanner first
    scanI2CDevices();
    delay(500);

    // Initialize sensor manager
    Serial.println("\nInitializing sensors...");
    Serial.println("========================================");

    sensorManager.begin();

    delay(500);

    // Initialize individual sensors for detailed readings
    if (bme.begin(0x76)) {
        bmeAvailable = true;
    }
    if (sht.begin()) {
        shtAvailable = true;
    }

    // Display sensor availability summary
    Serial.println("\n========================================");
    Serial.println("         SENSOR STATUS CHECK           ");
    Serial.println("========================================");

    SensorStatus status = sensorManager.getSensorStatus();

    Serial.printf("BME280 (0x76):       %s\n", status.bmeAvailable ? "✓ Available" : "✗ NOT Available");
    Serial.printf("BMP180 (0x77):       %s\n", status.bmpAvailable ? "✓ Available" : "✗ NOT Available");
    Serial.printf("SHT30 (0x44):        %s\n", status.shtAvailable ? "✓ Available" : "✗ NOT Available");
    Serial.printf("BH1750 (0x23):       %s\n", status.lightAvailable ? "✓ Available" : "✗ NOT Available");
    Serial.printf("GUVA-S12SD (Pin 32): %s\n", status.uvAvailable ? "✓ Available" : "✗ NOT Available");
    Serial.printf("AS5600 (0x36):       %s\n", status.directionAvailable ? "✓ Available" : "✗ NOT Available");
    Serial.printf("Slave (0x03):        %s\n", status.slaveAvailable ? "✓ Available" : "✗ NOT Available");

    Serial.println("========================================");
    delay(1000);
}

void loop() {

    Serial.println("\n========================================");
    Serial.println("         SENSOR READINGS               ");
    Serial.println("========================================");

    // Read all sensors
    SensorReadings readings = sensorManager.readAllSensors();

    Serial.println("\n--- Individual Sensor Readings ---");

    // BME280 readings
    if (bmeAvailable) {
        float bmeTemp = bme.readTemperature();
        float bmeHumid = bme.readHumidity();
        float bmePress = bme.readPressure() / 100.0F;
        Serial.println("BME280:");
        Serial.printf("  Temperature:  %.2f °C\n", bmeTemp);
        Serial.printf("  Humidity:     %.2f %%\n", bmeHumid);
        Serial.printf("  Pressure:     %.2f hPa\n", bmePress);
    } else {
        Serial.println("BME280:         N/A");
    }

    // SHT30 readings
    if (shtAvailable) {
        sht.read();
        float shtTemp = sht.getTemperature();
        float shtHumid = sht.getHumidity();
        Serial.println("SHT30:");
        Serial.printf("  Temperature:  %.2f °C\n", shtTemp);
        Serial.printf("  Humidity:     %.2f %%\n", shtHumid);
    } else {
        Serial.println("SHT30:          N/A");
    }

    Serial.println("\n--- Average Environmental Data ---");
    if (!isnan(readings.temperature)) {
        Serial.printf("Avg Temperature:  %.2f °C\n", readings.temperature);
    } else {
        Serial.println("Avg Temperature:  N/A");
    }

    if (!isnan(readings.humidity)) {
        Serial.printf("Avg Humidity:     %.2f %%\n", readings.humidity);
    } else {
        Serial.println("Avg Humidity:     N/A");
    }

    if (!isnan(readings.pressure)) {
        Serial.printf("Avg Pressure:     %.2f hPa\n", readings.pressure);
    } else {
        Serial.println("Avg Pressure:     N/A");
    }

    if (!isnan(readings.panelTemperature)) {
        Serial.printf("Panel Temp:       %.2f °C\n", readings.panelTemperature);
    } else {
        Serial.println("Panel Temp:       N/A");
    }

    Serial.println("\n--- Light & UV Data ---");
    if (!isnan(readings.lightIntensity)) {
        Serial.printf("Light Intensity:  %.2f lux\n", readings.lightIntensity);
    } else {
        Serial.println("Light Intensity:  N/A");
    }

    if (!isnan(readings.uvIndex)) {
        Serial.printf("UV Index:         %.0f\n", readings.uvIndex);
    } else {
        Serial.println("UV Index:         N/A");
    }

    Serial.println("\n--- Wind & Precipitation Data ---");
    if (!isnan(readings.windDirection)) {
        Serial.printf("Wind Direction:   %.2f °\n", readings.windDirection);
    } else {
        Serial.println("Wind Direction:   N/A");
    }

    if (!isnan(readings.windSpeed)) {
        Serial.printf("Wind Speed:       %.2f km/h\n", readings.windSpeed);
    } else {
        Serial.println("Wind Speed:       N/A");
    }

    if (!isnan(readings.precipitation)) {
        Serial.printf("Precipitation:    %.2f mm\n", readings.precipitation);
    } else {
        Serial.println("Precipitation:    N/A");
    }

    Serial.println("\n========================================");
    Serial.println("Waiting 10 seconds before next reading...");
    Serial.println("========================================\n");

    delay(10000); // Wait 10 seconds before next reading
}
