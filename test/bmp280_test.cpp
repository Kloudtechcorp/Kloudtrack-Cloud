#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp280;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("========================================");
    Serial.println("       BMP280 SENSOR TEST              ");
    Serial.println("========================================");

    // Initialize I2C
    Wire.begin();
    Wire.setClock(100000); // 100kHz
    delay(100);

    // Try to initialize BMP280 at address 0x76
    Serial.println("\nInitializing BMP280 at address 0x76...");

    if (!bmp280.begin(0x76)) {
        Serial.println("❌ BMP280 initialization FAILED at 0x76");

        // Try default address 0x77
        Serial.println("Trying default address 0x77...");
        if (!bmp280.begin()) {
            Serial.println("❌ BMP280 initialization FAILED at 0x77");
            Serial.println("\nCould not find a valid BMP280 sensor!");
            Serial.println("Check wiring or try a different address.");
        } else {
            Serial.println("✓ BMP280 initialized successfully at 0x77");
        }
    } else {
        Serial.println("✓ BMP280 initialized successfully at 0x76");

        // Configure sensor settings
        bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode
                          Adafruit_BMP280::SAMPLING_X2,      // Temp. oversampling
                          Adafruit_BMP280::SAMPLING_X16,     // Pressure oversampling
                          Adafruit_BMP280::FILTER_X16,       // Filtering
                          Adafruit_BMP280::STANDBY_MS_500);  // Standby time

        Serial.println("\nBMP280 Configuration:");
        Serial.println("- Mode: Normal");
        Serial.println("- Temperature oversampling: x2");
        Serial.println("- Pressure oversampling: x16");
        Serial.println("- Filter: x16");
        Serial.println("- Standby time: 500ms");
    }

    Serial.println("\n========================================");
    delay(1000);
}

void loop() {
    Serial.println("\n========================================");
    Serial.println("       BMP280 READINGS                 ");
    Serial.println("========================================");

    // Read temperature
    float temperature = bmp280.readTemperature();
    if (!isnan(temperature)) {
        Serial.printf("Temperature:  %.2f °C\n", temperature);
    } else {
        Serial.println("Temperature:  ERROR (NaN)");
    }

    // Read pressure
    float pressure = bmp280.readPressure() / 100.0F; // Convert Pa to hPa
    if (!isnan(pressure)) {
        Serial.printf("Pressure:     %.2f hPa\n", pressure);
    } else {
        Serial.println("Pressure:     ERROR (NaN)");
    }

    // Calculate altitude (assuming sea level pressure = 1013.25 hPa)
    float altitude = bmp280.readAltitude(1013.25);
    if (!isnan(altitude)) {
        Serial.printf("Altitude:     %.2f m (approx.)\n", altitude);
    } else {
        Serial.println("Altitude:     ERROR (NaN)");
    }

    Serial.println("========================================");
    Serial.println("Waiting 5 seconds before next reading...\n");

    delay(5000); // Wait 5 seconds before next reading
}
