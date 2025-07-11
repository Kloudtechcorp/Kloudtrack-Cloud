#include "SensorManager.h"

SensorManager::SensorManager()
    : light(0x23) {}

void SensorManager::begin() {
    Wire.begin();
    Wire.setClock(400000); // Set I2C frequency to 400kHz

    // Initialize BME280 Sensor
    if (!bme.begin(0x76)) {
        Serial.println("BME280 failed.");
        bmeReady = false;
    } else {
        Serial.println("BME280 ready.");
        bmeReady = true;
    }

    // Initialize BMP180 Sensor
    if (!bmp.begin()) {
        Serial.println("BMP180 failed.");
        bmpReady= false;
    } else {
        Serial.println("BMP180 ready.");
        bmpReady = true;
    }

    // Initialize SHT30 Sensor
    if (!sht.begin()) {
        Serial.println("SHT30 failed.");
        shtReady = false;
    } else {
        Serial.println("SHT30 ready.");
        shtReady = true;
    }

    // Initialize BH1750 Sensor
    if (!light.begin()) {
        Serial.println("BH1750 failed.");
        lightReady = false;
    } else {
        Serial.println("BH1750 ready.");
        lightReady = true;
    }

    // Initialize GUVA-S12SD Sensor
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    int sensorValue = 0;
    for (int i = 0; i < 10; i++) {
        sensorValue += analogRead(UV_PIN);
        delay(10);
    }
    sensorValue /= 10;

    float voltage = (sensorValue * 3.3) / 4095.0;
    if (voltage < 0.1 || voltage > 3.25) {
        Serial.println("GUVA-S12SD failed.");
        uvReady = false;
    } else {
        Serial.println("GUVA-S12SD ready.");
        uvReady = true;
    }

    // Initialize AS5600 Sensor
    loadCalibration();
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(AS5600_ANGLE_REG);
    byte error = Wire.endTransmission();
    if (error != 0) {
        Serial.println("AS5600 failed.");
        directionReady = false;
    } else {
        Serial.println("AS5600 ready.");
        directionReady = true;
    }

    // Initialize Slave Sensor
    Wire.beginTransmission(SLAVE_ADDRESS);
    byte slaveError = Wire.endTransmission();
    if (slaveError != 0) {
        Serial.println("Slave failed.");
        slaveReady = false;
    } else {
        Serial.println("Slave ready.");
        slaveReady = true;
    }

    Serial.println("Sensor initialization complete.");
    Serial.println("---------------------------------");
}

SensorStatus SensorManager::getSensorStatus() {
    SensorStatus status;

    status.bmeAvailable = bmeReady;
    status.bmpAvailable = bmpReady;
    status.shtAvailable = shtReady;
    status.lightAvailable = lightReady;
    status.uvAvailable = uvReady;
    status.directionAvailable = directionReady;
    status.slaveAvailable = slaveReady;

    return status;
}

SensorReadings SensorManager::readAllSensors() {
    // Function to read temperature, humidity, and pressure from BME280, BMP180, and SHT30
    updateSensorReadings();
    calculateAverages();

    // Function to read and calculate UV index
    readUvIntensity();
    calculateUvIndex();

    // Function to read wind direction
    performWindMeasurement();

    // Function to read slave sensor data
    updateSlaveReadings();

    // Prepare the SensorReadings structure
    SensorReadings readings;
    // Assigning the sensor readings to the structure
    readings.temperature = averageTemperature;
    readings.humidity = averageHumidity;
    readings.pressure = averagePressure;
    readings.lightIntensity = readLightIntensity();
    readings.uvIndex = guvaUv;
    readings.windDirection = windDirection;
    readings.windSpeed = windSpeed;
    readings.precipitation = precipitation;
    readings.panelTemperature = readBmpTemperature();
    return readings;
}

void SensorManager::updateSensorReadings() {
    if (bmeReady) {
        bmeTemperature = readBmeTemperature();
        bmeHumidity = readBmeHumidity();
        bmePressure = readBmePressure();
    }
    if (bmpReady) {
        bmpTemperature = readBmpTemperature();
        bmpPressure = readBmpPressure();
    }
    if (shtReady) {
        shtTemperature = readShtTemperature();
        shtHumidity = readShtHumidity();
    }
}

float SensorManager::readBmeTemperature() {
    if (!bmeReady) return NAN;
    float bmeTemperatureCalibrationFactor = 0.0f;
    float t = bme.readTemperature();
    return isnan(t) ? NAN : t;
}

float SensorManager::readBmeHumidity() {
    if (!bmeReady) return NAN;
    float bmeHumidityCalibrationFactor = 0.0f;
    float h = bme.readHumidity();
    return isnan(h) ? NAN : h;
}

float SensorManager::readBmePressure() {
    if (!bmeReady) return NAN;
    float bmePressureCalibrationFactor = 0.0f;
    float p = bme.readPressure() / 100.0F;
    return isnan(p) ? NAN : p;
}

float SensorManager::readBmpTemperature() {
    if (!bmpReady) return NAN;
    float bmpTemperatureCalibrationFactor = 0.0f;
    float t = bmp.readTemperature();
    return isnan(t) ? NAN : t;
}

float SensorManager::readBmpPressure() {
    if (!bmpReady) return NAN;
    float bmpPressureCalibrationFactor = 0.0f;
    float p = bmp.readPressure() / 100.0F;
    return isnan(p) ? NAN : p;
}

float SensorManager::readShtTemperature() {
    if (!shtReady) return NAN;
    uint32_t start, stop;
    start = micros(); sht.read(); stop = micros();
    float shtTemperatureCalibrationFactor = 0.0f;
    float t = sht.getTemperature();
    return isnan(t) ? NAN : t;
}

float SensorManager::readShtHumidity() {
    if (!shtReady) return NAN;
    uint32_t start, stop;
    start = micros(); sht.read(); stop = micros();
    float shtHumidityCalibrationFactor = 0.0f;
    float h = sht.getHumidity();
    return isnan(h) ? NAN : h;
}

float SensorManager::readLightIntensity() {
    if (!lightReady) return NAN;
    float lux = light.readLightLevel();
    return (lux < 0 || isnan(lux)) ? NAN : lux;
}

float SensorManager::readUvIntensity() {
    if (!uvReady) return NAN;
    int sensorValue = 0;
    for (int i = 0; i < 10; i++) {
        sensorValue += analogRead(UV_PIN);
        delay(10);
    }
    sensorValue /= 10;
    float voltage = (sensorValue * 3.3) / 4095.0;
    float uvIntensity = voltage * 1000.0;
    if (sensorValue < 150 || sensorValue > 4000) return NAN;
    return isnan(uvIntensity) ? NAN : uvIntensity;
}

int SensorManager::calculateUvIndex() {
    if (uvIntensity < 50) guvaUv = 0;
    else if (uvIntensity >= 50 && uvIntensity < 227)    guvaUv = 1;
    else if (uvIntensity >= 227 && uvIntensity < 318)   guvaUv = 2;
    else if (uvIntensity >= 318 && uvIntensity < 408)   guvaUv = 3;
    else if (uvIntensity >= 408 && uvIntensity < 503)   guvaUv = 4;
    else if (uvIntensity >= 503 && uvIntensity < 606)   guvaUv = 5;
    else if (uvIntensity >= 606 && uvIntensity < 696)   guvaUv = 6;
    else if (uvIntensity >= 696 && uvIntensity < 795)   guvaUv = 7;
    else if (uvIntensity >= 795 && uvIntensity < 881)   guvaUv = 8;
    else if (uvIntensity >= 881 && uvIntensity < 986)   guvaUv = 9;
    else if (uvIntensity >= 986 && uvIntensity < 1170)  guvaUv = 10;
    else if (uvIntensity >= 1170)                       guvaUv = 11;
    else { guvaUv = -1; }
    return guvaUv;
}

void SensorManager::updateSlaveReadings() {
    Wire.beginTransmission(SLAVE_ADDRESS);
    byte error = Wire.endTransmission();

    if (error != 0) {
        return;
    }

    Wire.requestFrom(SLAVE_ADDRESS, 4);

    // Get rain count and solve for precipitation
    if (Wire.available() >= 2) {
        byte msb = Wire.read();
        byte lsb = Wire.read();
        rainCount = (msb << 8) | lsb;
    }
    precipitation = rainCount * tipValue;

    // Get wind count and solve for wind speed
    if (Wire.available()) {
        byte msb = Wire.read();
        byte lsb = Wire.read();
        windCount = (msb << 8) | lsb;
    }
    circumference = 2 * PI * radius * windSpeedCalibrationFactor;
    windSpeed = ((circumference * windCount * 3.6) / period);
}

void SensorManager::calculateAverages() {
    // --- Temperature ---
    float tempSum = 0;
    int validTempCount = 0;

    if (!isnan(bmeTemperature)) {
        tempSum += bmeTemperature;
        validTempCount++;
    }
    if (!isnan(shtTemperature)) {
        tempSum += shtTemperature;
        validTempCount++;
    }

    averageTemperature = (validTempCount > 0) ? tempSum / validTempCount : NAN;

    // --- Humidity ---
    float humiditySum = 0;
    int validHumidityCount = 0;

    if (!isnan(bmeHumidity)) {
        humiditySum += bmeHumidity;
        validHumidityCount++;
    }
    if (!isnan(shtHumidity)) {
        humiditySum += shtHumidity;
        validHumidityCount++;
    }

    averageHumidity = (validHumidityCount > 0) ? humiditySum / validHumidityCount : NAN;

    // --- Pressure ---
    float pressureSum = 0;
    int validPressureCount = 0;

    if (!isnan(bmePressure)) {
        pressureSum += bmePressure;
        validPressureCount++;
    }
    if (!isnan(bmpPressure)) {
        pressureSum += bmpPressure;
        validPressureCount++;
    }

    averagePressure = (validPressureCount > 0) ? pressureSum / validPressureCount : NAN;

    // Error Handling for Averages
    if (validTempCount == 0) {
        Serial.println("Error: No valid temperature readings.");
    }
    if (validHumidityCount == 0) {
        Serial.println("Error: No valid humidity readings.");
    }
    if (validPressureCount == 0) {
        Serial.println("Error: No valid pressure readings.");
    }
}

float SensorManager::calculateCircularAverage(float* angles, int count) {
  if (count == 0) return 0;
  
  // Convert to unit vectors and average
  float sumX = 0, sumY = 0;
  
  for (int i = 0; i < count; i++) {
    float radians = angles[i] * PI / 180.0;
    sumX += cos(radians);
    sumY += sin(radians);
  }
  
  float avgX = sumX / count;
  float avgY = sumY / count;
  
  // Convert back to degrees
  avgAngle = atan2(avgY, avgX) * 180.0 / PI;
  
  // Normalize to 0-360 range
  while (avgAngle < 0) avgAngle += 360.0;
  while (avgAngle >= 360.0) avgAngle -= 360.0;
  
  return avgAngle;
}

void SensorManager::loadCalibration() {
    preferences.begin("windvane", false); // Open in read-only mode
  
    isCalibrated = preferences.getBool("calibrated", false);
    northOffset = preferences.getFloat("northOffset", 0.0);
    reverseDirection = preferences.getBool("reversed", false);
    
    preferences.end();
}

void SensorManager::saveCalibration() {
    preferences.begin("windvane", false);
  
    preferences.putBool("calibrated", isCalibrated);
    preferences.putFloat("northOffset", northOffset);
    preferences.putBool("reversed", reverseDirection);
    
    preferences.end();
}

float SensorManager::readAngle() {
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(AS5600_ANGLE_REG);
    byte error = Wire.endTransmission();

    if (error != 0) {
        return -1;
    }

    Wire.requestFrom(AS5600_ADDRESS, 2);

    if (Wire.available() >= 2) {
    byte highByte = Wire.read();
    byte lowByte = Wire.read();

    uint16_t rawValue = ((uint16_t)highByte << 8) | lowByte;
    degrees = (rawValue * 360.0) / 4096.0;

    return degrees;
    }

    return -1;
}

float SensorManager::applyWindvaneCalibration(float rawAngle) {
    calibratedAngle = rawAngle;

    // Apply direction reversal if needed
    if (reverseDirection) {
    calibratedAngle = 360.0 - calibratedAngle;
    }

    // Apply north offset
    calibratedAngle = calibratedAngle - northOffset;

    // Normalize to 0-360 range
    while (calibratedAngle < 0) calibratedAngle += 360.0;
    while (calibratedAngle >= 360.0) calibratedAngle -= 360.0;

    return calibratedAngle;
}

void SensorManager::calibrateNorth() {
  Serial.println("Point windvane to NORTH and press Enter...");
  
  // Clear serial buffer
  while (Serial.available()) Serial.read();
  while (!Serial.available()) {
    delay(100);
  }
  while (Serial.available()) Serial.read();
  
  // Take multiple readings for accuracy
  float totalAngle = 0;
  int validReadings = 0;
  
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
    northOffset = totalAngle / validReadings;
    isCalibrated = true;
    
    // Save to preferences
    saveCalibration();
    
    Serial.println("North calibrated and saved!");
    Serial.print("North offset: ");
    Serial.print(northOffset, 2);
    Serial.println("°");
  } else {
    Serial.println("Calibration failed - try again");
  }
}

float SensorManager::performWindMeasurement() {
    // Check for serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'c' || cmd == 'C') {
      calibrateNorth();
    } else if (cmd == 'r' || cmd == 'R') {
      reverseDirection = !reverseDirection;
      saveCalibration();
      Serial.print("Direction reversed: ");
      Serial.println(reverseDirection ? "YES" : "NO");
    } else if (cmd == 't' || cmd == 'T') {
      // Test measurement now
      performWindMeasurement();
    }
  }

  float readings[BURST_SAMPLES];
  int validReadings = 0;
  
  Serial.print("Collecting data");
  
  // Take burst of readings over 3 seconds
  for (int i = 0; i < BURST_SAMPLES; i++) {
    float rawAngle = readAngle();
    
    if (rawAngle >= 0) {
      // Apply calibration immediately
      readings[validReadings] = applyWindvaneCalibration(rawAngle);
      validReadings++;
    }
    
    // Show progress
    if (i % 10 == 0) Serial.print(".");
    
    delay(SAMPLE_INTERVAL);
  }
  
  Serial.println();
  
  if (validReadings < 15) { // Need at least half the samples
    Serial.println("ERROR: Insufficient valid readings");
    return NAN;
  }
  
  // Calculate statistics
  windDirection = calculateCircularAverage(readings, validReadings);
  Serial.println();

  return windDirection;
}