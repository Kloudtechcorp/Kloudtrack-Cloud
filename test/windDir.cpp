#include <Wire.h>
#include <Preferences.h>

// AS5600 I2C address
#define AS5600_ADDR 0x36

// AS5600 register addresses
#define AS5600_ANGLE_REG 0x0E        // Processed angle register (12-bit)

// Preferences object
Preferences preferences;

// Windvane calibration variables
float northOffset = 0.0;        // Offset to align with true North
bool isCalibrated = false;
bool reverseDirection = false;  // Some setups need direction reversed

// Sampling configuration
#define BURST_SAMPLES 30        // Number of readings in each burst
#define SAMPLE_INTERVAL 100     // ms between readings in burst (10Hz during burst)
#define MINUTE_INTERVAL 60000   // 1 minute between measurements

// Timing variables
unsigned long lastMinuteMeasurement = 0;

float calculateCircularAverage(float* angles, int count) {
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
  float avgAngle = atan2(avgY, avgX) * 180.0 / PI;
  
  // Normalize to 0-360 range
  while (avgAngle < 0) avgAngle += 360.0;
  while (avgAngle >= 360.0) avgAngle -= 360.0;
  
  return avgAngle;
}

void loadCalibration() {
  preferences.begin("windvane", false);
  
  isCalibrated = preferences.getBool("calibrated", false);
  northOffset = preferences.getFloat("northOffset", 0.0);
  reverseDirection = preferences.getBool("reversed", false);
  
  preferences.end();
}

void saveCalibration() {
  preferences.begin("windvane", false);
  
  preferences.putBool("calibrated", isCalibrated);
  preferences.putFloat("northOffset", northOffset);
  preferences.putBool("reversed", reverseDirection);
  
  preferences.end();
}

float readAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_ANGLE_REG);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    return -1;
  }
  
  Wire.requestFrom(AS5600_ADDR, 2);
  
  if (Wire.available() >= 2) {
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    
    uint16_t rawValue = ((uint16_t)highByte << 8) | lowByte;
    float degrees = (rawValue * 360.0) / 4096.0;
    
    return degrees;
  }
  
  return -1;
}

float applyWindvaneCalibration(float rawAngle) {
  float calibratedAngle = rawAngle;
  
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

void calibrateNorth() {
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

void performWindMeasurement() {
  float readings[BURST_SAMPLES];
  int validReadings = 0;
  
  Serial.print("Sampling");
  
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
    return;
  }
  
  // Calculate statistics
  float average = calculateCircularAverage(readings, validReadings);
  
  // Output human-readable summary
  Serial.print("Wind Direction: ");
  Serial.print(average, 1);
  Serial.print("°");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // Explicitly set SDA=21, SCL=22
  Wire.setClock(400000); // Set I2C clock to 400kHz
  
  // Load calibration from preferences
  loadCalibration();
  
  delay(1000);
  
  lastMinuteMeasurement = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
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
      lastMinuteMeasurement = currentTime; // Reset timer
    }
  }
  
  // Take measurement every minute
  if (currentTime - lastMinuteMeasurement >= MINUTE_INTERVAL) {
    performWindMeasurement();
    lastMinuteMeasurement = currentTime;
  }
  
  delay(100); // Small delay to prevent overwhelming the system
}
