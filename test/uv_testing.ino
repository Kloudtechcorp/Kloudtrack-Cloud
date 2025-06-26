// GUVA-S12SD UV Sensor Test Code for ESP32
// Connect sensor analog out to GPIO36 (A0), VCC to 3.3V, GND to GND

const int UV_SENSOR_PIN = 32;        // GPIO36 (A0) - ADC1_CH0
const float REFERENCE_VOLTAGE = 3.3;  // ESP32 reference voltage
const int ADC_RESOLUTION = 4096;      // 12-bit ADC (0-4095)

void setup() {
  Serial.begin(115200);
  
  // Configure ADC
  analogReadResolution(12);           // Set 12-bit resolution
  analogSetAttenuation(ADC_11db);     // Set input range to 3.3V
  
  Serial.println("GUVA-S12SD UV Sensor Test - ESP32");
  Serial.println("===================================");
  Serial.println("Raw ADC | Voltage | UV Index | Status");
  Serial.println("--------|---------|----------|--------");
  
  // Allow sensor to stabilize
  delay(1000);
}

void loop() {
  // Read analog value (multiple readings for better accuracy)
  int sensorValue = 0;
  for(int i = 0; i < 10; i++) {
    sensorValue += analogRead(UV_SENSOR_PIN);
    delay(10);
  }
  sensorValue /= 10; // Average
  
  // Convert to voltage
  float voltage = (sensorValue * REFERENCE_VOLTAGE) / ADC_RESOLUTION;
  
  // Convert voltage to UV Index (GUVA-S12SD: ~0.1V per UV index)
  float uvIndex = voltage / 0.1;
  
  // Determine status
  String status = getUVStatus(uvIndex);
  String working = isWorking(voltage, sensorValue) ? "WORKING" : "CHECK SENSOR";
  
  // Display readings
  Serial.printf("%7d | %6.3fV | %8.1f | %s (%s)\n", 
                sensorValue, voltage, uvIndex, status.c_str(), working.c_str());
  
  delay(1000);
}

String getUVStatus(float uvIndex) {
  if (uvIndex < 0.3) return "Disconnected? ";
  else if (uvIndex < 1.0) return "No UV/Dark    ";
  else if (uvIndex < 3) return "Low UV/Indoor ";
  else if (uvIndex < 6) return "Moderate UV   ";
  else if (uvIndex < 8) return "High UV       ";
  else return "Very High UV  ";
}

bool isWorking(float voltage, int rawValue) {
  // ESP32 ADC characteristics with pull-down resistor
  // With 10kΩ pull-down: disconnected sensor reads ~0V
  // Connected sensor should read > 0.3V even in darkness
  
  if (rawValue < 150) {  // ~0.12V - likely disconnected with pull-down
    Serial.println("WARNING: Very low reading - sensor may be disconnected");
    return false;
  }
  if (rawValue > 4000) {
    Serial.println("WARNING: Very high ADC reading - check circuit");
    return false;
  }
  if (voltage < 0.1 || voltage > 3.25) {
    return false;
  }
  
  return true;
}

// Enhanced diagnostics function for ESP32
void runAdvancedDiagnostics() {
  Serial.println("\n=== ESP32 UV SENSOR DIAGNOSTICS ===");
  
  // Test different ADC settings
  Serial.println("Testing ADC configurations...");
  
  // Test with different attenuations
  int attenuations[] = {ADC_0db, ADC_2_5db, ADC_6db, ADC_11db};
  String attNames[] = {"0dB (0-1.1V)", "2.5dB (0-1.5V)", "6dB (0-2.2V)", "11dB (0-3.3V)"};
  
  for(int i = 0; i < 4; i++) {
    analogSetAttenuation(attenuations[i]);
    delay(100);
    
    int reading = analogRead(UV_SENSOR_PIN);
    float voltage = 0;
    
    // Calculate voltage based on attenuation
    switch(attenuations[i]) {
      case ADC_0db: voltage = (reading * 1.1) / 4096; break;
      case ADC_2_5db: voltage = (reading * 1.5) / 4096; break;
      case ADC_6db: voltage = (reading * 2.2) / 4096; break;
      case ADC_11db: voltage = (reading * 3.3) / 4096; break;
    }
    
    Serial.printf("%s: Raw=%d, Voltage=%.3fV\n", attNames[i].c_str(), reading, voltage);
  }
  
  // Reset to 11dB for normal operation
  analogSetAttenuation(ADC_11db);
  
  // Stability test
  Serial.println("\nStability test (10 readings):");
  float readings[10];
  float sum = 0;
  
  for (int i = 0; i < 10; i++) {
    int rawValue = analogRead(UV_SENSOR_PIN);
    readings[i] = (rawValue * REFERENCE_VOLTAGE) / ADC_RESOLUTION;
    sum += readings[i];
    Serial.printf("Reading %d: %.3fV\n", i+1, readings[i]);
    delay(200);
  }
  
  float average = sum / 10;
  float variance = 0;
  
  for (int i = 0; i < 10; i++) {
    variance += pow(readings[i] - average, 2);
  }
  variance /= 10;
  float stdDev = sqrt(variance);
  
  Serial.printf("\nStatistics:\n");
  Serial.printf("Average: %.3fV\n", average);
  Serial.printf("Std Dev: %.4fV ", stdDev);
  
  if (stdDev < 0.01) {
    Serial.println("(VERY STABLE)");
  } else if (stdDev < 0.05) {
    Serial.println("(STABLE)");
  } else {
    Serial.println("(UNSTABLE - check connections/power)");
  }
  
  // Final assessment
  Serial.println("\n=== FINAL ASSESSMENT ===");
  if (average > 0.1 && average < 3.2 && stdDev < 0.05) {
    Serial.println("✓ SENSOR APPEARS TO BE WORKING CORRECTLY");
    Serial.println("Try moving sensor between indoor/outdoor light to test response");
  } else {
    Serial.println("✗ POTENTIAL ISSUES DETECTED");
    if (average <= 0.1) Serial.println("  - Very low voltage: Check VCC connection");
    if (average >= 3.2) Serial.println("  - Very high voltage: Check for short circuit");
    if (stdDev >= 0.05) Serial.println("  - Unstable readings: Check all connections");
  }
  
  Serial.println("================================\n");
}

// Call this function once to run full diagnostics
void setup_with_diagnostics() {
  setup();
  delay(2000);
  runAdvancedDiagnostics();
}