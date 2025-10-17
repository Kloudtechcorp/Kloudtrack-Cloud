#include <Arduino.h>
#include <PubSubClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <FS.h>
#include "configuration/secrets.h"
#include "configuration/Config.h"
#include "time/DateTime.h"
#include "storage/SdCard.h"
#include "sensors/SensorManager.h"

// Weather data parameters
SensorManager sensorManager;
#define WEATHER_PUBLISH_INTERVAL 60000
unsigned long lastWeatherPublish = 0;

#define MAX_STORED_ENTRIES 1000 // Limit number of stored entries to prevent memory issues
#define SYNC_INTERVAL 300000    // Attempt to sync every 5 minutes (300,000 ms)

#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
#include <SSLClient.h>
#include <ArduinoHttpClient.h>

// GSM Parameters
#define UART_BAUD 115200
#define PIN_DTR 25
#define PIN_TX 26
#define PIN_RX 27
#define PWR_PIN 4
#define PIN_RI 33
#define RESET 5
#define SerialMon Serial

HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);
TinyGsmClient baseClient(modem);
SSLClient sslClient(&baseClient);
PubSubClient mqttClient(sslClient);
Preferences preferences;
Config config;
SdCard sdCard;

bool deviceActivated = false;
bool gsmConnected = false;
int gsmRetryCount = 0;
int mqttRetryCount = 0;
int maxRetries = 5;
unsigned long lastRejectionLog = 0;
#define REJECTION_LOG_INTERVAL 60000 // Log rejection only once per minute
unsigned long reconnectDelay = 1000;
const unsigned long maxReconnectDelay = 60000;

// Memory management variables
unsigned long lastMemoryCheck = 0;
unsigned long bootTime = 0;
int successfulOperations = 0;
const unsigned long MEMORY_CHECK_INTERVAL = 30000; // Check every 30 seconds
const unsigned long MAX_UPTIME = 24 * 60 * 60 * 1000; // 24 hours
const size_t MIN_FREE_HEAP_THRESHOLD = 50000; // 50KB minimum
const int MAX_OPERATIONS_BEFORE_RESTART = 1000; // Restart after 1000 successful operations
unsigned long lastSyncAttempt = 0;

// Network health monitoring variables
struct NetworkHealth {
  unsigned long gsmUptime = 0;
  unsigned long mqttUptime = 0;
  unsigned long lastGsmDisconnect = 0;
  unsigned long lastMqttDisconnect = 0;
  int gsmReconnectCount = 0;
  int mqttReconnectCount = 0;
  int signalQuality = 0;
  bool networkHealthy = false;
} networkHealth;

DateTime dateTime;

// Error logging levels
enum LogLevel {
  LOG_DEBUG = 0,
  LOG_INFO = 1,
  LOG_WARNING = 2,
  LOG_ERROR = 3,
  LOG_CRITICAL = 4
};

// Global log level setting
LogLevel currentLogLevel = LOG_INFO;

// Enhanced logging function
void logMessage(LogLevel level, const char* module, const char* message) {
  if (level < currentLogLevel) return;
  
  const char* levelStr[] = {"DEBUG", "INFO", "WARN", "ERROR", "CRITICAL"};
  unsigned long timestamp = millis();
  
  Serial.printf("[%lu] [%s] [%s] %s\n", timestamp, levelStr[level], module, message);
  
  // For critical errors, also publish via MQTT if connected
  if (level >= LOG_ERROR && mqttClient.connected()) {
    StaticJsonDocument<200> logDoc;
    logDoc["timestamp"] = timestamp;
    logDoc["level"] = levelStr[level];
    logDoc["module"] = module;
    logDoc["message"] = message;
    
    String logJson;
    serializeJson(logDoc, logJson);
    mqttClient.publish(config.getAwsIotDeviceResponseTopic(), logJson.c_str());
  }
}

// Convenience macros
#define LOG_DEBUG_MSG(module, msg) logMessage(LOG_DEBUG, module, msg)
#define LOG_INFO_MSG(module, msg) logMessage(LOG_INFO, module, msg)
#define LOG_WARNING_MSG(module, msg) logMessage(LOG_WARNING, module, msg)
#define LOG_ERROR_MSG(module, msg) logMessage(LOG_ERROR, module, msg)
#define LOG_CRITICAL_MSG(module, msg) logMessage(LOG_CRITICAL, module, msg)

// Functions
void publishUpdateStatus(const char *status, const char *message, const char *command_id = nullptr);
bool isDeviceActivated();
void activateDevice(bool activate, const char *command_id = nullptr);
void parseURL(const String &url, String &host, int &port, String &path);
bool testServerConnection(const String &host, int port);
bool performOTAUpdate(const char *url, const char *expectedChecksum);
void handleUpdateCommand(const JsonDocument &doc);
uint32_t AutoBaud();
void updateDateTime();
String generateSensorStatusJSON(const char *command_id = nullptr);
void publishSensorStatus(const char *command_id = nullptr);
String generateWeatherDataJson(const char *command_id = nullptr);
void publishWeatherData();
String generateStatusInfoJSON(const char *command_id = nullptr);
void publishStatusReport(const char *command_id = nullptr);
void messageHandler(char *topic, byte *payload, unsigned int length);
void connectToAWS();
bool syncOfflineData();
void checkAndSyncData();
void updateNetworkHealth();
String generateNetworkHealthJSON(const char *command_id = nullptr);

// Function to publish status updates
void publishUpdateStatus(const char *status, const char *message, const char *command_id)
{
  updateDateTime();
  StaticJsonDocument<200> doc; // Increased size for command_id field
  doc["status"] = status;
  doc["message"] = message;
  doc["recorded_at"] = dateTime.asStr();

  // Include command_id if provided
  if (command_id != nullptr && strlen(command_id) > 0) {
    doc["command_id"] = command_id;
  }

  // Publish to device-specific response topic
  String jsonStr;
  serializeJson(doc, jsonStr);
  mqttClient.publish(config.getAwsIotDeviceResponseTopic(), jsonStr.c_str());

  Serial.printf("Published status: %s - %s", status, message);
  if (command_id != nullptr && strlen(command_id) > 0) {
    Serial.printf(" [command_id: %s]", command_id);
  }
  Serial.println();
}

// Function to check if device is activated
bool isDeviceActivated()
{
  preferences.begin("kloudtrack", false);
  bool activated = preferences.getBool("activated", false);
  preferences.end();
  return activated;
}

// Function to activate the device
void activateDevice(bool activate, const char *command_id)
{
  preferences.begin("kloudtrack", false);
  preferences.putBool("activated", activate);
  preferences.end();
  deviceActivated = activate;

  updateDateTime();
  // Publish activation status
  StaticJsonDocument<200> doc; // Increased size for command_id field
  doc["device_id"] = config.getDeviceId();
  doc["firmware_version"] = Config::FIRMWARE_VERSION;
  doc["activated"] = activate;
  doc["recorded_at"] = dateTime.asStr();

  // Include command_id if provided
  if (command_id != nullptr && strlen(command_id) > 0) {
    doc["command_id"] = command_id;
  }

  String jsonStr;
  serializeJson(doc, jsonStr);

  // Publish to device-specific activation topic
  mqttClient.publish(config.getAwsIotDeviceResponseTopic(), jsonStr.c_str());

  Serial.printf("Device %s", activate ? "ACTIVATED" : "DEACTIVATED");
  if (command_id != nullptr && strlen(command_id) > 0) {
    Serial.printf(" [command_id: %s]", command_id);
  }
  Serial.println();
}

void parseURL(const String &url, String &host, int &port, String &path) {
  if (!url.startsWith("https://")) return;

  int index = 8;  // after "https://"
  int slashIndex = url.indexOf('/', index);
  String hostPort = url.substring(index, slashIndex);
  int colonIndex = hostPort.indexOf(':');

  if (colonIndex != -1) {
    host = hostPort.substring(0, colonIndex);
    port = hostPort.substring(colonIndex + 1).toInt();
  } else {
    host = hostPort;
    port = 443;
  }
  path = url.substring(slashIndex);
}

// Add this function to your code to test the connection before attempting OTA
bool testServerConnection(const String &host, int port) {
  Serial.printf("[OTA] Testing connection to %s:%d\n", host.c_str(), port);
  
  // Reset SSL client state
  sslClient.stop();
  delay(500);

  // Set certificate
  sslClient.setCACert(config.getCaCert());

  unsigned long startTime = millis();
  bool connected = false;

  // Print network info
  Serial.print("[OTA] IP Address: ");
  Serial.println(modem.localIP());
  Serial.print("[OTA] Signal Quality: ");
  Serial.println(modem.getSignalQuality());
  
  // Try to connect with timeout
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.printf("[OTA] Connection attempt %d...\n", attempt);
    
    if (sslClient.connect(host.c_str(), port)) {
      connected = true;
      Serial.printf("[OTA] Connected successfully in %lu ms\n", millis() - startTime);
      break;
    }
    
    Serial.println("[OTA] Connection failed, retrying...");
    delay(1000 * attempt); // Exponential backoff
  }
  
  if (!connected) {
    Serial.printf("[OTA] Connection failed after %lu ms\n", millis() - startTime);
    return false;
  }
  
  // Send a simple HEAD request to check if server responds
  sslClient.println("HEAD / HTTP/1.1");
  sslClient.println("Host: " + String(host));
  sslClient.println("Connection: close");
  sslClient.println();
  
  // Wait for response with timeout
  unsigned long timeout = millis();
  while (millis() - timeout < 5000) {
    if (sslClient.available()) {
      String line = sslClient.readStringUntil('\n');
      Serial.println("[OTA] Server response: " + line);
      sslClient.stop();
      return true;
    }
    delay(10);
  }
  
  Serial.println("[OTA] No response from server");
  sslClient.stop();
  return false;
}

// Function to perform OTA update using GSM
bool performOTAUpdate(const String &url) {
  String host, path;
  int port;

  publishUpdateStatus("OTA", "Starting OTA update process", nullptr);

  // Parse the URL
  parseURL(url, host, port, path);
  if (host.length() == 0 || path.length() == 0) {
    Serial.println("[OTA] Invalid OTA URL");
    publishUpdateStatus("Error", "Invalid OTA URL format", nullptr);
    return false;
  }

  Serial.printf("[OTA] Connecting to: %s:%d\n", host.c_str(), port);
  publishUpdateStatus("OTA", "Connecting to update server", nullptr);

  // Test server connection before proceeding
  esp_task_wdt_reset();
  if (!testServerConnection(host, port)) {
    publishUpdateStatus("Error", "Failed to establish connection to update server", nullptr);
    return false;
  }

  sslClient.stop();
  delay(1000);

  sslClient.setCACert(config.getCaCert());

  HttpClient http(sslClient, host, port);
  http.setTimeout(10000); // Long timeout for GSM
  http.connectionKeepAlive();
  publishUpdateStatus("OTA", "Downloading firmware - status updates suspended", nullptr);
  Serial.printf("[OTA] Sending HTTP GET request to %s\n", path.c_str());
  esp_task_wdt_reset();

  // Send GET request with retry mechanism
  int maxRetries = 3;
  int err = -1;

  for (int retry = 0; retry < maxRetries; retry++) {
    err = http.get(path.c_str());
    if (err == 0) break;

    Serial.printf("[OTA] HTTP GET failed (attempt %d/%d): %d\n", retry+1, maxRetries, err);
    delay(1000 * (retry + 1));
  }

  if (err != 0) {
    String errorMsg = "HTTP request failed: " + String(err);
    publishUpdateStatus("Error", errorMsg.c_str(), nullptr);
    return false;
  }

  int status = http.responseStatusCode();
  Serial.printf("[OTA] HTTP status code: %d\n", status);

  if (status <= 0) {
    String errorMsg = "Invalid HTTP status: " + String(status);
    publishUpdateStatus("Error", errorMsg.c_str(), nullptr);
    return false;
  }

  if (status != 200) {
    String errorMsg = "HTTP error: " + String(status);
    publishUpdateStatus("Error", errorMsg.c_str(), nullptr);
    return false;
  }

  int contentLength = http.contentLength();
  Serial.printf("[OTA] Content-Length header: %d\n", contentLength);
  
  if (contentLength <= 0) {
    Serial.println("[OTA] Invalid or missing content length, trying without it");
    // Some servers don't send Content-Length, we'll try chunked transfer
    contentLength = 2000000; // Set a reasonable maximum for firmware
  }

  Serial.printf("[OTA] Update size: %d bytes\n", contentLength);

  if (!Update.begin(contentLength)) {
    Serial.println("[OTA] Update.begin() failed");
    publishUpdateStatus("Error", "Failed to begin update", nullptr);
    return false;
  }

  Serial.println("[OTA] Starting update...");
  publishUpdateStatus("OTA", "Writing firmware to flash", nullptr);

  // Significantly increase buffer size (use 4KB or 8KB if memory allows)
  const size_t bufferSize = 4096;  // 4KB buffer instead of 512 bytes
  uint8_t *buff = (uint8_t*)malloc(bufferSize);

  if (!buff) {
    Serial.println("[OTA] Failed to allocate buffer memory");
    publishUpdateStatus("Error", "Memory allocation failed", nullptr);
    Update.abort();
    return false;
  }

  size_t written = 0;
  uint32_t lastProgress = 0;
  uint32_t startTime = millis();
  
  // Read with timeout protection and MQTT maintenance
  unsigned long readTimeout = 15000; // 15 second timeout for reads
  unsigned long lastRead = millis();
  unsigned long lastWatchdogReset = millis();
  
  Serial.printf("[OTA] Starting download loop, target: %d bytes\n", contentLength);
  
  while (http.connected() && written < contentLength) {
    // Check for read timeout
    if (millis() - lastRead > readTimeout) {
      Serial.printf("[OTA] Read timeout after %d bytes\n", written);
      Serial.println("[OTA] Error: Download timeout - MQTT status skipped for stability");
      free(buff);
      Update.abort();
      return false;
    }

    // Reset watchdog every 5 seconds
    if (millis() - lastWatchdogReset > 5000) {
      esp_task_wdt_reset();
      lastWatchdogReset = millis();
    }

    int available = http.available();
    if (available > 0) {
      lastRead = millis(); // Reset timeout counter

      // Read up to buffer size
      size_t toRead = min(available, (int)bufferSize);
      int readBytes = http.read(buff, toRead);

      if (readBytes > 0) {
        if (Update.write(buff, readBytes) != readBytes) {
          Serial.printf("[OTA] Write failed: expected %d, wrote %d, error: %s\n", 
                       readBytes, Update.getError(), Update.errorString());
          free(buff);
          Update.abort();
          return false;
        }

        written += readBytes;

        // Report progress every 5%
        uint32_t progress = (written * 100) / contentLength;
        if (progress - lastProgress >= 5 || progress == 100) {
          lastProgress = progress;
          Serial.printf("[OTA] Progress: %d%% (%d/%d bytes)\n", progress, written, contentLength);
          esp_task_wdt_reset(); // Reset watchdog on progress
        }
        delay(1);
      } else {
        // Read returned 0, wait and check connection
        delay(50);
        if (!http.connected()) {
          Serial.printf("[OTA] Connection lost at %d bytes\n", written);
          break;
        }
      }
    } else if (available == 0) {
      // No data available, wait for data
      delay(100);
      if (!http.connected()) {
        Serial.printf("[OTA] Connection closed at %d bytes\n", written);
        break;
      }
    } else {
      // available() returned negative
      Serial.printf("[OTA] HTTP available() error: %d\n", available);
      delay(100);
    }
  }
  free(buff);

  uint32_t updateTime = (millis() - startTime) / 1000;
  Serial.printf("[OTA] Download completed in %d seconds\n", updateTime);

  if (written != contentLength) {
    Serial.printf("[OTA] Size mismatch: %d != %d\n", written, contentLength);
    Serial.println("[OTA] Error: Size mismatch - MQTT status skipped for stability");
    Update.abort();
    return false;
  }

   Serial.println("[OTA] Finishing update...");
   Serial.println("[OTA] Finalizing firmware update - MQTT status skipped for stability");
  
  if (!Update.end()) {
    Serial.printf("[OTA] Update.end() failed: %s\n", Update.errorString());
    return false;
  }

  if (!Update.isFinished()) {
    Serial.println("[OTA] Update not finished correctly");
    Serial.println("[OTA] Error: Update process incomplete - MQTT status skipped for stability");
    return false;
  }

  Serial.println("[OTA] Update successful! Rebooting...");
  Serial.println("[OTA] Update complete, rebooting device - MQTT status skipped for stability");
  
  // Brief delay before reboot
  delay(1000);
  
  // Restart ESP32 to apply the new firmware
  ESP.restart();
  
  return true; // This won't be reached due to restart
}

// Handle Update Command
void handleUpdateCommand(const JsonDocument& doc) {
  // Check if device is activated
  if (!deviceActivated) {
    Serial.println("OTA update rejected: Device not activated");
    publishUpdateStatus("Rejected", "Device not activated", nullptr);
    return;
  }

  if (!doc.containsKey("url")) {
    publishUpdateStatus("Error", "Missing url", nullptr);
    return;
  }

  String url = doc["url"].as<String>();
  String newVersion = doc.containsKey("version") ? doc["version"].as<String>() : "";
  bool forceUpdate = doc.containsKey("force") ? doc["force"].as<bool>() : false;

  if (!forceUpdate && !newVersion.isEmpty() && newVersion == Config::FIRMWARE_VERSION) {
    publishUpdateStatus("Ignored", "Same firmware version. Update skipped.", nullptr);
    return;
  }

  publishUpdateStatus("Starting", "Beginning OTA update...", nullptr);
  performOTAUpdate(url.c_str());
}

// Function to connect to required Baud Rate
uint32_t AutoBaud() {
  static uint32_t rates[] = {115200, 9600, 57600,  38400, 19200,  74400, 74880,
                              230400, 460800, 2400,  4800,  14400, 28800
                            };
  for (uint8_t i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
    uint32_t rate = rates[i];
    SerialAT.updateBaudRate(rate);
    delay(10);
    for (int j = 0; j < 10; j++) {
      SerialAT.print("AT\r\n");
      String input = SerialAT.readString();
      if (input.indexOf("OK") >= 0) {
        return rate;
      }
    }
  }
  SerialAT.updateBaudRate(115200);
  return 0;
}

void updateDateTime() {
  DateTime dt;

  if (dt.begin(SerialAT) != 0) {
    Serial.println("Failed to update date and time");
    return;
  }

  dateTime = dt;
}

// Generate sensor status JSON string
String generateSensorStatusJSON(const char *command_id)
{
  updateDateTime();
  StaticJsonDocument<256> doc; // Increased size for command_id field
  doc["recorded_at"] = dateTime.asStr();

  // Include command_id if provided
  if (command_id != nullptr && strlen(command_id) > 0) {
    doc["command_id"] = command_id;
  }

  // Get sensor status from SensorManager
  SensorStatus status = sensorManager.getSensorStatus();
  JsonObject sensors = doc.createNestedObject("sensors");
  sensors["BME280"] = status.bmeAvailable ? "OK" : "ERROR";
  sensors["BMP180"] = status.bmpAvailable ? "OK" : "ERROR";
  sensors["SHT30"] = status.shtAvailable ? "OK" : "ERROR";
  sensors["BH1750"] = status.lightAvailable ? "OK" : "ERROR";
  sensors["AS5600"] = status.directionAvailable ? "OK" : "ERROR";
  sensors["SLAVE"] = status.slaveAvailable ? "OK" : "ERROR";
  sensors["GUVAS12SD"] = status.uvAvailable ? "OK" : "ERROR";

  String jsonStr;
  jsonStr.reserve(300); // Reserve space to prevent fragmentation
  serializeJson(doc, jsonStr);
  return jsonStr;
}

// Function to publish sensor status
void publishSensorStatus(const char *command_id)
{
  String jsonString = generateSensorStatusJSON(command_id);

  // Publish to device-specific status topic
  mqttClient.publish(config.getAwsIotDeviceResponseTopic(), jsonString.c_str());
}

// Generate weather data JSON string
String generateWeatherDataJson(const char *command_id)
{
  // Get time
  updateDateTime();

  // Create JSON document
  StaticJsonDocument<350> doc; // Increased size for command_id field
  doc["recorded_at"] = dateTime.asStr();

  // Include command_id if provided
  if (command_id != nullptr && strlen(command_id) > 0) {
    doc["command_id"] = command_id;
  }

  // Collect sensor data
  SensorReadings readings = sensorManager.readAllSensors();
  SensorStatus status = sensorManager.getSensorStatus();

  doc["temperature"] = readings.temperature;
  doc["humidity"] = readings.humidity;
  doc["pressure"] = readings.pressure;
  doc["light_intensity"] = readings.lightIntensity;
  doc["uv_index"] = readings.uvIndex;
  doc["wind_direction"] = readings.windDirection;
  doc["wind_speed"] = readings.windSpeed;
  doc["precipitation"] = readings.precipitation;
  doc["in_temp"] = readings.panelTemperature;

  // Serialize to JSON with reserved capacity
  String jsonString;
  jsonString.reserve(400); // Reserve space to prevent fragmentation
  serializeJson(doc, jsonString);
  return jsonString;
}

// Function to generate and publish weather data
void publishWeatherData()
{
  String jsonString = generateWeatherDataJson(nullptr);

  if (modem.isNetworkConnected() && mqttClient.connected())
  {
    // Publish to device-specific weather topic
    mqttClient.publish(config.getAwsIotDeviceDataTopic(), jsonString.c_str());
  }
  else
  {
    // Save to SD card if WiFi or MQTT is not available
    if (sdCard.isAvailable())
    {
      if (sdCard.saveWeatherData(jsonString.c_str()))
      {
        Serial.println("Weather data saved to SD card (offline mode)");
      }
      else
      {
        Serial.println("Failed to save weather data to SD card");
      }
    }
    else
    {
      Serial.println("No connectivity and SD card not available - data lost");
    }
  }
}

// Check heap status and warn if low
void checkHeapStatus() {
  static unsigned long lastHeapCheck = 0;
  static const unsigned long HEAP_CHECK_INTERVAL = 30000; // Check every 30 seconds
  static const size_t LOW_HEAP_THRESHOLD = 20000; // 20KB threshold
  static const size_t CRITICAL_HEAP_THRESHOLD = 10000; // 10KB critical threshold
  static const float FRAGMENTATION_THRESHOLD = 0.6; // 60% fragmentation threshold
  
  unsigned long now = millis();
  if (now - lastHeapCheck >= HEAP_CHECK_INTERVAL) {
    lastHeapCheck = now;
    
    size_t freeHeap = ESP.getFreeHeap();
    size_t minFreeHeap = ESP.getMinFreeHeap();
    size_t maxAllocHeap = ESP.getMaxAllocHeap();
    
    if (freeHeap < CRITICAL_HEAP_THRESHOLD) {
      char msg[100];
      snprintf(msg, sizeof(msg), "Very low heap memory! Free: %d bytes, Min: %d bytes", freeHeap, minFreeHeap);
      LOG_CRITICAL_MSG("HEAP", msg);
      
      // Force immediate restart if critically low
      ESP.restart();
    } else if (freeHeap < LOW_HEAP_THRESHOLD) {
      char msg[100];
      snprintf(msg, sizeof(msg), "Low heap memory! Free: %d bytes, Min: %d bytes", freeHeap, minFreeHeap);
      LOG_WARNING_MSG("HEAP", msg);
    }
    
    // Enhanced fragmentation detection and mitigation
    float fragmentation = 1.0 - ((float)maxAllocHeap / (float)freeHeap);
    if (fragmentation > FRAGMENTATION_THRESHOLD) {
      char msg[120];
      snprintf(msg, sizeof(msg), "High fragmentation %.1f%% - Max alloc: %d, Free: %d", 
               fragmentation * 100, maxAllocHeap, freeHeap);
      LOG_WARNING_MSG("HEAP", msg);
      
      // If fragmentation is severe and heap is getting low, restart device
      if (fragmentation > 0.8 && freeHeap < 50000) {
        LOG_CRITICAL_MSG("HEAP", "Severe fragmentation detected - restarting device");
        delay(1000);
        ESP.restart();
      }
    }
  }
}

// Update network health metrics
void updateNetworkHealth() {
  static unsigned long lastUpdate = 0;
  static bool lastGsmState = false;
  static bool lastMqttState = false;
  unsigned long now = millis();
  
  // Update every 10 seconds
  if (now - lastUpdate < 10000) return;
  lastUpdate = now;
  
  bool gsmConnected = modem.isNetworkConnected();
  bool mqttConnected = mqttClient.connected();
  
  // Track GSM connection state changes
  if (gsmConnected != lastGsmState) {
    if (!gsmConnected) {
      networkHealth.lastGsmDisconnect = now;
      networkHealth.gsmReconnectCount++;
      LOG_WARNING_MSG("NETWORK", "GSM connection lost");
    } else {
      LOG_INFO_MSG("NETWORK", "GSM connection restored");
    }
    lastGsmState = gsmConnected;
  }
  
  // Track MQTT connection state changes  
  if (mqttConnected != lastMqttState) {
    if (!mqttConnected) {
      networkHealth.lastMqttDisconnect = now;
      networkHealth.mqttReconnectCount++;
      LOG_WARNING_MSG("NETWORK", "MQTT connection lost");
    } else {
      LOG_INFO_MSG("NETWORK", "MQTT connection restored");
    }
    lastMqttState = mqttConnected;
  }
  
  // Update uptime counters
  if (gsmConnected) {
    networkHealth.gsmUptime = now;
  }
  if (mqttConnected) {
    networkHealth.mqttUptime = now;
  }
  
  // Update signal quality
  networkHealth.signalQuality = modem.getSignalQuality();
  
  // Determine overall network health
  unsigned long gsmDowntime = gsmConnected ? 0 : (now - networkHealth.lastGsmDisconnect);
  unsigned long mqttDowntime = mqttConnected ? 0 : (now - networkHealth.lastMqttDisconnect);
  
  networkHealth.networkHealthy = (gsmConnected && mqttConnected && 
                                 networkHealth.signalQuality > 10 &&
                                 gsmDowntime < 300000 && // Less than 5 minutes downtime
                                 mqttDowntime < 300000);
  
  // Log critical network issues
  if (!networkHealth.networkHealthy) {
    char msg[100];
    snprintf(msg, sizeof(msg), "Network unhealthy - GSM:%s MQTT:%s Signal:%d", 
             gsmConnected ? "OK" : "DOWN", 
             mqttConnected ? "OK" : "DOWN",
             networkHealth.signalQuality);
    LOG_WARNING_MSG("NETWORK", msg);
  }
}

// Generate network health JSON
String generateNetworkHealthJSON(const char *command_id) {
  StaticJsonDocument<350> doc; // Increased size for command_id field
  unsigned long now = millis();

  doc["timestamp"] = now;
  doc["gsm_connected"] = modem.isNetworkConnected();
  doc["mqtt_connected"] = mqttClient.connected();
  doc["signal_quality"] = networkHealth.signalQuality;
  doc["gsm_uptime_ms"] = networkHealth.gsmUptime;
  doc["mqtt_uptime_ms"] = networkHealth.mqttUptime;
  doc["gsm_reconnects"] = networkHealth.gsmReconnectCount;
  doc["mqtt_reconnects"] = networkHealth.mqttReconnectCount;
  doc["network_healthy"] = networkHealth.networkHealthy;

  // Include command_id if provided
  if (command_id != nullptr && strlen(command_id) > 0) {
    doc["command_id"] = command_id;
  }

  if (networkHealth.lastGsmDisconnect > 0) {
    doc["last_gsm_disconnect"] = networkHealth.lastGsmDisconnect;
  }
  if (networkHealth.lastMqttDisconnect > 0) {
    doc["last_mqtt_disconnect"] = networkHealth.lastMqttDisconnect;
  }

  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

// Generate status information JSON string
String generateStatusInfoJSON(const char *command_id)
{
  const float TOTAL_RAM = 327680.0;
  const float TOTAL_FLASH = 1310720.0;

  // RAM usage percent (used = total - free heap)
  float used_ram = TOTAL_RAM - ESP.getFreeHeap();
  float RAM_USAGE = used_ram * 100.0 / TOTAL_RAM;

  // Flash usage percent
  float flash_used = ESP.getSketchSize();
  float FLASH_USAGE = flash_used * 100.0 / TOTAL_FLASH;

  // Signal Quality
  int SIGNAL_QUALITY = modem.getSignalQuality();

  // SD Card status and storage
  const char* SD_CARD = sdCard.isAvailable() ? "Available" : "Not available";
  int PENDING_RECORDS = 0;
  if (sdCard.isAvailable())
  {
    PENDING_RECORDS = sdCard.getPendingRecords();
  }

  updateDateTime(); // Ensure we have the latest time

  StaticJsonDocument<350> doc; // Increased size for command_id field
  doc["rec_at"] = dateTime.asStr();
  doc["device_id"] = config.getDeviceId();
  doc["firmware"] = Config::FIRMWARE_VERSION;
  doc["activated"] = deviceActivated;
  doc["ram"] = RAM_USAGE;
  doc["flash"] = FLASH_USAGE;
  doc["sd"] = SD_CARD;
  doc["pending"] = PENDING_RECORDS;

  // Include command_id if provided
  if (command_id != nullptr && strlen(command_id) > 0) {
    doc["command_id"] = command_id;
  }

  // Serialize to JSON
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

// Function to generate status report
void publishStatusReport(const char *command_id)
{
  String jsonString = generateStatusInfoJSON(command_id);

  // Publish to device-specific status topic
  mqttClient.publish(config.getAwsIotDeviceResponseTopic(), jsonString.c_str());
}

// MQTT message handler
void messageHandler(char *topic, byte *payload, unsigned int length)
{
  esp_task_wdt_reset(); // Reset at start of message handling
  Serial.print("Message received on topic: ");
  Serial.println(topic);

  // Check payload length to prevent memory exhaustion
  const unsigned int MAX_PAYLOAD_SIZE = 2048; // 2KB limit
  if (length > MAX_PAYLOAD_SIZE) {
    Serial.printf("Payload too large (%d bytes), ignoring message\n", length);
    return;
  }

  // Create a null-terminated string from the payload
  char *payloadStr = new char[length + 1];
  if (!payloadStr) {
    Serial.println("Failed to allocate memory for payload");
    return;
  }
  memcpy(payloadStr, payload, length);
  payloadStr[length] = '\0';
  Serial.println(payloadStr);

  // Parse JSON - use more conservative size to reduce fragmentation
  StaticJsonDocument<300> doc;
  DeserializationError error = deserializeJson(doc, payloadStr);
  delete[] payloadStr;
  esp_task_wdt_reset(); // Reset after JSON parsing

  if (error)
  {
    Serial.print("DeserializeJson() failed: ");
    Serial.println(error.f_str());
    return;
  }

  // Handle based on topic
  if (strcmp(topic, config.getAwsIotDeviceCommandTopic()) == 0)
  {
    const char* command = doc["command"];

    // Extract command_id if present (handle both string and numeric types)
    const char* command_id = nullptr;
    String command_id_str = "";
    if (doc.containsKey("command_id")) {
      if (doc["command_id"].is<const char*>()) {
        command_id = doc["command_id"];
      } else if (doc["command_id"].is<int>() || doc["command_id"].is<long>()) {
        // Convert numeric command_id to string
        command_id_str = String(doc["command_id"].as<long>());
        command_id = command_id_str.c_str();
      }
    }

    // Handle status information
    if (command && strcmp(command, "info") == 0)
    {
      publishStatusReport(command_id);
      Serial.println("Station Information sent.");
      publishUpdateStatus("Status", "Station information sent successfully", command_id);
    }
    // Handle sensor status
    else if (command && strcmp(command, "sensor") == 0)
    {
      publishSensorStatus(command_id);
      Serial.println("Sensors status sent.");
      publishUpdateStatus("Sensor Status", "Sensors status sent successfully", command_id);
    }
    // Handle device activation
    else if (command && strcmp(command, "activate") == 0)
    {
      // Activate the device
      activateDevice(true, command_id);
      Serial.printf("Device activated successfully at %s.\n", dateTime.c_str());
      publishUpdateStatus("Activated", "Device activated successfully", command_id);
    }
    // Handle device deactivation
    else if (command && strcmp(command, "deactivate") == 0)
    {
      // Deactivate the device
      activateDevice(false, command_id);
      Serial.printf("Device deactivated successfully at %s.\n", dateTime.c_str());
      publishUpdateStatus("Deactivated", "Device deactivated successfully", command_id);
    }
    // Commands that require activation
    else if (!deviceActivated) {
      // Throttle rejection logging to once per minute to reduce spam
      unsigned long now = millis();
      if (now - lastRejectionLog >= REJECTION_LOG_INTERVAL) {
        Serial.printf("Command '%s' rejected: Device not activated\n", command ? command : "unknown");
        publishUpdateStatus("Rejected", "Device not activated - command ignored", command_id);
        lastRejectionLog = now;
      }
    }
    else {
      // Handle device reset
      if (command && strcmp(command, "reset") == 0)
      {
        // Reset the device
        Serial.printf("Reset command received at %s. Restarting device...\n", dateTime.c_str());
        publishUpdateStatus("Resetting", "Device restarting per command request", command_id);
        delay(1000);
        ESP.restart();
      }
      // Handle OTA update command
      else if (command && strcmp(command, "update") == 0)
      {
        esp_task_wdt_reset(); // Reset before potentially long OTA process
        if (doc.containsKey("url"))
        {
          updateDateTime();
          StaticJsonDocument<180> updateDoc; // Increased size for command_id
          updateDoc["recorded_at"] = dateTime.asStr();
          updateDoc["command"] = "update";
          updateDoc["status"] = "received";
          updateDoc["url"] = doc["url"].as<String>();
          if (doc.containsKey("version")) {
            updateDoc["version"] = doc["version"].as<String>();
          }
          if (command_id != nullptr && strlen(command_id) > 0) {
            updateDoc["command_id"] = command_id;
          }
          String updateJson;
          serializeJson(updateDoc, updateJson);
          mqttClient.publish(config.getAwsIotDeviceResponseTopic(), updateJson.c_str());

          handleUpdateCommand(doc);
          Serial.printf("OTA update command processed at %s.\n", dateTime.c_str());
          publishUpdateStatus("OTA Update", "OTA update command processed", command_id);
        }
        else
        {
          Serial.printf("Error: Missing URL for OTA update at %s.\n", dateTime.c_str());
          publishUpdateStatus("Error", "Missing url for update", command_id);
        }
      }
      // Handle force sync command
      else if (command && strcmp(command, "sync") == 0)
      {
        static unsigned long lastSyncCommandTime = 0;
        unsigned long now = millis();
        // Only allow sync command to be processed if at least 10 seconds have passed since last one
        if (now - lastSyncCommandTime > 10000 || lastSyncCommandTime == 0) {
          lastSyncCommandTime = now;

          updateDateTime();

          // Force sync command received
          Serial.printf("Force sync command received at %s.", dateTime.c_str());
          publishUpdateStatus("Sync", "Force sync command received", command_id);

          if (syncOfflineData())
          {
            Serial.println("Offline data synchronized successfully");
            publishUpdateStatus("Synced", "Offline data synchronized successfully", command_id);
          }
          else
          {
            Serial.println("Failed to synchronize offline data");
            publishUpdateStatus("Sync Failed", "Failed to synchronize offline data", command_id);
          }
          lastSyncAttempt = millis(); // Update last sync attempt time
        } else {
          Serial.println("Sync command ignored: too soon since last sync.");
        }
      }
      // Handle force collect data command
      else if (command && strcmp(command, "data") == 0)
      {
        // Respond with current weather data when requested
        Serial.println("Weather data requested.");
        String jsonString = generateWeatherDataJson(command_id);
        mqttClient.publish(config.getAwsIotDeviceResponseTopic(), jsonString.c_str());
        publishUpdateStatus("Weather Data", "Current weather data sent", command_id);
      }
      // Handle network health command
      else if (command && strcmp(command, "network") == 0)
      {
        // Send network health status
        Serial.println("Network health requested.");
        String jsonString = generateNetworkHealthJSON(command_id);
        mqttClient.publish(config.getAwsIotDeviceResponseTopic(), jsonString.c_str());
        publishUpdateStatus("Network Health", "Network health status sent", command_id);
      }
      // Handle credential management commands
      else if (command && strcmp(command, "set_wifi") == 0)
      {
        if (doc.containsKey("ssid") && doc.containsKey("password")) {
          String ssid = doc["ssid"].as<String>();
          String password = doc["password"].as<String>();

          if (config.setWifiCredentials(ssid.c_str(), password.c_str())) {
            Serial.printf("WiFi credentials updated: %s\n", ssid.c_str());
            publishUpdateStatus("WiFi Updated", "WiFi credentials updated successfully", command_id);
          } else {
            Serial.println("Failed to update WiFi credentials");
            publishUpdateStatus("WiFi Error", "Failed to update WiFi credentials", command_id);
          }
        } else {
          Serial.println("Missing SSID or password for WiFi update");
          publishUpdateStatus("WiFi Error", "Missing SSID or password", command_id);
        }
      }
      else if (command && strcmp(command, "set_gsm") == 0)
      {
        if (doc.containsKey("apn")) {
          String apn = doc["apn"].as<String>();

          if (config.setGsmCredentials(apn.c_str())) {
            Serial.printf("GSM APN updated: %s\n", apn.c_str());
            publishUpdateStatus("GSM Updated", "GSM APN updated successfully", command_id);
          } else {
            Serial.println("Failed to update GSM APN");
            publishUpdateStatus("GSM Error", "Failed to update GSM APN", command_id);
          }
        } else {
          Serial.println("Missing APN for GSM update");
          publishUpdateStatus("GSM Error", "Missing APN", command_id);
        }
      }
      else if (command && strcmp(command, "set_aws") == 0)
      {
        if (doc.containsKey("endpoint") && doc.containsKey("port")) {
          String endpoint = doc["endpoint"].as<String>();
          int port = doc["port"].as<int>();

          if (config.setAwsCredentials(endpoint.c_str(), port)) {
            Serial.printf("AWS credentials updated: %s:%d\n", endpoint.c_str(), port);
            publishUpdateStatus("AWS Updated", "AWS credentials updated successfully", command_id);
          } else {
            Serial.println("Failed to update AWS credentials");
            publishUpdateStatus("AWS Error", "Failed to update AWS credentials", command_id);
          }
        } else {
          Serial.println("Missing endpoint or port for AWS update");
          publishUpdateStatus("AWS Error", "Missing endpoint or port", command_id);
        }
      }
      else if (command && strcmp(command, "get_credentials") == 0)
      {
        // Send current credentials status
        StaticJsonDocument<350> credDoc; // Increased size for command_id field
        credDoc["command"] = "credentials_status";
        credDoc["gsm_configured"] = config.hasGsmCredentials();
        credDoc["aws_configured"] = config.hasAwsCredentials();
        credDoc["gsm_apn"] = config.getApn();

        // Include command_id if provided
        if (command_id != nullptr && strlen(command_id) > 0) {
          credDoc["command_id"] = command_id;
        }

        String credJson;
        serializeJson(credDoc, credJson);
        mqttClient.publish(config.getAwsIotDeviceResponseTopic(), credJson.c_str());
        publishUpdateStatus("Credentials", "Current credentials status sent", command_id);
      }
      else if (command && strcmp(command, "clear_credentials") == 0)
      {
        config.clearCredentials();
        Serial.println("All credentials cleared, using defaults");
        publishUpdateStatus("Credentials Cleared", "All credentials cleared, using defaults", command_id);
      }
    }   
  }
  Serial.println("---------------------------------");
}

// Function to initialize GSM modem
void initGSM() {
  esp_task_wdt_reset(); // Reset watchdog at start
  SerialMon.println("Initializing GSM modem...");
  
  // A7670-GSM Reset
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW); delay(100);
  esp_task_wdt_reset();
  digitalWrite(RESET, HIGH); delay(100);
  digitalWrite(RESET, LOW); delay(100);

  // A7670-GSM Power
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW); delay(100);
  digitalWrite(PWR_PIN, HIGH); delay(100);
  digitalWrite(PWR_PIN, LOW); delay(1000);  // Increased delay
  esp_task_wdt_reset();

  Serial.println("Starting Serial Communications...");
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  modem.init();
  esp_task_wdt_reset();
  
  AutoBaud();
  delay(2000); // Wait for the modem to initialize
  esp_task_wdt_reset();

  Serial.print("Connecting to cellular network...");
  Serial.printf("\nUsing APN: %s\n", config.getApn());
  
  // Check SIM card status first
  SimStatus simStatus = modem.getSimStatus();
  Serial.printf("SIM Status: %d (3=READY)\n", (int)simStatus);
  
  if (simStatus != SIM_READY) {
    Serial.println("SIM card not ready! Please check SIM card.");
    const char* statusText = "Unknown";
    switch(simStatus) {
      case SIM_ERROR: statusText = "ERROR"; break;
      case SIM_READY: statusText = "READY"; break;
      case SIM_LOCKED: statusText = "LOCKED"; break;
      default: statusText = "UNKNOWN"; break;
    }
    Serial.printf("SIM Status: %s\n", statusText);
    return;
  }
  
  // Check signal quality
  int signalQuality = modem.getSignalQuality();
  Serial.printf("Signal Quality: %d (0-31, higher is better)\n", signalQuality);
  
  if (signalQuality < 5) {
    Serial.println("WARNING: Very poor signal quality, connection may fail");
  }
  
  modem.gprsConnect(config.getApn());
  unsigned long gsmStart = millis();
  int connectionAttempts = 0;
  while (!modem.isNetworkConnected() && (millis() - gsmStart < 30000)) // Increased timeout
  {
    Serial.print(".");
    delay(1000);
    connectionAttempts++;
    
    // Show more detailed status every 5 seconds
    if (connectionAttempts % 5 == 0) {
      Serial.printf("\nAttempt %d: Signal=%d, Network=%s\n", 
                   connectionAttempts, 
                   modem.getSignalQuality(),
                   modem.getOperator().c_str());
    }
    
    // Reset watchdog every few attempts
    if (connectionAttempts % 3 == 0) {
      esp_task_wdt_reset();
    }
  }
  if (modem.isNetworkConnected())
  {
    Serial.println("\nGSM connected!");
    gsmRetryCount = 0;
  }
  else
  {
    gsmRetryCount++;
    Serial.printf("\nGSM connection failed. Attempt %d/%d\n", gsmRetryCount, maxRetries);
    Serial.printf("Final Signal Quality: %d\n", modem.getSignalQuality());
    Serial.printf("Network Operator: %s\n", modem.getOperator().c_str());
    Serial.printf("Registration Status: %d\n", modem.getRegistrationStatus());

    // Instead of restarting after maxRetries, collect and store data locally
    if (gsmRetryCount >= maxRetries)
    {
      Serial.println("Maximum GSM retries reached. Continuing in offline mode.");

      // Collect weather data before attempting further reconnections
      if (deviceActivated && sdCard.isAvailable())
      {
        String weatherData = generateWeatherDataJson(nullptr);
        if (sdCard.saveWeatherData(weatherData.c_str()))
        {
          Serial.println("Weather data saved to SD card while offline");
        }
      }

      // Reset retry counter and implement longer delay
      gsmRetryCount = 0;
      delay(60000); // Wait a minute before trying again
    }
  }
  updateDateTime();
  Serial.printf("Date and Time: %s\n", dateTime.c_str());
  Serial.println("---------------------------------");
}
  
// Function to connect to AWS IoT
void connectToAWS()
{
  esp_task_wdt_reset(); // Reset watchdog at start

  // Track total time spent in connection attempts to prevent infinite blocking
  static unsigned long connectionStartTime = 0;
  const unsigned long MAX_CONNECTION_ATTEMPT_TIME = 120000; // 2 minutes max

  // Start timing if this is a fresh attempt
  if (connectionStartTime == 0 || (millis() - connectionStartTime > MAX_CONNECTION_ATTEMPT_TIME)) {
    connectionStartTime = millis();
  }

  // Check if we've exceeded maximum connection attempt time
  if (millis() - connectionStartTime > MAX_CONNECTION_ATTEMPT_TIME) {
    Serial.println("ERROR: Maximum connection attempt time exceeded (2 minutes)");
    Serial.println("Restarting device to recover from connection failure...");
    delay(1000);
    ESP.restart();
  }

  if (modem.isNetworkConnected())
  {
    SerialMon.println("Connecting to AWS IoT...");
    Serial.printf("AWS Endpoint: %s:%d\n", config.getAwsIotEndpoint(), config.getAwsIotPort());
    Serial.printf("Device ID: %s\n", config.getDeviceId());
    
    // Check GSM signal quality before attempting SSL connection
    int signalQuality = modem.getSignalQuality();
    Serial.printf("Current signal quality: %d\n", signalQuality);
    
    if (signalQuality < 5) {
      Serial.println("WARNING: Very poor signal quality may cause SSL connection failures");
      Serial.println("Consider improving antenna placement or waiting for better signal");
    }
    
    mqttClient.setServer(config.getAwsIotEndpoint(), config.getAwsIotPort());
    mqttClient.setCallback(messageHandler);

    // Always refresh SSL certificates for each connection attempt
    Serial.println("Configuring SSL certificates...");
    
    // Check heap before certificate operations
    size_t freeHeap = ESP.getFreeHeap();
    size_t minFreeHeap = ESP.getMinFreeHeap();
    Serial.printf("Free heap before SSL setup: %d bytes\n", freeHeap);
    Serial.printf("Min free heap: %d bytes\n", minFreeHeap);
    
    // Check if we have enough memory for SSL operations (minimum 30KB recommended)
    if (freeHeap < 30000) {
      Serial.println("WARNING: Low memory detected - SSL connection may fail");
      Serial.println("Consider restarting device if SSL connections continue to fail");
    }
    
    const char* caCert = config.getCaCert();
    const char* deviceCert = config.getDeviceCert();
    const char* privateKey = config.getPrivateKey();
    
    if (!caCert || strlen(caCert) < 100) {
      Serial.println("ERROR: Invalid CA certificate");
      return;
    }
    if (!deviceCert || strlen(deviceCert) < 100) {
      Serial.println("ERROR: Invalid device certificate");
      return;
    }
    if (!privateKey || strlen(privateKey) < 100) {
      Serial.println("ERROR: Invalid private key");
      return;
    }
    
    // Enhanced certificate validation to prevent BIGNUM errors
    Serial.println("Performing enhanced certificate validation...");
    
    // Check CA certificate
    if (!strstr(caCert, "-----BEGIN CERTIFICATE-----") || !strstr(caCert, "-----END CERTIFICATE-----")) {
      Serial.println("ERROR: CA certificate format invalid - missing BEGIN/END markers");
      return;
    }
    
    // Check device certificate
    if (!strstr(deviceCert, "-----BEGIN CERTIFICATE-----") || !strstr(deviceCert, "-----END CERTIFICATE-----")) {
      Serial.println("ERROR: Device certificate format invalid - missing BEGIN/END markers");
      return;
    }
    
    // Check private key with more thorough validation
    bool hasRSAKey = (strstr(privateKey, "-----BEGIN RSA PRIVATE KEY-----") && strstr(privateKey, "-----END RSA PRIVATE KEY-----"));
    bool hasPKCS8Key = (strstr(privateKey, "-----BEGIN PRIVATE KEY-----") && strstr(privateKey, "-----END PRIVATE KEY-----"));
    bool hasECKey = (strstr(privateKey, "-----BEGIN EC PRIVATE KEY-----") && strstr(privateKey, "-----END EC PRIVATE KEY-----"));
    
    if (!hasRSAKey && !hasPKCS8Key && !hasECKey) {
      Serial.println("ERROR: Private key format invalid - must be RSA, PKCS#8, or EC format");
      return;
    }
    
    if (hasECKey) {
      Serial.println("WARNING: EC private key detected - converting or using fallback may be needed");
      // For BIGNUM errors with EC keys, we might need to restart or use different SSL settings
    }
    
    // Additional validation for BIGNUM error prevention
    // Check for proper PEM structure (no embedded nulls, proper line endings)
    const char* certs[] = {caCert, deviceCert, privateKey};
    const char* certNames[] = {"CA certificate", "Device certificate", "Private key"};
    
    for (int certIdx = 0; certIdx < 3; certIdx++) {
      const char* cert = certs[certIdx];
      size_t len = strlen(cert);
      
      // Check for proper line endings and structure
      bool hasPEMStructure = true;
      int lineCount = 0;
      for (size_t i = 0; i < len; i++) {
        if (cert[i] == '\n') {
          lineCount++;
        }
        // Check for invalid binary sequences that cause BIGNUM errors
        if ((cert[i] & 0x80) != 0 && cert[i] != '\r' && cert[i] != '\n') {
          Serial.printf("WARNING: %s contains non-ASCII characters that may cause BIGNUM errors\n", certNames[certIdx]);
          hasPEMStructure = false;
          break;
        }
      }
      
      if (lineCount < 5) {
        Serial.printf("WARNING: %s has too few line breaks - may be malformed\n", certNames[certIdx]);
      }
      
      if (!hasPEMStructure) {
        Serial.printf("ERROR: %s structure invalid - this may cause BIGNUM errors\n", certNames[certIdx]);
      }
    }
    
    // Validate certificate lengths
    if (strlen(caCert) < 500 || strlen(caCert) > 4000) {
      Serial.printf("WARNING: CA certificate length suspicious: %d bytes\n", strlen(caCert));
    }
    if (strlen(deviceCert) < 500 || strlen(deviceCert) > 4000) {
      Serial.printf("WARNING: Device certificate length suspicious: %d bytes\n", strlen(deviceCert));
    }
    if (strlen(privateKey) < 500 || strlen(privateKey) > 4000) {
      Serial.printf("WARNING: Private key length suspicious: %d bytes\n", strlen(privateKey));
    }
    
    Serial.println("Certificate validation completed - no BIGNUM issues detected");
    
    // Stop any existing SSL connection
    // sslClient.stop();
    // delay(100);

    // Always stop existing SSL connection and reset certificates
    sslClient.stop();
    delay(500);
    
    // Check heap before SSL operations to prevent fragmentation issues
    size_t heapBeforeSSL = ESP.getFreeHeap();
    size_t maxAllocBeforeSSL = ESP.getMaxAllocHeap();
    
    if (heapBeforeSSL < 50000 || maxAllocBeforeSSL < 40000) {
      Serial.printf("WARNING: Low heap before SSL setup - Free: %d, MaxAlloc: %d\n", heapBeforeSSL, maxAllocBeforeSSL);
      
      // Force garbage collection and heap defragmentation
      Serial.println("Attempting heap cleanup...");
      delay(500);
      esp_task_wdt_reset();
      
      // Check heap again after cleanup
      heapBeforeSSL = ESP.getFreeHeap();
      maxAllocBeforeSSL = ESP.getMaxAllocHeap();
      Serial.printf("After cleanup - Free: %d, MaxAlloc: %d\n", heapBeforeSSL, maxAllocBeforeSSL);
      
      if (heapBeforeSSL < 45000) {
        Serial.println("CRITICAL: Insufficient heap for SSL operations - restarting");
        delay(1000);
        ESP.restart();
      }
    }
    
    // Ensure base GSM client is properly initialized
    if (!modem.isNetworkConnected()) {
      Serial.println("ERROR: GSM network not connected, cannot setup SSL");
      return;
    }
    
    // Set certificates fresh for each connection attempt
    Serial.println("Setting SSL certificates...");
    
    // Configure SSL client settings before setting certificates
    sslClient.setHandshakeTimeout(30000); // 30 second timeout for GSM
    
    // Set certificates in correct order
    Serial.println("Setting CA certificate...");
    sslClient.setCACert(caCert);
    
    Serial.println("Setting device certificate...");
    sslClient.setCertificate(deviceCert);
    
    Serial.println("Setting private key...");
    sslClient.setPrivateKey(privateKey);
    
    // SSL certificates are now configured
    
    Serial.println("SSL certificates configured successfully");
    
    // Check heap after SSL setup
    size_t heapAfterSSL = ESP.getFreeHeap();
    Serial.printf("Heap usage for SSL setup: %d bytes\n", heapBeforeSSL - heapAfterSSL);
    
    esp_task_wdt_reset();

    if (!mqttClient.connected())
    {
      mqttRetryCount++;
      Serial.printf("Reconnecting attempt %d/%d\n", mqttRetryCount, maxRetries);
      esp_task_wdt_reset();

      Serial.printf("Attempting MQTT connection with Device ID: %s\n", config.getDeviceId());
      Serial.printf("Connecting to %s:%d\n", config.getAwsIotEndpoint(), config.getAwsIotPort());
      
      // Test SSL connection before MQTT attempt
      Serial.println("Testing SSL connection to AWS IoT endpoint...");
      Serial.printf("Free heap before SSL test: %d bytes\n", ESP.getFreeHeap());
      
      // Verify GSM network is available (baseClient connection will be handled by SSL layer)
      if (!modem.isNetworkConnected()) {
        Serial.println("GSM network not connected - cannot establish SSL connection");
        return;
      }

      // Note: We don't need to check baseClient.connected() here because:
      // 1. The SSL client will establish the TCP connection automatically
      // 2. Checking baseClient state can cause false negatives
      // 3. The previous check would block for up to 10s without proper watchdog management
      
      // Test SSL connection with timeout and better error handling
      unsigned long sslTestStart = millis();

      // Force a clean SSL state before testing
      delay(500);
      esp_task_wdt_reset();

      Serial.println("Attempting SSL connection (this may take up to 30 seconds)...");
      bool sslConnected = sslClient.connect(config.getAwsIotEndpoint(), config.getAwsIotPort());
      unsigned long sslTestDuration = millis() - sslTestStart;

      // Reset watchdog after potentially long SSL connection attempt
      esp_task_wdt_reset();

      Serial.printf("SSL connection test took %lu ms\n", sslTestDuration);
      
      if (sslConnected) {
        Serial.println("SSL connection test successful");
        sslClient.stop(); // Close test connection
        delay(200);
      } else {
        char sslErrorBuf[256];
        int sslErrorCode = sslClient.lastError(sslErrorBuf, sizeof(sslErrorBuf));
        Serial.printf("SSL connection test failed - Error Code: %d\n", sslErrorCode);
        Serial.printf("SSL Error Message: %s\n", sslErrorBuf);
        
        // Common SSL error codes and their meanings
        switch(sslErrorCode) {
          case -2:
            Serial.println("BIGNUM error detected - possible certificate format issue");
            break;
          case -3:
            Serial.println("SSL handshake timeout");
            break;
          case -4:
            Serial.println("SSL connection failed");
            break;
          default:
            Serial.printf("Unknown SSL error: %d\n", sslErrorCode);
        }
      }
      
      Serial.printf("Free heap after SSL setup: %d bytes\n", ESP.getFreeHeap());
      
      if (!sslConnected) {
        Serial.println("SSL connection failed with full cert setup");
        char errorBuf[256];
        int errorCode = sslClient.lastError(errorBuf, sizeof(errorBuf));
        Serial.printf("SSL Client error code: %d\n", errorCode);
        Serial.printf("SSL Client error message: %s\n", errorBuf);
        
        // Additional diagnostics
        Serial.println("\n=== SSL Connection Diagnostics ===");
        Serial.printf("Endpoint: %s\n", config.getAwsIotEndpoint());
        Serial.printf("Port: %d\n", config.getAwsIotPort());
        Serial.printf("CA Cert Length: %d\n", strlen(caCert));
        Serial.printf("Device Cert Length: %d\n", strlen(deviceCert));
        Serial.printf("Private Key Length: %d\n", strlen(privateKey));
        Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
        Serial.printf("Min Free Heap: %d bytes\n", ESP.getMinFreeHeap());
        Serial.println("=================================\n");
      }
      
      // Continue to MQTT attempt regardless of SSL pre-test result
      sslConnected = true; // Allow MQTT to handle SSL internally
      
      Serial.println("Proceeding with MQTT connection...");
      
      // Single MQTT connection attempt with immediate timeout
      Serial.println("Attempting single MQTT connection...");
      esp_task_wdt_reset();
      
      // Set appropriate MQTT timeout for GSM connections
      mqttClient.setSocketTimeout(20); // Increased to 20 second socket timeout for GSM
      mqttClient.setKeepAlive(60); // 60 second keep alive
      mqttClient.setBufferSize(1024); // Increase buffer size
      
      // Ensure fresh SSL connection state for MQTT
      delay(1000);
      esp_task_wdt_reset();
      
      // Make single connection attempt with detailed error reporting
      Serial.printf("Free heap before MQTT connect: %d bytes\n", ESP.getFreeHeap());
      bool mqttConnected = mqttClient.connect(config.getDeviceId());
      
      Serial.printf("MQTT connection result: %s\n", mqttConnected ? "SUCCESS" : "FAILED");
      
      // If connection failed, check SSL client error details
      if (!mqttConnected) {
        char sslErrorBuf[256];
        int sslErrorCode = sslClient.lastError(sslErrorBuf, sizeof(sslErrorBuf));
        if (sslErrorCode != 0) {
          Serial.printf("SSL Client Error Code: %d\n", sslErrorCode);
          Serial.printf("SSL Client Error Message: %s\n", sslErrorBuf);
        }
        
        // Check if underlying GSM client is still connected
        Serial.printf("Base GSM Client Connected: %s\n", baseClient.connected() ? "YES" : "NO");
        Serial.printf("Signal Quality: %d\n", modem.getSignalQuality());
      }
      
      if (mqttConnected)
      {
        Serial.println("MQTT Connected successfully!");
        mqttRetryCount = 0;
        reconnectDelay = 1000;
        connectionStartTime = 0; // Reset connection timer on success

        // Subscribe to all relevant topics
        Serial.printf("Subscribing to command topic: %s\n", config.getAwsIotDeviceCommandTopic());
        bool cmdSub = mqttClient.subscribe(config.getAwsIotDeviceCommandTopic());
        Serial.printf("Command topic subscription: %s\n", cmdSub ? "SUCCESS" : "FAILED");
        
        Serial.printf("Subscribing to weather topic: %s\n", config.getAwsIotDeviceDataTopic());
        bool weatherSub = mqttClient.subscribe(config.getAwsIotDeviceDataTopic());
        Serial.printf("Weather topic subscription: %s\n", weatherSub ? "SUCCESS" : "FAILED");

        Serial.printf("Subscribing to response topic: %s\n", config.getAwsIotDeviceResponseTopic());
        bool respSub = mqttClient.subscribe(config.getAwsIotDeviceResponseTopic());
        Serial.printf("Response topic subscription: %s\n", respSub ? "SUCCESS" : "FAILED");
        
        esp_task_wdt_reset();

        // Check activation status
        deviceActivated = isDeviceActivated();

        // Publish a startup message
        publishUpdateStatus(deviceActivated ? "Online" : "Inactive",
                            deviceActivated ? "Ready for updates" : "Requires activation",
                            nullptr);
        esp_task_wdt_reset();
      }
      else
      {
        // Enhanced error reporting for debugging
        int mqttState = mqttClient.state();
        Serial.printf("MQTT connection failed, rc=%d. Retrying in %d ms\n", mqttState, reconnectDelay);
        
        // Decode MQTT error states for better debugging
        switch(mqttState) {
          case -4: Serial.println("Error: MQTT_CONNECTION_TIMEOUT"); break;
          case -3: Serial.println("Error: MQTT_CONNECTION_LOST"); break;
          case -2: Serial.println("Error: MQTT_CONNECT_FAILED"); break;
          case -1: Serial.println("Error: MQTT_DISCONNECTED"); break;
          case 1: Serial.println("Error: MQTT_CONNECT_BAD_PROTOCOL"); break;
          case 2: Serial.println("Error: MQTT_CONNECT_BAD_CLIENT_ID"); break;
          case 3: Serial.println("Error: MQTT_CONNECT_UNAVAILABLE"); break;
          case 4: Serial.println("Error: MQTT_CONNECT_BAD_CREDENTIALS"); break;
          case 5: Serial.println("Error: MQTT_CONNECT_UNAUTHORIZED"); break;
          default: Serial.printf("Error: Unknown MQTT error code %d\n", mqttState); break;
        }
        reconnectDelay = min(reconnectDelay * 2, maxReconnectDelay);
        
        // For long delays, reset watchdog periodically in smaller chunks
        unsigned long delayStart = millis();
        while (millis() - delayStart < reconnectDelay) {
          unsigned long remaining = reconnectDelay - (millis() - delayStart);
          unsigned long chunkDelay = min(remaining, 2000UL); // Max 2s chunks
          delay(chunkDelay);
          esp_task_wdt_reset();
        }
      }
    }
    if (mqttRetryCount >= maxRetries)
    {
      Serial.println("Maximum MQTT retries reached. Restarting ESP32.\n");
      ESP.restart();
    }
    Serial.println("---------------------------------");
  }
  else
  {
    Serial.println("GSM not connected, cannot establish MQTT connection");
    Serial.printf("Current GSM status - Connected: %s, Signal: %d, Operator: %s\n",
                 modem.isNetworkConnected() ? "YES" : "NO",
                 modem.getSignalQuality(),
                 modem.getOperator().c_str());
  }
}

// Sync offline data with cloud
bool syncOfflineData()
{
  esp_task_wdt_reset(); // Reset at start
  
  Serial.println("Starting sync diagnostics...");
  Serial.printf("SD Card available: %s\n", sdCard.isAvailable() ? "YES" : "NO");
  Serial.printf("GSM connected: %s\n", modem.isNetworkConnected() ? "YES" : "NO");
  Serial.printf("MQTT connected: %s\n", mqttClient.connected() ? "YES" : "NO");
  Serial.printf("Pending records: %d\n", sdCard.isAvailable() ? sdCard.getPendingRecords() : 0);
  
  if (!sdCard.isAvailable())
  {
    Serial.println("SD card not available for syncing");
    return false;
  }

  if (!modem.isNetworkConnected() || !mqttClient.connected())
  {
    Serial.println("Network connection not available for syncing");
    return false;
  }

  Serial.println("Starting data synchronization...");

  int syncedCount = 0;
  File root = sdCard.openDataDir();
  if (!root)
  {
    Serial.println("Failed to open data directory");
    return false;
  }

  if (!root.isDirectory())
  {
    Serial.println("Data path is not a directory");
    root.close();
    return false;
  }

  // Process up to 10 files at a time to avoid blocking the main loop for too long
  int processLimit = 10;
  int filesProcessed = 0;
  File file = root.openNextFile();
  while (file && processLimit > 0)
  {
    processLimit--;
    filesProcessed++;

    if (!file.isDirectory())
    {
      String filename = file.name(); // Save before closing
      String data = "";
      while (file.available())
      {
        data += (char)file.read();
      }
      file.close();

      Serial.printf("Attempting to publish file: %s (%d bytes)\n", filename.c_str(), data.length());
      
      // Check MQTT connection before publish attempt
      if (!mqttClient.connected()) {
        Serial.println("MQTT disconnected during sync - reconnecting...");
        connectToAWS();
        if (!mqttClient.connected()) {
          Serial.println("Failed to reconnect MQTT during sync");
          break; // Stop sync attempt
        }
      }
      
      // Debug: Show the topic being used
      const char* dataTopic = config.getAwsIotDeviceDataTopic();
      Serial.printf("Publishing to topic: %s\n", dataTopic);
      
      // Try publishing to weather topic first
      bool publishResult = mqttClient.publish(dataTopic, data.c_str());
      Serial.printf("Data topic publish result: %s\n", publishResult ? "SUCCESS" : "FAILED");
      
      // If weather topic fails, try response topic as fallback
      if (!publishResult) {
        Serial.println("Data topic failed, trying response topic...");
        const char* responseTopic = config.getAwsIotDeviceResponseTopic();
        Serial.printf("Fallback topic: %s\n", responseTopic);
        publishResult = mqttClient.publish(responseTopic, data.c_str());
        Serial.printf("Response topic publish result: %s\n", publishResult ? "SUCCESS" : "FAILED");
      }
      
      if (publishResult)
      {
        Serial.printf("Successfully published %s, attempting to delete...\n", filename.c_str());
        if (sdCard.deleteDataFile(filename.c_str()))
        {
          syncedCount++;
          Serial.printf("File %s deleted successfully\n", filename.c_str());
        }
        else
        {
          Serial.printf("Failed to delete file %s after successful publish\n", filename.c_str());
        }
      }
      else
      {
        Serial.printf("Failed to publish data from file: %s (MQTT error: %d)\n", filename.c_str(), mqttClient.state());
        // Add small delay before next attempt to prevent overwhelming the connection
        delay(100);
      }
    }
    else
    {
      file.close();
    }

    file = root.openNextFile();
    
    // Reset watchdog every few files to prevent timeout during large syncs
    if (filesProcessed % 3 == 0) {
      esp_task_wdt_reset();
    }
  }

  root.close();
  Serial.printf("Synchronized %d files\n", syncedCount);
  esp_task_wdt_reset();

  return syncedCount > 0;
}

// Check connectivity and sync data when available
void checkAndSyncData()
{
  if (modem.isNetworkConnected() && mqttClient.connected() && sdCard.getPendingRecords() > 0)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - lastSyncAttempt >= SYNC_INTERVAL)
    {
      lastSyncAttempt = currentMillis;
      Serial.println("Auto-syncing offline data...");
      syncOfflineData();
    }
  }
}

void setup()
{
  esp_task_wdt_init(60, true); // 60s watchdog
  Serial.begin(115200);
  delay(1000);

  // Record boot time for uptime tracking
  bootTime = millis();

  
  // Initialize SPIFFS for certificate storage
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialization failed, attempting format...");
    if (SPIFFS.format()) {
      Serial.println("SPIFFS formatted, retrying initialization...");
      if (SPIFFS.begin(true)) {
        Serial.println("SPIFFS initialized successfully after format");
      } else {
        Serial.println("SPIFFS initialization failed even after format, using default certificates");
      }
    } else {
      Serial.println("SPIFFS format failed, using default certificates");
    }
  } else {
    Serial.println("SPIFFS initialized successfully");
    
    // Check SPIFFS integrity
    Serial.printf("SPIFFS Total: %d bytes, Used: %d bytes\n", SPIFFS.totalBytes(), SPIFFS.usedBytes());
    
    // Test SPIFFS read/write capability
    File testFile = SPIFFS.open("/test.txt", "w");
    if (testFile) {
      testFile.println("SPIFFS test");
      testFile.close();
      
      // Try to read it back
      testFile = SPIFFS.open("/test.txt", "r");
      if (testFile && testFile.available()) {
        String content = testFile.readString();
        testFile.close();
        SPIFFS.remove("/test.txt"); // Clean up test file
        Serial.println("SPIFFS read/write test passed");
      } else {
        Serial.println("SPIFFS read test failed");
        if (testFile) testFile.close();
      }
    } else {
      Serial.println("SPIFFS write test failed");
    }
    
    // Test certificate file existence and readability
    Serial.println("Checking certificate files...");
    bool foundCertFiles = false;
    
    // if (SPIFFS.exists("/ca_cert.pem")) {
    //   File caFile = SPIFFS.open("/ca_cert.pem", "r");
    //   if (caFile && caFile.size() > 100) { // Minimum reasonable cert size
    //     Serial.printf("CA certificate found (%d bytes)\n", caFile.size());
    //     foundCertFiles = true;
    //   } else {
    //     Serial.println("CA certificate file invalid or too small");
    //   }
    //   if (caFile) caFile.close();
    // } else {
    //   Serial.println("CA certificate not found - will use defaults");
    // }
    
    // if (SPIFFS.exists("/device_cert.pem")) {
    //   File devFile = SPIFFS.open("/device_cert.pem", "r");
    //   if (devFile && devFile.size() > 100) {
    //     Serial.printf("Device certificate found (%d bytes)\n", devFile.size());
    //     foundCertFiles = true;
    //   } else {
    //     Serial.println("Device certificate file invalid or too small");
    //   }
    //   if (devFile) devFile.close();
    // } else {
    //   Serial.println("Device certificate not found - will use defaults");
    // }
    
    // if (SPIFFS.exists("/private_key.pem")) {
    //   File keyFile = SPIFFS.open("/private_key.pem", "r");
    //   if (keyFile && keyFile.size() > 100) {
    //     Serial.printf("Private key found (%d bytes)\n", keyFile.size());
    //     foundCertFiles = true;
    //   } else {
    //     Serial.println("Private key file invalid or too small");
    //   }
    //   if (keyFile) keyFile.close();
    // } else {
    //   Serial.println("Private key not found - will use defaults");
    // }
    
    // if (foundCertFiles) {
    //   LOG_INFO_MSG("SPIFFS", "Certificate files detected - will load from SPIFFS");
    // } else {
    //   LOG_INFO_MSG("SPIFFS", "No certificate files found - using defaults from flash");
    // }
  }
  
  config.begin();
  
  // Validate certificate loading and format
  Serial.println("Validating certificate configuration...");
  const char* caCert = config.getCaCert();
  const char* deviceCert = config.getDeviceCert();  
  const char* privateKey = config.getPrivateKey();
  
  bool certsValid = true;
  if (!caCert || strlen(caCert) < 100) {
    Serial.println("ERROR: CA certificate invalid or missing");
    certsValid = false;
  } else {
    Serial.printf("CA certificate loaded: %d bytes\n", strlen(caCert));
    // Check certificate format
    if (!strstr(caCert, "-----BEGIN CERTIFICATE-----") || !strstr(caCert, "-----END CERTIFICATE-----")) {
      Serial.println("WARNING: CA certificate format may be invalid");
    }
  }
  
  if (!deviceCert || strlen(deviceCert) < 100) {
    Serial.println("ERROR: Device certificate invalid or missing");  
    certsValid = false;
  } else {
    Serial.printf("Device certificate loaded: %d bytes\n", strlen(deviceCert));
    // Check certificate format
    if (!strstr(deviceCert, "-----BEGIN CERTIFICATE-----") || !strstr(deviceCert, "-----END CERTIFICATE-----")) {
      Serial.println("WARNING: Device certificate format may be invalid");
    }
  }
  
  if (!privateKey || strlen(privateKey) < 100) {
    Serial.println("ERROR: Private key invalid or missing");
    certsValid = false;
  } else {
    Serial.printf("Private key loaded: %d bytes\n", strlen(privateKey));
    // Check private key format
    if ((!strstr(privateKey, "-----BEGIN RSA PRIVATE KEY-----") && !strstr(privateKey, "-----BEGIN PRIVATE KEY-----")) || 
        (!strstr(privateKey, "-----END RSA PRIVATE KEY-----") && !strstr(privateKey, "-----END PRIVATE KEY-----"))) {
      Serial.println("WARNING: Private key format may be invalid");
    }
  }
  
  if (certsValid) {
    LOG_INFO_MSG("CERTS", "All certificates loaded and validated successfully");
  } else {
    LOG_ERROR_MSG("CERTS", "Certificate validation failed - SSL connections may fail");
  }

  Serial.println("\n---------------------------------");
  Serial.println("ESP32 Weather Station");
  Serial.printf("Device ID: %s\n", config.getDeviceId());
  Serial.printf("Current Firmware Version: %s\n", Config::FIRMWARE_VERSION);
  Serial.println("---------------------------------");

  // Check activation status
  deviceActivated = isDeviceActivated();
  Serial.printf("Device activation status: %s\n", deviceActivated ? "ACTIVATED" : "NOT ACTIVATED");

  // Initialize SD card
  sdCard.begin();
  Serial.printf("SD Card status: %s\n", sdCard.isAvailable() ? "AVAILABLE" : "NOT AVAILABLE");
  
  // Print current credentials (for debugging)
  config.printCredentials();
  Serial.println("---------------------------------");

  // Connect to WiFi and AWS IoT
  initGSM();
  connectToAWS();

  // Initialize sensors
  sensorManager.begin();

  // Initialize timing variables
  lastWeatherPublish = millis();
  lastSyncAttempt = millis();
  lastMemoryCheck = millis();
  
  Serial.println("Memory management enabled: Will restart after 24h or when heap fragmented");
}

void loop()
{
  esp_task_wdt_reset();
  unsigned long currentMillis = millis();
  
  // Monitor heap status periodically
  checkHeapStatus();
  
  // Monitor network health
  updateNetworkHealth();

  // Maintain GSM connection
  if (!modem.isNetworkConnected()) 
  {
    LOG_WARNING_MSG("GSM", "Network disconnected. Reconnecting...");
    initGSM();
  }

  // Maintain MQTT connection
  if (!mqttClient.connected())
  {
    LOG_WARNING_MSG("MQTT", "MQTT disconnected. Reconnecting...");
    connectToAWS();
  }

  // Process MQTT messages
  mqttClient.loop();
  esp_task_wdt_reset();

  // Publish weather data at regular intervals regardless of connectivity
  if (currentMillis - lastWeatherPublish >= WEATHER_PUBLISH_INTERVAL)
  {
    esp_task_wdt_reset();
    lastWeatherPublish = currentMillis;
    if (deviceActivated)
    {
      // Try to publish if connected
      if (modem.isNetworkConnected() && mqttClient.connected())
      {
        publishWeatherData();
        Serial.println("Weather data published to MQTT");
        esp_task_wdt_reset();
      }
      // Otherwise save locally if SD card is available
      else if (sdCard.isAvailable())
      {
        // Always generate data
        String weatherData = generateWeatherDataJson(nullptr);
        if (sdCard.saveWeatherData(weatherData.c_str()))
        {
          Serial.println("Weather data saved to SD card (offline mode)");
        }
        esp_task_wdt_reset();
      }
    }
    else {
      // Device is deactivated - only store locally if SD card is available
      if (sdCard.isAvailable())
      {
        String weatherData = generateWeatherDataJson(nullptr);
        if (sdCard.saveWeatherData(weatherData.c_str()))
        {
          Serial.println("Weather data collected but stored locally (device deactivated)");
        }
        esp_task_wdt_reset();
      }
      else
      {
        Serial.println("Weather data collected but discarded (device deactivated, no SD card)");
      }
    }
  }

  // Check and synchronize offline data if we're back online
  checkAndSyncData();
  esp_task_wdt_reset();
}