/*
  Version 2.3.4
  - Store credentials in Preferences
*/ 
#include <Arduino.h>
#include <PubSubClient.h>
#include <Update.h>
#include "secrets.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <FS.h>
#include "Config.h"
#include "DateTime.h"
#include "SdCard.h"
#include "SensorManager.h"

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
unsigned long reconnectDelay = 1000;
const unsigned long maxReconnectDelay = 60000;
unsigned long lastSyncAttempt = 0;

DateTime dateTime;

// Functions
void publishUpdateStatus(const char *status, const char *message);
bool isDeviceActivated();
void activateDevice(bool activate);
void parseURL(const String &url, String &host, int &port, String &path);
bool testServerConnection(const String &host, int port);
bool performOTAUpdate(const char *url, const char *expectedChecksum);
void handleUpdateCommand(const JsonDocument &doc);
uint32_t AutoBaud();
void updateDateTime();
String generateSensorStatusJSON();
void publishSensorStatus();
String generateWeatherDataJson();
void publishWeatherData();
String generateStatusInfoJSON();
void publishStatusReport();
void messageHandler(char *topic, byte *payload, unsigned int length);
void connectToAWS();
bool syncOfflineData();
void checkAndSyncData();

// Function to publish status updates
void publishUpdateStatus(const char *status, const char *message)
{
  StaticJsonDocument<200> doc;
  doc["status"] = status;
  doc["message"] = message;

  // Publish to device-specific activation topic
  String jsonStr;
  serializeJson(doc, jsonStr);
  mqttClient.publish(config.getAwsIotDeviceResponseTopic(), jsonStr.c_str());

  Serial.printf("Published status: %s - %s\n", status, message);
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
void activateDevice(bool activate)
{
  preferences.begin("kloudtrack", false);
  preferences.putBool("activated", activate);
  preferences.end();
  deviceActivated = activate;

  updateDateTime();
  // Publish activation status
  StaticJsonDocument<200> doc;
  doc["device_id"] = config.getDeviceId();
  doc["firmware_version"] = Config::FIRMWARE_VERSION;
  doc["activated"] = activate;
  doc["recorded_at"] = dateTime.asStr();

  String jsonStr;
  serializeJson(doc, jsonStr);

  // Publish to device-specific activation topic
  mqttClient.publish(config.getAwsIotDeviceResponseTopic(), jsonStr.c_str());

  Serial.printf("Device %s\n", activate ? "ACTIVATED" : "DEACTIVATED");
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
  sslClient.setCACert(AWS_CERT_CA);
  
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

  publishUpdateStatus("OTA", "Starting OTA update process");

  // Parse the URL
  parseURL(url, host, port, path);
  if (host.length() == 0 || path.length() == 0) {
    Serial.println("[OTA] Invalid OTA URL");
    publishUpdateStatus("Error", "Invalid OTA URL format");
    return false;
  }

  Serial.printf("[OTA] Connecting to: %s:%d\n", host.c_str(), port);
  publishUpdateStatus("OTA", "Connecting to update server");

  // Test server connection before proceeding
  if (!testServerConnection(host, port)) {
    publishUpdateStatus("Error", "Failed to establish connection to update server");
    return false;
  }

  // Reset the SSL client before creating HTTP client to ensure fresh connection
  sslClient.stop();
  delay(1000);
  
  // Set the certificates for the OTA server specifically
  sslClient.setCACert(AWS_CERT_CA);

  HttpClient http(sslClient, host, port);
  http.setTimeout(10000); // 10 seconds timeout

  http.connectionKeepAlive();
  publishUpdateStatus("OTA", "Downloading firmware");
  Serial.println("[OTA] Sending HTTP GET request");

  // Send GET request with retry mechanism
  int maxRetries = 3;
  int err = -1;
  
  for (int retry = 0; retry < maxRetries; retry++) {
    err = http.get(path);
    if (err == 0) break;
    
    Serial.printf("[OTA] HTTP GET failed (attempt %d/%d): %d\n", retry+1, maxRetries, err);
    delay(1000 * (retry + 1)); // Exponential backoff
  }
  
  if (err != 0) {
    String errorMsg = "HTTP request failed: " + String(err);
    publishUpdateStatus("Error", errorMsg.c_str());
    return false;
  }

  int status = http.responseStatusCode();
  Serial.printf("[OTA] HTTP status code: %d\n", status);

  if (status <= 0) {
    String errorMsg = "Invalid HTTP status: " + String(status);
    publishUpdateStatus("Error", errorMsg.c_str());
    return false;
  }
  
  if (status != 200) {
    String errorMsg = "HTTP error: " + String(status);
    publishUpdateStatus("Error", errorMsg.c_str());
    return false;
  }

  int contentLength = http.contentLength();
  if (contentLength <= 0) {
    Serial.println("[OTA] Invalid content length");
    publishUpdateStatus("Error", "Invalid content length");
    return false;
  }

  Serial.printf("[OTA] Update size: %d bytes\n", contentLength);

  if (!Update.begin(contentLength)) {
    Serial.println("[OTA] Update.begin() failed");
    return false;
  }

  Serial.println("[OTA] Starting update...");
  publishUpdateStatus("OTA", "Writing firmware to flash");

  // Significantly increase buffer size (use 4KB or 8KB if memory allows)
  const size_t bufferSize = 4096;  // 4KB buffer instead of 512 bytes
  uint8_t *buff = (uint8_t*)malloc(bufferSize);

  if (!buff) {
    Serial.println("[OTA] Failed to allocate buffer memory");
    publishUpdateStatus("Error", "Memory allocation failed");
    Update.abort();
    return false;
  }

  size_t written = 0;
  uint32_t lastProgress = 0;
  uint32_t startTime = millis();
  
  // Read with timeout protection
  unsigned long readTimeout = 15000; // 15 second timeout for reads
  unsigned long lastRead = millis();
  
  while (http.connected() && written < contentLength) {
    // Check for read timeout
    if (millis() - lastRead > readTimeout) {
      Serial.println("[OTA] Read timeout");
      publishUpdateStatus("Error", "Download timeout");
      free(buff);
      Update.abort();
      return false;
    }

    esp_task_wdt_reset(); // Reset watchdog during long download
    
    int available = http.available();
    if (available > 0) {
      lastRead = millis(); // Reset timeout counter
      
      // Read up to buffer size
      size_t toRead = min(available, (int)bufferSize);
      int readBytes = http.read(buff, toRead);
      
      if (readBytes > 0) {
        // Write to Update
        if (Update.write(buff, readBytes) != readBytes) {
          Serial.printf("[OTA] Write failed: %s\n", Update.errorString());
          free(buff);
          Update.abort();
          return false;
        }
        
        written += readBytes;
        
        // Report progress every 5%
        uint32_t progress = (written * 100) / contentLength;
        if (progress - lastProgress >= 5 || progress == 100) {
          lastProgress = progress;
          Serial.printf("[OTA] Progress: %d%%\n", progress);
        }
        
        // Small delay to allow background tasks (WiFi/GSM stack, etc.)
        delay(1);
      }
    } else if (available == 0) {
      // No data available, give the GSM modem time to process
      delay(10);
    }
  }
  
  free(buff); // Free the buffer
  
  uint32_t updateTime = (millis() - startTime) / 1000;
  Serial.printf("[OTA] Download completed in %d seconds\n", updateTime);
  
  if (written != contentLength) {
    Serial.printf("[OTA] Size mismatch: %d != %d\n", written, contentLength);
    publishUpdateStatus("Error", "Size mismatch in downloaded firmware");
    Update.abort();
    return false;
  }

  Serial.println("[OTA] Finishing update...");
  publishUpdateStatus("OTA", "Finalizing firmware update");
  
  if (!Update.end()) {
    Serial.printf("[OTA] Update.end() failed: %s\n", Update.errorString());
    return false;
  }

  if (!Update.isFinished()) {
    Serial.println("[OTA] Update not finished correctly");
    publishUpdateStatus("Error", "Update process incomplete");
    return false;
  }

  Serial.println("[OTA] Update successful! Rebooting...");
  publishUpdateStatus("Success", "Update complete, rebooting device");
  
  // Add a delay to ensure the final status message is sent
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
    publishUpdateStatus("Rejected", "Device not activated");
    return;
  }

  if (!doc.containsKey("url")) {
    publishUpdateStatus("Error", "Missing url");
    return;
  }

  String url = doc["url"].as<String>();
  String newVersion = doc.containsKey("version") ? doc["version"].as<String>() : "";
  bool forceUpdate = doc.containsKey("force") ? doc["force"].as<bool>() : false;

  if (!forceUpdate && !newVersion.isEmpty() && newVersion == Config::FIRMWARE_VERSION) {
    publishUpdateStatus("Ignored", "Same firmware version. Update skipped.");
    return;
  }

  publishUpdateStatus("Starting", "Beginning OTA update...");
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
String generateSensorStatusJSON()
{
  updateDateTime();
  StaticJsonDocument<256> doc;
  doc["recorded_at"] = dateTime.asStr();

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
  serializeJson(doc, jsonStr);
  return jsonStr;
}

// Function to publish sensor status
void publishSensorStatus()
{
  String jsonString = generateSensorStatusJSON();

  // Publish to device-specific status topic
  mqttClient.publish(config.getAwsIotDeviceResponseTopic(), jsonString.c_str());
}

// Generate weather data JSON string
String generateWeatherDataJson()
{
  // Get time
  updateDateTime();

  // Create JSON document
  StaticJsonDocument<256> doc;
  doc["recorded_at"] = dateTime.asStr();

  // Collect sensor data
  SensorReadings readings = sensorManager.readAllSensors();
  doc["temperature"] = readings.temperature;
  doc["humidity"] = readings.humidity;
  doc["pressure"] = readings.pressure;
  doc["light_intensity"] = readings.lightIntensity;
  doc["uv_index"] = readings.uvIndex;
  doc["wind_direction"] = readings.windDirection;
  doc["wind_speed"] = readings.windSpeed;
  doc["precipitation"] = readings.precipitation;
  doc["in_temp"] = readings.panelTemperature;

  // Serialize to JSON
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

// Function to generate and publish weather data
void publishWeatherData()
{
  String jsonString = generateWeatherDataJson();

  if (modem.isNetworkConnected() && mqttClient.connected())
  {
    // Publish to device-specific weather topic
    mqttClient.publish(config.getAwsIotDeviceWeatherTopic(), jsonString.c_str());
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

// Generate status information JSON string
String generateStatusInfoJSON()
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

  StaticJsonDocument<512> doc;
  doc["rec_at"] = dateTime.asStr();
  doc["device_id"] = config.getDeviceId();
  doc["firmware"] = Config::FIRMWARE_VERSION;
  doc["activated"] = deviceActivated;
  doc["ram"] = RAM_USAGE;
  doc["flash"] = FLASH_USAGE;
  doc["sd"] = SD_CARD;
  doc["pending"] = PENDING_RECORDS;

  // Serialize to JSON
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

// Function to generate status report
void publishStatusReport()
{
  String jsonString = generateStatusInfoJSON();

  // Publish to device-specific status topic
  mqttClient.publish(config.getAwsIotDeviceResponseTopic(), jsonString.c_str());
}

// MQTT message handler
void messageHandler(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message received on topic: ");
  Serial.println(topic);

  // Create a null-terminated string from the payload
  char *payloadStr = new char[length + 1];
  memcpy(payloadStr, payload, length);
  payloadStr[length] = '\0';
  Serial.println(payloadStr);

  // Parse JSON
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payloadStr);
  delete[] payloadStr;

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
    // Handle status information
    if (command && strcmp(command, "info") == 0)
    {
      publishStatusReport();
      Serial.println("Station Information sent.");
      publishUpdateStatus("Status", "Station information sent successfully");
    }
    // Handle sensor status
    else if (command && strcmp(command, "sensor") == 0)
    {
      publishSensorStatus();
      Serial.println("Sensors status sent.");
      publishUpdateStatus("Sensor Status", "Sensors status sent successfully");
    }
    // Handle device activation
    else if (command && strcmp(command, "activate") == 0)
    {
      // Activate the device
      activateDevice(true);
      Serial.printf("Device activated successfully at %s.\n", dateTime.c_str());
      publishUpdateStatus("Activated", "Device activated successfully");
    }
    // Handle device deactivation
    else if (command && strcmp(command, "deactivate") == 0)
    {     
      // Deactivate the device
      activateDevice(false);
      Serial.printf("Device deactivated successfully at %s.\n", dateTime.c_str());
      publishUpdateStatus("Deactivated", "Device deactivated successfully");
    }
    // Commands that require activation
    else if (!deviceActivated) {
      Serial.printf("Command '%s' rejected: Device not activated\n", command ? command : "unknown");
      publishUpdateStatus("Rejected", "Device not activated - command ignored");
    }
    else {
      // Handle device reset
      if (command && strcmp(command, "reset") == 0)
      {
        updateDateTime();
        StaticJsonDocument<128> resetDoc;
        resetDoc["recorded_at"] = dateTime.asStr();
        resetDoc["command"] = "reset";
        String resetJson;
        serializeJson(resetDoc, resetJson);
        mqttClient.publish(config.getAwsIotDeviceCommandTopic(), resetJson.c_str());
        
        // Reset the device
        Serial.printf("Reset command received at %s. Restarting device...\n", dateTime.c_str());
        publishUpdateStatus("Resetting", "Device restarting per command request");
        delay(1000);
        ESP.restart();
      }
      // Handle OTA update command
      else if (command && strcmp(command, "update") == 0)
      {
        if (doc.containsKey("url"))
        {
          updateDateTime();
          StaticJsonDocument<128> updateDoc;
          updateDoc["recorded_at"] = dateTime.asStr();
          updateDoc["command"] = "update";
          updateDoc["status"] = "received";
          updateDoc["url"] = doc["url"].as<String>();
          if (doc.containsKey("version")) {
            updateDoc["version"] = doc["version"].as<String>();
          }
          String updateJson;
          serializeJson(updateDoc, updateJson);
          mqttClient.publish(config.getAwsIotDeviceCommandTopic(), updateJson.c_str());

          handleUpdateCommand(doc);
          Serial.printf("OTA update command processed at %s.\n", dateTime.c_str());
          publishUpdateStatus("OTA Update", "OTA update command processed");
        }
        else
        {
          Serial.printf("Error: Missing URL for OTA update at %s.\n", dateTime.c_str());
          publishUpdateStatus("Error", "Missing url for update");
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
          StaticJsonDocument<128> syncDoc;
          syncDoc["recorded_at"] = dateTime.asStr();
          syncDoc["command"] = "sync";
          String syncJson;
          serializeJson(syncDoc, syncJson);
          mqttClient.publish(config.getAwsIotDeviceCommandTopic(), syncJson.c_str());

          // Force sync command received
          Serial.printf("Force sync command received at %s.", dateTime.c_str());
          publishUpdateStatus("Sync", "Force sync command received");
          
          if (syncOfflineData())
          {
            Serial.println("Offline data synchronized successfully");
            publishUpdateStatus("Synced", "Offline data synchronized successfully");
          }
          else
          {
            Serial.println("Failed to synchronize offline data");
            publishUpdateStatus("Sync Failed", "Failed to synchronize offline data");
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
        String jsonString = generateWeatherDataJson();
        mqttClient.publish(config.getAwsIotDeviceCommandTopic(), jsonString.c_str());
        publishUpdateStatus("Weather Data", "Current weather data sent");
      }
      // Handle credential management commands
      else if (command && strcmp(command, "set_wifi") == 0)
      {
        if (doc.containsKey("ssid") && doc.containsKey("password")) {
          String ssid = doc["ssid"].as<String>();
          String password = doc["password"].as<String>();
          
          if (config.setWifiCredentials(ssid.c_str(), password.c_str())) {
            Serial.printf("WiFi credentials updated: %s\n", ssid.c_str());
            publishUpdateStatus("WiFi Updated", "WiFi credentials updated successfully");
          } else {
            Serial.println("Failed to update WiFi credentials");
            publishUpdateStatus("WiFi Error", "Failed to update WiFi credentials");
          }
        } else {
          Serial.println("Missing SSID or password for WiFi update");
          publishUpdateStatus("WiFi Error", "Missing SSID or password");
        }
      }
      else if (command && strcmp(command, "set_gsm") == 0)
      {
        if (doc.containsKey("apn")) {
          String apn = doc["apn"].as<String>();
          
          if (config.setGsmCredentials(apn.c_str())) {
            Serial.printf("GSM APN updated: %s\n", apn.c_str());
            publishUpdateStatus("GSM Updated", "GSM APN updated successfully");
          } else {
            Serial.println("Failed to update GSM APN");
            publishUpdateStatus("GSM Error", "Failed to update GSM APN");
          }
        } else {
          Serial.println("Missing APN for GSM update");
          publishUpdateStatus("GSM Error", "Missing APN");
        }
      }
      else if (command && strcmp(command, "set_aws") == 0)
      {
        if (doc.containsKey("endpoint") && doc.containsKey("port")) {
          String endpoint = doc["endpoint"].as<String>();
          int port = doc["port"].as<int>();
          
          if (config.setAwsCredentials(endpoint.c_str(), port)) {
            Serial.printf("AWS credentials updated: %s:%d\n", endpoint.c_str(), port);
            publishUpdateStatus("AWS Updated", "AWS credentials updated successfully");
          } else {
            Serial.println("Failed to update AWS credentials");
            publishUpdateStatus("AWS Error", "Failed to update AWS credentials");
          }
        } else {
          Serial.println("Missing endpoint or port for AWS update");
          publishUpdateStatus("AWS Error", "Missing endpoint or port");
        }
      }
      else if (command && strcmp(command, "get_credentials") == 0)
      {
        // Send current credentials status
        StaticJsonDocument<256> credDoc;
        credDoc["command"] = "credentials_status";
        credDoc["wifi_configured"] = config.hasWifiCredentials();
        credDoc["gsm_configured"] = config.hasGsmCredentials();
        credDoc["aws_configured"] = config.hasAwsCredentials();
        credDoc["wifi_ssid"] = config.getWifiSsid();
        credDoc["gsm_apn"] = config.getApn();
        credDoc["aws_endpoint"] = config.getAwsIotEndpoint();
        credDoc["aws_port"] = config.getAwsIotPort();
        
        String credJson;
        serializeJson(credDoc, credJson);
        mqttClient.publish(config.getAwsIotDeviceCommandTopic(), credJson.c_str());
        publishUpdateStatus("Credentials", "Current credentials status sent");
      }
      else if (command && strcmp(command, "clear_credentials") == 0)
      {
        config.clearCredentials();
        Serial.println("All credentials cleared, using defaults");
        publishUpdateStatus("Credentials Cleared", "All credentials cleared, using defaults");
      }
    }   
  }
  Serial.println("---------------------------------");
}

// Function to initialize GSM modem
void initGSM() {
  SerialMon.println("Initializing GSM modem...");
  // A7670-GSM Reset
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW); delay(100);
  digitalWrite(RESET, HIGH); delay(100);
  digitalWrite(RESET, LOW); delay(100);

  // A7670-GSM Power
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW); delay(100);
  digitalWrite(PWR_PIN, HIGH); delay(100);
  digitalWrite(PWR_PIN, LOW); delay(1000);  // Increased delay

  Serial.println("Starting Serial Communications...");
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  modem.init();
  AutoBaud();
  delay(2000); // Wait for the modem to initialize

  Serial.println("Connecting to cellular network...");
  modem.gprsConnect(config.getApn());
  unsigned long gsmStart = millis();
  while (!modem.isNetworkConnected() && (millis() - gsmStart < 10000))
  {
    Serial.print(".");
    delay(1000);
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

    // Instead of restarting after maxRetries, collect and store data locally
    if (gsmRetryCount >= maxRetries)
    {
      Serial.println("Maximum GSM retries reached. Continuing in offline mode.");

      // Collect weather data before attempting further reconnections
      if (deviceActivated && sdCard.isAvailable())
      {
        String weatherData = generateWeatherDataJson();
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
  if (modem.isNetworkConnected())
  {
    SerialMon.println("Connecting to AWS IoT...");
    mqttClient.setServer(config.getAwsIotEndpoint(), config.getAwsIotPort());
    mqttClient.setCallback(messageHandler);

    if (!sslClient.connected()) {
      // Only set certs if SSL connection is fresh
      sslClient.setCACert(AWS_CERT_CA);
      sslClient.setCertificate(AWS_CERT_CRT);
      sslClient.setPrivateKey(AWS_CERT_PRIVATE);
    }

    if (!mqttClient.connected())
    {
      mqttRetryCount++;
      Serial.printf("Reconnecting attempt %d/%d\n", mqttRetryCount, maxRetries);

      if (mqttClient.connect(config.getDeviceId()))
      {
        Serial.println("Connected!");
        mqttRetryCount = 0;
        reconnectDelay = 1000;

        // Subscribe to all relevant topics
        mqttClient.subscribe(config.getAwsIotDeviceCommandTopic());
        mqttClient.subscribe(config.getAwsIotDeviceWeatherTopic());

        // Check activation status
        deviceActivated = isDeviceActivated();

        // Publish a startup message
        publishUpdateStatus(deviceActivated ? "Online" : "Inactive",
                            deviceActivated ? "Ready for updates" : "Requires activation");
      }
      else
      {
        // Exponential backoff
        // Serial.printf("Failed, rc=%d. Retrying in %d ms\n", mqttClient.state(), reconnectDelay);
        reconnectDelay = min(reconnectDelay * 2, maxReconnectDelay);
        delay(reconnectDelay);
      }
    }
    if (mqttRetryCount >= maxRetries)
    {
      Serial.println("Maximum MQTT retries reached. Restarting ESP32.\n");
      ESP.restart();
    }
    Serial.println("---------------------------------");
  }
}

// Sync offline data with cloud
bool syncOfflineData()
{
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
  File file = root.openNextFile();
  while (file && processLimit > 0)
  {
    processLimit--;

    if (!file.isDirectory())
    {
      String filename = file.name(); // Save before closing
      String data = "";
      while (file.available())
      {
        data += (char)file.read();
      }
      file.close();

      if (mqttClient.publish(config.getAwsIotDeviceWeatherTopic(), data.c_str()))
      {
        if (sdCard.deleteDataFile(filename.c_str()))
        {
          syncedCount++;
        }
      }
      else
      {
          Serial.printf("Failed to publish data from file: %s\n", filename.c_str());
      }
    }
    else
    {
      file.close();
    }

    file = root.openNextFile();
  }

  root.close();
  Serial.printf("Synchronized %d files\n", syncedCount);

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
  esp_task_wdt_init(60, true); // 30s watchdog
  Serial.begin(115200);
  delay(1000);

  config.begin();

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

  // Collecting last time
  lastWeatherPublish = millis();
  lastSyncAttempt = millis();
}

void loop()
{
  esp_task_wdt_reset();
  unsigned long currentMillis = millis();

  // Maintain GSM connection
  if (!modem.isNetworkConnected()) 
  {
    Serial.println("Network disconnected. Reconnecting...");
    initGSM();
  }

  // Maintain MQTT connection
  if (!mqttClient.connected())
  {
    Serial.println("MQTT disconnected. Reconnecting...");
    connectToAWS();
  }

  // Process MQTT messages
  mqttClient.loop();

  // Publish weather data at regular intervals regardless of connectivity
  if (currentMillis - lastWeatherPublish >= WEATHER_PUBLISH_INTERVAL)
  {
    lastWeatherPublish = currentMillis;
    if (deviceActivated)
    {
      // Try to publish if connected
      if (modem.isNetworkConnected() && mqttClient.connected())
      {
        publishWeatherData();
        Serial.println("Weather data published to MQTT");
      }
      // Otherwise save locally if SD card is available
      else if (sdCard.isAvailable())
      {
        // Always generate data
        String weatherData = generateWeatherDataJson();
        if (sdCard.saveWeatherData(weatherData.c_str()))
        {
          Serial.println("Weather data saved to SD card (offline mode)");
        }
      }
    }
    else {
      // Device is deactivated - only store locally if SD card is available
      if (sdCard.isAvailable())
      {
        String weatherData = generateWeatherDataJson();
        if (sdCard.saveWeatherData(weatherData.c_str()))
        {
          Serial.println("Weather data collected but stored locally (device deactivated)");
        }
      }
      else
      {
        Serial.println("Weather data collected but discarded (device deactivated, no SD card)");
      }
    }
  }

  // Check and synchronize offline data if we're back online
  checkAndSyncData();
}