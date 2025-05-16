/*
  Version 2.3.2
  FreeRTOS for Kloudtrack - GSM
  - ESP32 would run different tasks on different cores
*/ 
#include <Arduino.h>
#include <PubSubClient.h>
#include <Update.h>
#include "secrets.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

// MQTT Topics
char AWS_IOT_DEVICE_COMMAND_TOPIC[50];
char AWS_IOT_DEVICE_WEATHER_TOPIC[50];
char AWS_IOT_DEVICE_STATUS_TOPIC[50];

// Device ID
#define DEVICE_ID "KT-DEVICE-12345"

// Current firmware version
#define FIRMWARE_VERSION "2.3.1"

// Weather data parameters
#define WEATHER_PUBLISH_INTERVAL 60000
unsigned long lastWeatherPublish = 0;

// Define task priorities (higher number = higher priority)
#define GSM_TASK_PRIORITY       3
#define MQTT_TASK_PRIORITY      2
#define WEATHER_TASK_PRIORITY   1
#define SD_SYNC_TASK_PRIORITY   1
#define OTA_TASK_PRIORITY       4  // High priority for OTA updates

// Define task stack sizes (in words)
#define GSM_TASK_STACK_SIZE     4096
#define MQTT_TASK_STACK_SIZE    4096
#define WEATHER_TASK_STACK_SIZE 4096
#define SD_SYNC_TASK_STACK_SIZE 4096
#define OTA_TASK_STACK_SIZE     8192  // Larger stack for OTA updates

// Task handles
TaskHandle_t gsmTaskHandle = NULL;
TaskHandle_t mqttTaskHandle = NULL;
TaskHandle_t weatherTaskHandle = NULL;
TaskHandle_t sdSyncTaskHandle = NULL;
TaskHandle_t otaTaskHandle = NULL;

// Event group for system state flags
EventGroupHandle_t systemStateEventGroup = NULL;

// Define event bits
#define GSM_CONNECTED_BIT       (1 << 0)
#define MQTT_CONNECTED_BIT      (1 << 1)
#define SD_CARD_READY_BIT       (1 << 2)
#define DEVICE_ACTIVATED_BIT    (1 << 3)
#define OTA_IN_PROGRESS_BIT     (1 << 4)

// Mutex for protecting shared resources
SemaphoreHandle_t sdCardMutex = NULL;
SemaphoreHandle_t modemMutex = NULL;

// Queue for weather data
QueueHandle_t weatherDataQueue = NULL;

// Weather data ranges
#define MIN_TEMPERATURE 15.0
#define MAX_TEMPERATURE 35.0
#define MIN_HUMIDITY 30.0
#define MAX_HUMIDITY 90.0
#define MIN_PRESSURE 980.0
#define MAX_PRESSURE 1040.0
#define MIN_WIND_SPEED 0.0
#define MAX_WIND_SPEED 20.0

// SD Card pins for A7670G module TF card interface
#define SD_MISO_PIN 2  // MISO pin
#define SD_MOSI_PIN 15 // MOSI pin
#define SD_SCLK_PIN 14 // SCLK pin
#define SD_CS_PIN 13   // CS pin
#define SD_DATA_DIR "/kloudtrack"
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

bool deviceActivated = false;
bool sdCardAvailable = false;
bool gsmConnected = false;
int gsmRetryCount = 0;
int mqttRetryCount = 0;
int maxRetries = 5;
unsigned long reconnectDelay = 1000;
const unsigned long maxReconnectDelay = 60000;
unsigned long lastSyncAttempt = 0;
int pendingRecords = 0;

// Task function declarations
void gsmTask(void *pvParameters);
void mqttTask(void *pvParameters);
void weatherTask(void *pvParameters);
void sdSyncTask(void *pvParameters);
void otaTask(void *pvParameters);

// Function to publish status updates
void publishUpdateStatus(const char *status, const char *message)
{
  StaticJsonDocument<200> doc;
  doc["status"] = status;
  doc["message"] = message;

  Serial.printf("Published status: %s - %s\n", status, message);
}

// Function to check if device is activated - now thread-safe
bool isDeviceActivated()
{
  preferences.begin("kloudtrack", false);
  bool activated = preferences.getBool("activated", false);
  preferences.end();
  return activated;
}

// Function to activate the device - now thread-safe
void activateDevice(bool activate)
{
  preferences.begin("kloudtrack", false);
  preferences.putBool("activated", activate);
  preferences.end();
  deviceActivated = activate;

  // Update the event group bit
  if (activate) {
    xEventGroupSetBits(systemStateEventGroup, DEVICE_ACTIVATED_BIT);
  } else {
    xEventGroupClearBits(systemStateEventGroup, DEVICE_ACTIVATED_BIT);
  }

  // Publish activation status
  StaticJsonDocument<200> doc;
  doc["device_id"] = DEVICE_ID;
  doc["firmware_version"] = FIRMWARE_VERSION;
  doc["activated"] = activate;

  String jsonStr;
  serializeJson(doc, jsonStr);

  // Publish to device-specific activation topic if MQTT is connected
  if (mqttClient.connected()) {
    mqttClient.publish(AWS_IOT_DEVICE_COMMAND_TOPIC, jsonStr.c_str());
  }

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

// Updated OTA update function to work with FreeRTOS
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
  vTaskDelay(pdMS_TO_TICKS(1000));
  
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
    vTaskDelay(pdMS_TO_TICKS(1000 * (retry + 1))); // Exponential backoff
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

  // Significantly increase buffer size (use 4KB buffer)
  const size_t bufferSize = 4096;  // 4KB buffer
  uint8_t *buff = (uint8_t*)heap_caps_malloc(bufferSize, MALLOC_CAP_8BIT);

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
      heap_caps_free(buff);
      Update.abort();
      return false;
    }

    // Reset task watchdog to prevent timeouts during long download
    vTaskDelay(1);
    
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
          heap_caps_free(buff);
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
      }
    } else if (available == 0) {
      // No data available, give the GSM modem time to process
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  
  heap_caps_free(buff); // Free the buffer
  
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
  vTaskDelay(pdMS_TO_TICKS(1000));
  
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
  String newVersion = doc.containsKey("version") ? doc["version"].as<String>() : FIRMWARE_VERSION;
  bool forceUpdate = doc.containsKey("force") ? doc["force"].as<bool>() : false;

  if (!forceUpdate && newVersion == FIRMWARE_VERSION) {
    publishUpdateStatus("Ignored", "Same firmware version. Update skipped.");
    return;
  }

  publishUpdateStatus("Starting", "Beginning OTA update...");
  performOTAUpdate(url.c_str());
}

// Generate weather data JSON string
String generateWeatherDataJson()
{
  // Generate random weather data
  float temperature = random(MIN_TEMPERATURE * 100, MAX_TEMPERATURE * 100) / 100.0;
  float humidity = random(MIN_HUMIDITY * 100, MAX_HUMIDITY * 100) / 100.0;
  float pressure = random(MIN_PRESSURE * 10, MAX_PRESSURE * 10) / 10.0;
  float windSpeed = random(MIN_WIND_SPEED * 100, MAX_WIND_SPEED * 100) / 100.0;

  // Get random wind direction (N, NE, E, SE, S, SW, W, NW)
  const char *windDirections[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  const char *windDirection = windDirections[random(0, 8)];

  // Get random weather condition
  const char *weatherConditions[] = {"Sunny", "Partly Cloudy", "Cloudy", "Rainy", "Thunderstorm", "Foggy", "Snowy"};
  const char *weatherCondition = weatherConditions[random(0, 7)];

  // Create JSON document
  StaticJsonDocument<256> doc;
  doc["device_id"] = DEVICE_ID;
  doc["timestamp"] = millis();

  // Weather data
  JsonObject weather = doc.createNestedObject("weather");
  weather["temperature"] = temperature;
  weather["humidity"] = humidity;
  weather["pressure"] = pressure;
  weather["wind_speed"] = windSpeed;
  weather["wind_direction"] = windDirection;
  weather["condition"] = weatherCondition;

  // Serialize to JSON
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

// Function to generate status report
void publishStatusReport()
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
  const char* SD_CARD = sdCardAvailable ? "Available" : "Not available";
  int PENDING_RECORDS;
  if (sdCardAvailable)
  {
    PENDING_RECORDS = pendingRecords;
  }

  StaticJsonDocument<256> doc;
  doc["device_id"] = DEVICE_ID;
  doc["firmware_version"] = FIRMWARE_VERSION;
  doc["activated"] = deviceActivated;
  doc["ram_usage"] = RAM_USAGE;
  doc["flash_usage"] = FLASH_USAGE;
  doc["signal_quality"] = SIGNAL_QUALITY;
  doc["sd_card"] = SD_CARD;
  doc["pending_records"] = PENDING_RECORDS;

  String jsonStr;
  serializeJson(doc, jsonStr);

  // Publish to device-specific status topic
  mqttClient.publish(AWS_IOT_DEVICE_STATUS_TOPIC, jsonStr.c_str());
}

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
  if (strcmp(topic, AWS_IOT_DEVICE_STATUS_TOPIC) == 0)
  {
    // Check for status request
    if (doc.containsKey("status") && doc["status"].as<bool>())
    {
      // Respond with current status when requested
      Serial.println("Status command received.");
      publishStatusReport();
    }
  } 
  else if (strcmp(topic, AWS_IOT_DEVICE_COMMAND_TOPIC) == 0)
  {
    // Handle device reset
    if (doc.containsKey("reset") && doc["reset"].as<bool>())
    {
      // Publish notification that device is restarting
      publishUpdateStatus("Resetting", "Device restarting per command request");
      Serial.println("Reset command received. Restarting device...");

      // Give time for the MQTT message to be sent
      vTaskDelay(pdMS_TO_TICKS(1000));

      // Restart the ESP32
      ESP.restart();
    }
    // Handle device activation
    else if (doc.containsKey("activate"))
    {
      bool activateState = doc["activate"].as<bool>();

      // Set activation state based on the boolean value
      activateDevice(activateState);

      if (activateState)
      {
        Serial.println("Device activated successfully");
        publishUpdateStatus("Activated", "Device activated successfully");
        xEventGroupSetBits(systemStateEventGroup, DEVICE_ACTIVATED_BIT);
      }
      else
      {
        Serial.println("Device deactivated");
        publishUpdateStatus("Deactivated", "Device deactivated");
        xEventGroupClearBits(systemStateEventGroup, DEVICE_ACTIVATED_BIT);
      }
    }
    // Handle OTA update command
    else if (doc.containsKey("update") && doc["update"].as<bool>() == true)
    {
      if (doc.containsKey("url"))
      {
        // Check if OTA is already in progress
        EventBits_t bits = xEventGroupGetBits(systemStateEventGroup);
        if ((bits & OTA_IN_PROGRESS_BIT) == OTA_IN_PROGRESS_BIT) {
          publishUpdateStatus("Busy", "OTA update already in progress");
          return;
        }

        // Start OTA process in a separate task
        String url = doc["url"].as<String>();
        String* urlCopy = new String(url);  // Create copy for task parameter
        
        // Check if the OTA task is already created
        if (otaTaskHandle != NULL) {
          // Resume the task with new parameters
          vTaskResume(otaTaskHandle);
        } else {
          // Create a new OTA task
          xTaskCreate(otaTask, "OTA_Task", OTA_TASK_STACK_SIZE, (void*)urlCopy, OTA_TASK_PRIORITY, &otaTaskHandle);
        }
        
        publishUpdateStatus("Starting", "OTA update process initiated");
      }
      else
      {
        publishUpdateStatus("Error", "Missing url for update");
      }
    }
    // Handle force sync command
    else if (doc.containsKey("sync") && doc["sync"].as<bool>() == true)
    {
      // Flag for immediate sync
      lastSyncAttempt = 0;  // This will trigger sync in the SD task
      publishUpdateStatus("Syncing", "Starting offline data synchronization");
    }
  }
  
  Serial.println("---------------------------------");
}

// Modified setupSDCard function
bool setupSDCard()
{
  Serial.println("Initializing SD card...");

  // Configure SPI pins explicitly for this board (A7670G)
  SPI.begin(SD_SCLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  // Initialize SD card with the CS pin
  SD.end();
  if (!SD.begin(SD_CS_PIN))
  {
    Serial.println("SD Card initialization failed!");
    return false;
  }

  Serial.println("SD Card initialized successfully");

  // Create data directory if it doesn't exist
  if (!SD.exists(SD_DATA_DIR))
  {
    if (SD.mkdir(SD_DATA_DIR))
    {
      Serial.printf("Created directory: %s\n", SD_DATA_DIR);
    }
    else
    {
      Serial.printf("Failed to create directory: %s\n", SD_DATA_DIR);
      return false;
    }
  }

  // Count existing data files
  pendingRecords = 0;
  File root = SD.open(SD_DATA_DIR);
  if (root)
  {
    File file = root.openNextFile();
    while (file)
    {
      pendingRecords++;
      file.close();
      file = root.openNextFile();
    }
    root.close();
  }

  Serial.printf("Found %d pending data records\n", pendingRecords);
  return true;
}

// Generate a unique filename for data storage
String getNextDataFilename()
{
  char filename[32];
  sprintf(filename, "%s/data_%lu.json", SD_DATA_DIR, millis());
  return String(filename);
}

// Delete a data file after successful sync
bool deleteDataFile(const String &filename)
{
  if (SD.remove(filename))
  {
    Serial.printf("Deleted file: %s\n", filename.c_str());
    return true;
  }
  else
  {
    Serial.printf("Failed to delete file: %s\n", filename.c_str());
    return false;
  }
}

// Modified saveWeatherDataToSD function to be called with mutex protection
bool saveWeatherDataToSD(const char *jsonData)
{
  // Note: This function assumes the SD card mutex is already held by the caller
  
  if (!sdCardAvailable)
  {
    return false;
  }

  // Generate a filename
  String filename = getNextDataFilename();

  // Save the data
  File dataFile = SD.open(filename, FILE_WRITE);
  if (!dataFile)
  {
    Serial.printf("Failed to open file for writing: %s\n", filename.c_str());
    return false;
  }

  if (dataFile.print(jsonData))
  {
    dataFile.close();
    Serial.printf("Data saved to %s\n", filename.c_str());
    return true;
  }
  else
  {
    dataFile.close();
    Serial.printf("Failed to write to %s\n", filename.c_str());
    return false;
  }
}

void gsmTask(void *pvParameters)
{
  // GSM connection retry parameters
  int retryCount = 0;
  const int maxRetries = 5;
  TickType_t reconnectDelay = pdMS_TO_TICKS(1000);
  const TickType_t maxReconnectDelay = pdMS_TO_TICKS(60000);

  // Initalize GSM
  if (xSemaphoreTake(modemMutex, portMAX_DELAY) == pdTRUE) {
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

    xSemaphoreGive(modemMutex);
  }

  vTaskDelay(pdMS_TO_TICKS(3000));

  // Main task loop
  for (;;) {
    // Check if GSM is connected
    if (xSemaphoreTake(modemMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      bool isConnected = modem.isNetworkConnected();
      xSemaphoreGive(modemMutex);

      if (!isConnected) {
        Serial.println("GSM not connected, attempting to connect...");

        if (xSemaphoreTake(modemMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
          Serial.println("Connecting to GSM network...");
          modem.gprsConnect(APN);

          // Wait for connection with timeout
          TickType_t startTime = xTaskGetTickCount();
          bool connected = false;

          while ((xTaskGetTickCount() - startTime) < pdMS_TO_TICKS(10000)) {
            if (modem.isNetworkConnected()) {
              connected = true;
              break;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            Serial.print(".");
          }

          if (connected) {
            Serial.println("\nGSM connected!");
            retryCount = 0;
            reconnectDelay = pdMS_TO_TICKS(1000);

            // Set the GSM connected bit
            xEventGroupSetBits(systemStateEventGroup, GSM_CONNECTED_BIT);
          } 
          else {
            retryCount++;
            Serial.printf("\nGSM connection failed. Attempt %d/%d\n", retryCount, maxRetries);

            // Clear the GSM connected bit
            xEventGroupClearBits(systemStateEventGroup, GSM_CONNECTED_BIT);

            if (retryCount >= maxRetries) {
              Serial.println("Max retries reached. Waiting before next attempts.");
              retryCount = 0;
              reconnectDelay = maxReconnectDelay;
            }
            else {
              // Exponential backoff
              reconnectDelay = (reconnectDelay * 2 < maxReconnectDelay) ? reconnectDelay * 2 : maxReconnectDelay;
            }
          }

          xSemaphoreGive(modemMutex);
        }

        // Wait before next attempt if needed
        vTaskDelay(reconnectDelay);
      }
      else {
        // Modem is connected, maintain periodic check
        vTaskDelay(pdMS_TO_TICKS(30000)); // Check every 10 seconds
      }
    }
    else {
      // Could not acquire mutex, try again shortly
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void mqttTask(void *pvParameters) {
  int retryCount = 0;
  const int maxRetries = 5;
  TickType_t reconnectDelay = pdMS_TO_TICKS(1000);
  const TickType_t maxReconnectDelay = pdMS_TO_TICKS(60000);

  // Setup MQTT client
  mqttClient.setServer(AWS_IOT_ENDPOINT, AWS_IOT_PORT);
  mqttClient.setCallback(messageHandler);
  
  // Wait for GSM to be ready before attempting MQTT connection
  for (;;) {
    EventBits_t bits = xEventGroupWaitBits(
      systemStateEventGroup,
      GSM_CONNECTED_BIT,
      pdFALSE,   // Don't clear bit
      pdTRUE,    // Wait for all bits
      pdMS_TO_TICKS(10000)
    );
    
    if ((bits & GSM_CONNECTED_BIT) == GSM_CONNECTED_BIT) {
      Serial.println("GSM ready, attempting MQTT connection");
      break;
    } else {
      Serial.println("Waiting for GSM connection...");
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
  
  // Main task loop
  for (;;) {
    // Check if we're connected to MQTT
    if (!mqttClient.connected()) {
      // Clear the MQTT connected bit
      xEventGroupClearBits(systemStateEventGroup, MQTT_CONNECTED_BIT);
      
      if (xSemaphoreTake(modemMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
        bool gsmConnected = modem.isNetworkConnected();
        xSemaphoreGive(modemMutex);
        
        if (gsmConnected) {
          Serial.printf("Connecting to AWS IoT. Attempt %d/%d\n", retryCount + 1, maxRetries);
          
          // Set SSL certificates
          if (!sslClient.connected()) {
            sslClient.setCACert(AWS_CERT_CA);
            sslClient.setCertificate(AWS_CERT_CRT);
            sslClient.setPrivateKey(AWS_CERT_PRIVATE);
          }
          
          if (mqttClient.connect(DEVICE_ID)) {
            Serial.println("Connected to AWS IoT!");
            retryCount = 0;
            reconnectDelay = pdMS_TO_TICKS(1000);
            
            // Subscribe to topics
            mqttClient.subscribe(AWS_IOT_DEVICE_COMMAND_TOPIC);
            mqttClient.subscribe(AWS_IOT_DEVICE_WEATHER_TOPIC);
            mqttClient.subscribe(AWS_IOT_DEVICE_STATUS_TOPIC);
            
            // Set the MQTT connected bit
            xEventGroupSetBits(systemStateEventGroup, MQTT_CONNECTED_BIT);
            
            // Publish a startup message
            publishUpdateStatus(deviceActivated ? "Online" : "Inactive",
                                deviceActivated ? "Ready for updates" : "Requires activation");
          } else {
            retryCount++;
            Serial.printf("Failed to connect to MQTT, rc=%d\n", mqttClient.state());
            
            if (retryCount >= maxRetries) {
              Serial.println("Maximum MQTT retries reached. Will try again later.");
              retryCount = 0;
              // Wait longer before retrying
              vTaskDelay(pdMS_TO_TICKS(60000));
            } else {
              // Exponential backoff
              reconnectDelay = (reconnectDelay * 2 < maxReconnectDelay) ? reconnectDelay * 2 : maxReconnectDelay;
              vTaskDelay(reconnectDelay);
            }
          }
        } else {
          Serial.println("GSM not connected. Cannot establish MQTT connection.");
          vTaskDelay(pdMS_TO_TICKS(5000));
        }
      } else {
        // Could not acquire mutex
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    } else {
      // MQTT is connected, process messages
      mqttClient.loop();
      
      // Check for messages in weather data queue and publish if connected
      String* weatherJson = NULL;
      if (xQueueReceive(weatherDataQueue, &weatherJson, 0) == pdTRUE) {
        if (weatherJson != NULL) {
          bool published = mqttClient.publish(AWS_IOT_DEVICE_WEATHER_TOPIC, weatherJson->c_str());
          if (published) {
            Serial.println("Weather data published to MQTT");
          } else {
            Serial.println("Failed to publish weather data");
            // Put the message back in the queue for retry
            if (xQueueSendToFront(weatherDataQueue, &weatherJson, 0) != pdTRUE) {
              delete weatherJson;  // If queue is full, delete to prevent memory leak
            }
          }
        }
      }
      
      // Short delay before next loop iteration
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

void weatherTask(void *pvParameters)
{
  TickType_t lastWeatherPublishTime = 0;
  const TickType_t weatherPublishInterval = pdMS_TO_TICKS(WEATHER_PUBLISH_INTERVAL);
  
  // Wait a bit before starting to allow other systems to initialize
  vTaskDelay(pdMS_TO_TICKS(5000));
  
  // Main task loop
  for (;;) {
    // Check if it's time to publish weather data
    TickType_t currentTime = xTaskGetTickCount();
    
    if ((currentTime - lastWeatherPublishTime) >= weatherPublishInterval) {
      lastWeatherPublishTime = currentTime;
      
      // Check if device is activated before collecting data
      EventBits_t bits = xEventGroupGetBits(systemStateEventGroup);
      if ((bits & DEVICE_ACTIVATED_BIT) == DEVICE_ACTIVATED_BIT) {
        // Generate the weather data JSON
        String* weatherJsonStr = new String(generateWeatherDataJson());
        
        if (weatherJsonStr != NULL && weatherJsonStr->length() > 0) {
          Serial.println("Weather data generated: " + *weatherJsonStr);
          
          // Check if we have MQTT connection
          bits = xEventGroupGetBits(systemStateEventGroup);
          bool mqttConnected = ((bits & MQTT_CONNECTED_BIT) == MQTT_CONNECTED_BIT);
          
          if (mqttConnected) {
            // Send to queue for MQTT publishing
            if (xQueueSend(weatherDataQueue, &weatherJsonStr, pdMS_TO_TICKS(1000)) != pdTRUE) {
              Serial.println("Failed to add weather data to queue - queue full");
              
              // If can't queue, try to save to SD card
              if ((bits & SD_CARD_READY_BIT) == SD_CARD_READY_BIT) {
                if (xSemaphoreTake(sdCardMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                  if (saveWeatherDataToSD(weatherJsonStr->c_str())) {
                    Serial.println("Weather data saved to SD card (queue full)");
                    pendingRecords++;
                  }
                  xSemaphoreGive(sdCardMutex);
                }
              }
              delete weatherJsonStr;  // Clean up if not queued or saved
            }
          } else {
            // No MQTT connection, save to SD if available
            if ((bits & SD_CARD_READY_BIT) == SD_CARD_READY_BIT) {
              if (xSemaphoreTake(sdCardMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                if (saveWeatherDataToSD(weatherJsonStr->c_str())) {
                  Serial.println("Weather data saved to SD card (offline mode)");
                  pendingRecords++;
                }
                xSemaphoreGive(sdCardMutex);
              }
              delete weatherJsonStr;  // Clean up after saving
            } else {
              Serial.println("No connectivity and SD card not available - data lost");
              delete weatherJsonStr;  // Clean up if not saved
            }
          }
        }
      }
    }
    
    // Delay before next check
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void sdSyncTask(void *pvParameters)
{
  TickType_t lastSyncAttempt = 0;
  const TickType_t syncInterval = pdMS_TO_TICKS(SYNC_INTERVAL);
  
  // Wait for system to stabilize before starting
  vTaskDelay(pdMS_TO_TICKS(10000));
  
  // Main task loop
  for (;;) {
    EventBits_t bits = xEventGroupGetBits(systemStateEventGroup);
    bool sdCardReady = ((bits & SD_CARD_READY_BIT) == SD_CARD_READY_BIT);
    bool mqttConnected = ((bits & MQTT_CONNECTED_BIT) == MQTT_CONNECTED_BIT);
    
    // Check if it's time to attempt sync
    TickType_t currentTime = xTaskGetTickCount();
    
    if (sdCardReady && mqttConnected && pendingRecords > 0 && 
        ((currentTime - lastSyncAttempt) >= syncInterval || lastSyncAttempt == 0)) {
        
      lastSyncAttempt = currentTime;
      Serial.println("Starting offline data synchronization...");
      
      if (xSemaphoreTake(sdCardMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
        int syncedCount = 0;
        File root = SD.open(SD_DATA_DIR);
        
        if (root && root.isDirectory()) {
          // Process up to 5 files at a time to avoid blocking too long
          int processLimit = 5;
          File file = root.openNextFile();
          
          while (file && processLimit > 0) {
            processLimit--;
            
            if (!file.isDirectory()) {
              // Read the file content
              String path = String(SD_DATA_DIR) + "/" + String(file.name());
              String data = "";
              
              while (file.available()) {
                data += (char)file.read();
              }
              file.close();
              
              // Release SD mutex during MQTT publish
              xSemaphoreGive(sdCardMutex);
              
              // Publish the data if MQTT still connected
              if (mqttClient.connected() && mqttClient.publish(AWS_IOT_DEVICE_WEATHER_TOPIC, data.c_str())) {
                // Take mutex again to delete file
                if (xSemaphoreTake(sdCardMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                  if (deleteDataFile(path)) {
                    syncedCount++;
                    pendingRecords--;
                  }
                } else {
                  // If can't get mutex, try delete in next sync cycle
                  break;
                }
              } else {
                // If MQTT failed, take mutex back and exit loop
                if (xSemaphoreTake(sdCardMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
                  break;
                }
                Serial.println("MQTT disconnected during sync");
                break;
              }
            } else {
              file.close();
            }
            
            // Get next file if we still have the mutex
            if (xSemaphoreGetMutexHolder(sdCardMutex) == xTaskGetCurrentTaskHandle()) {
              file = root.openNextFile();
            } else {
              break;
            }
          }
          
          root.close();
        }
        
        // Make sure we release the mutex
        if (xSemaphoreGetMutexHolder(sdCardMutex) == xTaskGetCurrentTaskHandle()) {
          xSemaphoreGive(sdCardMutex);
        }
        
        Serial.printf("Synchronized %d files, %d remaining\n", syncedCount, pendingRecords);
        
        // If there are still records to sync, we'll try again later
        if (syncedCount > 0 && pendingRecords > 0) {
          // Schedule next sync sooner if we're making progress
          lastSyncAttempt = currentTime - (syncInterval / 2);
        }
      }
    } else if (!sdCardReady || pendingRecords == 0) {
      // Nothing to do, check again after a delay
      vTaskDelay(pdMS_TO_TICKS(60000));  // Check every minute
    } else {
      // Either waiting for MQTT connection or waiting for sync interval
      vTaskDelay(pdMS_TO_TICKS(5000));  // Check every 5 seconds
    }
  }
}

void otaTask(void *pvParameters)
{
  // This task is created in suspended state and resumed when an OTA update is required
  String updateUrl = *((String*)pvParameters);
  free(pvParameters);  // Free the URL memory that was passed

  Serial.println("OTA task started");
  
  // Set the OTA in progress bit
  xEventGroupSetBits(systemStateEventGroup, OTA_IN_PROGRESS_BIT);
  
  // Perform the OTA update
  bool updateSuccess = false;
  
  if (xSemaphoreTake(modemMutex, portMAX_DELAY) == pdTRUE) {
    // Ensure we have GSM connectivity
    if (modem.isNetworkConnected()) {
      updateSuccess = performOTAUpdate(updateUrl);
    } else {
      Serial.println("OTA failed - no network connection");
      publishUpdateStatus("Error", "No network connection for OTA");
    }
    xSemaphoreGive(modemMutex);
  }
  
  if (!updateSuccess) {
    // Clear the OTA in progress bit
    xEventGroupClearBits(systemStateEventGroup, OTA_IN_PROGRESS_BIT);
    Serial.println("OTA update failed");
  }
  
  // This task will delete itself when done
  vTaskDelete(NULL);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Configure device-specific topics
  snprintf(AWS_IOT_DEVICE_COMMAND_TOPIC, 50, "kloudtrack/%s/command", DEVICE_ID);
  snprintf(AWS_IOT_DEVICE_WEATHER_TOPIC, 50, "kloudtrack/%s/data", DEVICE_ID);
  snprintf(AWS_IOT_DEVICE_STATUS_TOPIC, 50, "kloudtrack/%s/status", DEVICE_ID);

  Serial.println("\n---------------------------------");
  Serial.println("ESP32 Weather Station");
  Serial.printf("Device ID: %s\n", DEVICE_ID);
  Serial.printf("Current Firmware Version: %s\n", FIRMWARE_VERSION);
  Serial.println("---------------------------------");

  // Check activation status
  deviceActivated = isDeviceActivated();
  Serial.printf("Device activation status: %s\n", deviceActivated ? "ACTIVATED" : "NOT ACTIVATED");

  // Create mutexes
  sdCardMutex = xSemaphoreCreateMutex();
  modemMutex = xSemaphoreCreateMutex();
  
  // Create event group
  systemStateEventGroup = xEventGroupCreate();
  
  // Create queue for weather data (holds up to 10 JSON strings)
  weatherDataQueue = xQueueCreate(10, sizeof(String*));

  // Initialize SD card - with mutex protection
  sdCardAvailable = setupSDCard();
  Serial.printf("SD Card status: %s\n", sdCardAvailable ? "AVAILABLE" : "NOT AVAILABLE");
  
  // Set initial event group bits based on current state
  if (sdCardAvailable) {
    xEventGroupSetBits(systemStateEventGroup, SD_CARD_READY_BIT);
  }
  
  if (deviceActivated) {
    xEventGroupSetBits(systemStateEventGroup, DEVICE_ACTIVATED_BIT);
  }

  // Create tasks
  xTaskCreate(gsmTask, "GSM_Task", GSM_TASK_STACK_SIZE, NULL, GSM_TASK_PRIORITY, &gsmTaskHandle);
  xTaskCreate(mqttTask, "MQTT_Task", MQTT_TASK_STACK_SIZE, NULL, MQTT_TASK_PRIORITY, &mqttTaskHandle);
  xTaskCreate(weatherTask, "Weather_Task", WEATHER_TASK_STACK_SIZE, NULL, WEATHER_TASK_PRIORITY, &weatherTaskHandle);
  xTaskCreate(sdSyncTask, "SD_Sync_Task", SD_SYNC_TASK_STACK_SIZE, NULL, SD_SYNC_TASK_PRIORITY, &sdSyncTaskHandle);
  
  // OTA task is created but suspended until needed
  xTaskCreate(otaTask, "OTA_Task", OTA_TASK_STACK_SIZE, NULL, OTA_TASK_PRIORITY, &otaTaskHandle);
  vTaskSuspend(otaTaskHandle);
}

void loop()
{
  // Just delay to prevent watchdog issues
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}