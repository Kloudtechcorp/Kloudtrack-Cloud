/*
  Version 2.3.2
  Included correct date and time for Kloudtrack - GSM
  - Time is fetched using AT Commands
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

// MQTT Topics
char AWS_IOT_DEVICE_COMMAND_TOPIC[50];
char AWS_IOT_DEVICE_WEATHER_TOPIC[50];

// Device ID
#define DEVICE_ID "KT-DEVICE-12345"

// Current firmware version
#define FIRMWARE_VERSION "2.3.1"

// Weather data parameters
#define WEATHER_PUBLISH_INTERVAL 60000
unsigned long lastWeatherPublish = 0;

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

String response, dateTime, year, month, day, hour, minute, second;
#define TIME_THRESHOLD 70  // Allow up to 70s jump (for 1-min sleep)
String lastValidTime = "";  // Store last valid time
unsigned long lastEpoch = 0;  // Store last valid epoch timestamp

// Functions
void publishUpdateStatus(const char *status, const char *message);
bool isDeviceActivated();
void activateDevice(bool activate);
void parseURL(const String &url, String &host, int &port, String &path);
bool testServerConnection(const String &host, int port);
bool performOTAUpdate(const char *url, const char *expectedChecksum);
void handleUpdateCommand(const JsonDocument &doc);
uint32_t AutoBaud();
unsigned long convertToEpoch(String year, String month, String day, String hour, String minute, String second);
void getTime();
void publishWeatherData();
void publishStatusReport();
void messageHandler(char *topic, byte *payload, unsigned int length);
void connectWiFi();
void connectToAWS();
bool setupSDCard();
bool saveWeatherDataToSD(const char *jsonData);
bool syncOfflineData();
String generateWeatherDataJson();
void checkAndSyncData();
String getNextDataFilename();
bool deleteDataFile(const String &filename);

// Function to publish status updates
void publishUpdateStatus(const char *status, const char *message)
{
  StaticJsonDocument<200> doc;
  doc["status"] = status;
  doc["message"] = message;

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

  // Publish activation status
  StaticJsonDocument<200> doc;
  doc["device_id"] = DEVICE_ID;
  doc["firmware_version"] = FIRMWARE_VERSION;
  doc["activated"] = activate;

  String jsonStr;
  serializeJson(doc, jsonStr);

  // Publish to device-specific activation topic
  mqttClient.publish(AWS_IOT_DEVICE_COMMAND_TOPIC, jsonStr.c_str());

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
  String newVersion = doc.containsKey("version") ? doc["version"].as<String>() : FIRMWARE_VERSION;
  bool forceUpdate = doc.containsKey("force") ? doc["force"].as<bool>() : false;

  if (!forceUpdate && newVersion == FIRMWARE_VERSION) {
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

// Function to convert date/time to UNIX timestamp
unsigned long convertToEpoch(String year, String month, String day, String hour, String minute, String second) {
  struct tm t;
  t.tm_year = year.toInt() + 2000 - 1900;
  t.tm_mon = month.toInt() - 1;
  t.tm_mday = day.toInt();
  t.tm_hour = hour.toInt();
  t.tm_min = minute.toInt();
  t.tm_sec = second.toInt();
  return mktime(&t);
}

// Function to get the time
void getTime() {
  response = "";
  SerialAT.print("AT+CCLK?\r\n");
  delay(100);
  response = SerialAT.readString();
  if (response != "") {
    int startIndex = response.indexOf("+CCLK: \"");
    int endIndex = response.indexOf("\"", startIndex + 8);
    if (startIndex == -1 || endIndex == -1) return;  // Invalid response
    String dateTimeString = response.substring(startIndex + 8, endIndex);

    int dayIndex = dateTimeString.indexOf("/");
    int monthIndex = dateTimeString.indexOf("/", dayIndex + 1);
    int yearIndex = dateTimeString.indexOf(",");

    String year = dateTimeString.substring(0, dayIndex);
    String month = dateTimeString.substring(dayIndex + 1, monthIndex);
    String day = dateTimeString.substring(monthIndex + 1, yearIndex);

    String timeString = dateTimeString.substring(yearIndex + 1);

    int hourIndex = timeString.indexOf(":");
    int minuteIndex = timeString.indexOf(":", hourIndex + 1);

    String hour = timeString.substring(0, hourIndex);
    String minute = timeString.substring(hourIndex + 1, minuteIndex);
    String second = timeString.substring(minuteIndex + 1);

    int plusIndex = second.indexOf("+");
    if (plusIndex != -1) {
      second = second.substring(0, plusIndex);
    }

    // Convert to epoch time
    unsigned long newEpoch = convertToEpoch(year, month, day, hour, minute, second);

    // Filtering: Ignore large jumps
    if (lastEpoch == 0 || abs((long)newEpoch - (long)lastEpoch) <= TIME_THRESHOLD) {  
      lastEpoch = newEpoch;
      lastValidTime = "20" + year + "-" + month + "-" + day + " " + hour + ":" + minute + ":" + second;
      dateTime = lastValidTime;
    } else {
      Serial.print("Time jump detected ("); 
      Serial.print(abs((long)newEpoch - (long)lastEpoch));
      Serial.println("s), ignoring...");  
    }
  }
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

  // Get time
  getTime();

  // Create JSON document
  StaticJsonDocument<256> doc;
  doc["device_id"] = DEVICE_ID;
  doc["recorded_at"] = dateTime;

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

// Function to generate and publish weather data
void publishWeatherData()
{
  String jsonString = generateWeatherDataJson();

  if (modem.isNetworkConnected() && mqttClient.connected())
  {
    // Publish to device-specific weather topic
    mqttClient.publish(AWS_IOT_DEVICE_WEATHER_TOPIC, jsonString.c_str());
  }
  else
  {
    // Save to SD card if WiFi or MQTT is not available
    if (sdCardAvailable)
    {
      if (saveWeatherDataToSD(jsonString.c_str()))
      {
        Serial.println("Weather data saved to SD card (offline mode)");
        pendingRecords++;
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

  getTime();

  StaticJsonDocument<256> doc;
  doc["device_id"] = DEVICE_ID;
  doc["firmware_version"] = FIRMWARE_VERSION;
  doc["activated"] = deviceActivated;
  doc["ram_usage"] = RAM_USAGE;
  doc["flash_usage"] = FLASH_USAGE;
  doc["signal_quality"] = SIGNAL_QUALITY;
  doc["sd_card"] = SD_CARD;
  doc["pending_records"] = PENDING_RECORDS;
  doc["date_time"] = dateTime;

  String jsonStr;
  serializeJson(doc, jsonStr);

  // Publish to device-specific status topic
  mqttClient.publish(AWS_IOT_DEVICE_COMMAND_TOPIC, jsonStr.c_str());
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
  if (strcmp(topic, AWS_IOT_DEVICE_COMMAND_TOPIC) == 0)
  {
    // Handle device reset
    if (doc.containsKey("reset") && doc["reset"].as<bool>())
    {
      // Publish notification that device is restarting
      publishUpdateStatus("Resetting", "Device restarting per command request");

      Serial.println("Reset command received. Restarting device...");

      // Give time for the MQTT message to be sent
      delay(1000);

      // Restart the ESP32
      ESP.restart();
    }
    // Handle status information
    else if (doc.containsKey("status") && doc["status"].as<bool>())
    {
      // Respond with current status when requested
      Serial.println("Status command received.");
      publishStatusReport();
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
      }
      else
      {
        Serial.println("Device deactivated");
        publishUpdateStatus("Deactivated", "Device deactivated");
      }
    }
    // Handle OTA update command
    else if (doc.containsKey("update") && doc["update"].as<bool>() == true)
    {
      if (doc.containsKey("url"))
      {
        // Optional: allow version and force update flags
        handleUpdateCommand(doc);
      }
      else
      {
        publishUpdateStatus("Error", "Missing url for update");
      }
    }
    // Handle force sync command
    else if (doc.containsKey("sync") && doc["sync"].as<bool>() == true)
    {
      if (syncOfflineData())
      {
        publishUpdateStatus("Synced", "Offline data synchronized successfully");
      }
      else
      {
        publishUpdateStatus("Sync Failed", "Failed to synchronize offline data");
      }
    }
  }
  else if (strcmp(topic, AWS_IOT_DEVICE_WEATHER_TOPIC) == 0) {
    if (doc.containsKey("data") && doc["data"].as<bool>())
    {
      // Respond with current status when requested
      Serial.println("Collecting weather data.");
      String weatherData = generateWeatherDataJson();
      publishWeatherData();
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
  modem.gprsConnect(APN);
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
      if (deviceActivated && sdCardAvailable)
      {
        String weatherData = generateWeatherDataJson();
        if (saveWeatherDataToSD(weatherData.c_str()))
        {
          Serial.println("Weather data saved to SD card while offline");
          pendingRecords++;
        }
      }

      // Reset retry counter and implement longer delay
      gsmRetryCount = 0;
      delay(60000); // Wait a minute before trying again
    }
  }
  getTime();
  Serial.println("Date and Time: " + dateTime);
  Serial.println("---------------------------------");
}
  
// Function to connect to AWS IoT
void connectToAWS()
{
  if (modem.isNetworkConnected())
  {
    SerialMon.println("Connecting to AWS IoT...");
    mqttClient.setServer(AWS_IOT_ENDPOINT, AWS_IOT_PORT);
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

      if (mqttClient.connect(DEVICE_ID))
      {
        Serial.println("Connected!");
        mqttRetryCount = 0;
        reconnectDelay = 1000;

        // Subscribe to all relevant topics
        mqttClient.subscribe(AWS_IOT_DEVICE_COMMAND_TOPIC);
        mqttClient.subscribe(AWS_IOT_DEVICE_WEATHER_TOPIC);

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

// Initialize SD card
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

// Save weather data to SD card
bool saveWeatherDataToSD(const char *jsonData)
{
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

// Sync offline data with cloud
bool syncOfflineData()
{
  if (!sdCardAvailable)
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
  File root = SD.open(SD_DATA_DIR);
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
      // Read the file content
      String path = String(SD_DATA_DIR) + "/" + String(file.name());
      String data = "";
      while (file.available())
      {
        data += (char)file.read();
      }
      file.close();

      // Publish the data
      if (mqttClient.publish(AWS_IOT_DEVICE_WEATHER_TOPIC, data.c_str()))
      {
        // Delete the file after successful publish
        if (deleteDataFile(path))
        {
          syncedCount++;
          pendingRecords--;
        }
      }
      else
      {
        Serial.printf("Failed to publish data from file: %s\n", path.c_str());
      }
    }
    else
    {
      file.close();
    }

    // Get next file
    file = root.openNextFile();
  }

  root.close();
  Serial.printf("Synchronized %d files\n", syncedCount);

  return syncedCount > 0;
}

// Check connectivity and sync data when available
void checkAndSyncData()
{
  if (modem.isNetworkConnected() && mqttClient.connected() && pendingRecords > 0)
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

  // Configure device-specific topics
  snprintf(AWS_IOT_DEVICE_COMMAND_TOPIC, 50, "kloudtrack/%s/command", DEVICE_ID);
  snprintf(AWS_IOT_DEVICE_WEATHER_TOPIC, 50, "kloudtrack/%s/data", DEVICE_ID);

  Serial.println("\n---------------------------------");
  Serial.println("ESP32 Weather Station");
  Serial.printf("Device ID: %s\n", DEVICE_ID);
  Serial.printf("Current Firmware Version: %s\n", FIRMWARE_VERSION);
  Serial.println("---------------------------------");

  // Check activation status
  deviceActivated = isDeviceActivated();
  Serial.printf("Device activation status: %s\n", deviceActivated ? "ACTIVATED" : "NOT ACTIVATED");

  // Initialize SD card
  sdCardAvailable = setupSDCard();
  Serial.printf("SD Card status: %s\n", sdCardAvailable ? "AVAILABLE" : "NOT AVAILABLE");
  Serial.println("---------------------------------");

  // Connect to WiFi and AWS IoT
  initGSM();
  connectToAWS();
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
      // Always generate data
      String weatherData = generateWeatherDataJson();

      // Try to publish if connected
      if (modem.isNetworkConnected() && mqttClient.connected())
      {
        publishWeatherData();
        Serial.println("Weather data published to MQTT");
      }
      // Otherwise save locally if SD card is available
      else if (sdCardAvailable)
      {
        if (saveWeatherDataToSD(weatherData.c_str()))
        {
          Serial.println("Weather data saved to SD card (offline mode)");
          pendingRecords++;
        }
      }
    }
  }

  // Check and synchronize offline data if we're back online
  checkAndSyncData();
}