#include "WifiClient.h"

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <Update.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "secrets.h"

// Weather data ranges
#define MIN_TEMPERATURE 15.0
#define MAX_TEMPERATURE 35.0
#define MIN_HUMIDITY 30.0
#define MAX_HUMIDITY 90.0
#define MIN_PRESSURE 980.0
#define MAX_PRESSURE 1040.0
#define MIN_WIND_SPEED 0.0
#define MAX_WIND_SPEED 20.0

void WifiClient::connectWifi()
{
  if (WiFi.isConnected())
  {
    return;
  }

  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long wifiStart = millis();

  while (!WiFi.isConnected() && (millis() - wifiStart < 10000))
  {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.isConnected())
  {
    Serial.println("\nWiFi connected!");
    _wifiRetryCount = 0;
  }
  else
  {
    _wifiRetryCount++;
    Serial.printf("\nWiFi connection failed. Attempt %d/%d\n", _wifiRetryCount, MAX_CONNECT_RETRIES);

    // Instead of restarting after maxRetries, collect and store data locally
    if (_wifiRetryCount >= MAX_CONNECT_RETRIES)
    {
      Serial.println("Maximum WiFi retries reached. Continuing in offline mode.");

      // Collect weather data before attempting further reconnections
      if (_isDeviceActivated && _sdCard.isAvailable())
      {
        String weatherData = generateWeatherDataJson();
        if (_sdCard.saveWeatherData(weatherData.c_str()))
        {
          Serial.println("Weather data saved to SD card while offline");
        }
      }

      // Reset retry counter and implement longer delay
      _wifiRetryCount = 0;
      delay(60000); // Wait a minute before trying again
    }
  }

  updateDateTime();
  Serial.println("Date and Time: " + _dateTime.asStr());
  Serial.println("---------------------------------");
}

void WifiClient::connectToAws()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("Connecting to AWS IoT...");
    // Connect to the MQTT broker on AWS IoT
    _mqttClient.setServer(AWS_IOT_ENDPOINT, 8883);
    _mqttClient.setCallback([this](char *topic, byte *payload, unsigned int length)
                            { handleMqttMessage(topic, payload, length); });

    // Configure WiFiClientSecure to use the AWS IoT device credentials
    if (!_wifiClient.connected())
    {
      _wifiClient.setCACert(AWS_CERT_CA);
      _wifiClient.setCertificate(AWS_CERT_CRT);
      _wifiClient.setPrivateKey(AWS_CERT_PRIVATE);
    }

    if (!_mqttClient.connected())
    {
      _mqttRetryCount++;
      Serial.printf("Connecting to AWS IoT. Attempt %d/%d\n", _mqttRetryCount, MAX_CONNECT_RETRIES);

      if (_mqttClient.connect(_config.getDeviceId()))
      {
        Serial.println("Connected!");
        _mqttRetryCount = 0;
        _reconnectDelay = 1000;

        // Subscribe to all relevant topics
        _mqttClient.subscribe(_config.getAwsIotDeviceCommandTopic());
        _mqttClient.subscribe(_config.getAwsIotDeviceWeatherTopic());

        // Check activation status
        _isDeviceActivated = isDeviceActivated();

        // Publish a startup message
        publishUpdateStatus(_isDeviceActivated ? "Online" : "Inactive",
                            _isDeviceActivated ? "Ready for updates" : "Requires activation");
      }
      else
      {
        // Exponential backoff
        // Serial.printf("Failed, rc=%d. Retrying in %d ms\n", mqttClient.state(), reconnectDelay);
        _reconnectDelay = min(_reconnectDelay * 2, MAX_RECONNECT_DELAY);
        delay(_reconnectDelay);
      }
    }
    if (_mqttRetryCount >= MAX_CONNECT_RETRIES)
    {
      Serial.println("Maximum MQTT retries reached. Restarting ESP32.\n");
      ESP.restart();
    }
    Serial.println("---------------------------------");
  }
}

// MQTT message handler
void WifiClient::handleMqttMessage(char *topic, byte *payload, unsigned int length)
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
  if (strcmp(topic, _config.getAwsIotDeviceCommandTopic()) == 0)
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
      if (doc.containsKey("otaUrl"))
      {
        // Optional: allow version and force update flags
        handleUpdateCommand(doc);
      }
      else
      {
        publishUpdateStatus("Error", "Missing otaUrl for update");
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
  Serial.println("---------------------------------");
}

// Handle update commands
void WifiClient::handleUpdateCommand(const JsonDocument &doc)
{
  // Check if device is activated before proceeding
  if (!_isDeviceActivated)
  {
    Serial.println("Update command rejected: Device not activated");
    publishUpdateStatus("Rejected", "Device not activated");
    return;
  }

  if (!doc.containsKey("url"))
  {
    publishUpdateStatus("Error", "Missing url");
    return;
  }

  String url = doc["url"].as<String>();
  String newVersion = doc.containsKey("version") ? doc["version"].as<String>() : Config::FIRMWARE_VERSION;
  bool forceUpdate = doc.containsKey("force") ? doc["force"].as<bool>() : false;

  // Check if we need to update
  if (!forceUpdate && newVersion == Config::FIRMWARE_VERSION)
  {
    publishUpdateStatus("Ignored", "Same firmware version. Update skipped.");
    return;
  }

  // Compare versions
  if (strcmp(newVersion.c_str(), Config::FIRMWARE_VERSION) == 0 && !forceUpdate)
  {
    Serial.println("Already running this version, update skipped");
    publishUpdateStatus("Skipped", "Already running the requested version");
    return;
  }

  Serial.printf("Starting update to version %s from %s\n", newVersion, url);
  publishUpdateStatus("Started", "Starting firmware update");

  // Perform the update
  if (performOTAUpdate(url))
  {
    ESP.restart();
  }
}

// Function to perform OTA update
bool WifiClient::performOTAUpdate(const String &url)
{
  // Check if device is activated
  if (!_isDeviceActivated)
  {
    Serial.println("OTA update rejected: Device not activated");
    publishUpdateStatus("Rejected", "Device not activated");
    return false;
  }

  HTTPClient http;
  bool success = false;

  // Start the OTA update process
  Serial.println("Attempting to download firmware...");
  publishUpdateStatus("Downloading", "Starting firmware download");

  http.begin(_wifiClient, url);
  http.addHeader("Content-Type", "application/octet-stream");

  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK)
  {
    WiFiClient *stream = http.getStreamPtr();
    size_t total = http.getSize();

    // Prepare for OTA update
    if (Update.begin(total))
    {
      size_t written = 0;
      esp_task_wdt_reset(); // Reset watchdog during long download

      // Read and write the stream
      uint8_t buffer[1024];
      while (http.connected() && (written < total))
      {
        size_t available = stream->available();
        if (available)
        {
          size_t bytesRead = stream->readBytes(buffer, min(available, sizeof(buffer)));
          size_t bytesWritten = Update.write(buffer, bytesRead);
          if (bytesWritten > 0)
          {
            written += bytesWritten;
            // Log progress (every 10%)
            if (written % (total / 10) < 1024)
            {
              Serial.printf("Progress: %d%%\n", (written * 100) / total);
              char progressMsg[32];
              sprintf(progressMsg, "Progress: %d%%", (written * 100) / total);
              publishUpdateStatus("Downloading", progressMsg);
            }
          }
          else
          {
            Serial.println("Error writing update");
            publishUpdateStatus("Failed", "Error writing update");
            break;
          }
        }
        delay(1);
      }

      // Check if the update is complete
      if (written == total)
      {
        Serial.println("Firmware download complete, verifying...");
        publishUpdateStatus("Verifying", "Firmware download complete, verifying");

        // Verify update before finalizing
        if (Update.end())
        {
          if (Update.isFinished())
          {
            Serial.println("OTA Update successful, restarting...");
            publishUpdateStatus("Success", "Update successful, restarting");
            success = true;
            // Give MQTT message time to send before restart
            delay(1000);
          }
          else
          {
            Serial.println("OTA Update not finished");
            publishUpdateStatus("Failed", "Update not finished properly");
          }
        }
        else
        {
          Serial.printf("Error during update finalization: %d\n", Update.getError());
          publishUpdateStatus("Failed", "Error during update finalization");
        }
      }
      else
      {
        Serial.println("Firmware size mismatch");
        publishUpdateStatus("Failed", "Firmware size mismatch");
        Update.abort();
      }
    }
    else
    {
      Serial.println("Not enough space for update");
      publishUpdateStatus("Failed", "Not enough space for update");
    }
  }
  else
  {
    Serial.printf("HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
    char errorMsg[64];
    sprintf(errorMsg, "HTTP error: %d", httpCode);
    publishUpdateStatus("Failed", errorMsg);
  }
  http.end();
  return success;
}

// Generate weather data JSON string
String WifiClient::generateWeatherDataJson()
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
  updateDateTime();

  // Create JSON document
  StaticJsonDocument<256> doc;
  doc["device_id"] = _config.getDeviceId();
  doc["recorded_at"] = _dateTime.asStr();

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

// Function to publish status updates
void WifiClient::publishUpdateStatus(const char *status, const char *message)
{
  StaticJsonDocument<200> doc;
  doc["status"] = status;
  doc["message"] = message;

  Serial.printf("Published status: %s - %s\n", status, message);
}

// Function to generate and publish weather data
void WifiClient::publishWeatherData()
{
  String jsonString = generateWeatherDataJson();

  if (WiFi.status() == WL_CONNECTED && _mqttClient.connected())
  {
    // Publish to device-specific weather topic
    _mqttClient.publish(_config.getAwsIotDeviceWeatherTopic(), jsonString.c_str());
  }
  else
  {
    // Save to SD card if WiFi or MQTT is not available
    if (_sdCard.isAvailable())
    {
      if (_sdCard.saveWeatherData(jsonString.c_str()))
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

// Function to generate status report
void WifiClient::publishStatusReport()
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
  int WIFI_RSSI = WiFi.RSSI();

  // SD Card status and storage
  const char *SD_CARD = _sdCard.isAvailable() ? "Available" : "Not available";
  int PENDING_RECORDS;
  if (_sdCard.isAvailable())
  {
    PENDING_RECORDS = _sdCard.getPendingRecords();
  }

  updateDateTime();

  StaticJsonDocument<256> doc;
  doc["device_id"] = _config.getDeviceId();
  doc["firmware_version"] = Config::FIRMWARE_VERSION;
  doc["activated"] = _isDeviceActivated;
  doc["ram_usage"] = RAM_USAGE;
  doc["flash_usage"] = FLASH_USAGE;
  doc["wifi_quality"] = WIFI_RSSI;
  doc["sd_card"] = SD_CARD;
  doc["pending_records"] = PENDING_RECORDS;
  doc["date_time"] = _dateTime.asStr();

  String jsonStr;
  serializeJson(doc, jsonStr);

  // Publish to device-specific status topic
  _mqttClient.publish(_config.getAwsIotDeviceCommandTopic(), jsonStr.c_str());
}

int WifiClient::updateDateTime()
{
  _timeClient.update();
  unsigned long unix_epoch = _timeClient.getEpochTime();

  // Convert Unix epoch time to readable format
  struct tm *timeinfo;
  time_t rawtime = unix_epoch;
  timeinfo = localtime(&rawtime);

  if (timeinfo->tm_year >= 70)
  {
    byte second_ = timeinfo->tm_sec;
    byte minute_ = timeinfo->tm_min;
    byte hour_ = timeinfo->tm_hour;
    byte day_ = timeinfo->tm_mday;
    byte month_ = timeinfo->tm_mon + 1;
    int year_ = timeinfo->tm_year + 1900;

    _dateTime = DateTime(year_, month_, day_, hour_, minute_, second_);
    return 0;
  }

  return -1;
}

WifiClient::WifiClient(const char *ssid, const char *password)
    : _timeClient(_ntpUdp, "asia.pool.ntp.org", 28800, 60000),
      BaseClient(_wifiClient)
{
}

void WifiClient::beginInner()
{
}

void WifiClient::loop()
{
  esp_task_wdt_reset();
  unsigned long currentMillis = millis();

  // Maintain WiFi connection
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Network disconnected. Reconnecting...");
    connectWifi();
  }

  // Maintain MQTT connection
  if (!_mqttClient.connected())
  {
    Serial.println("MQTT disconnected. Reconnecting...");
    connectToAws();
  }

  // Process MQTT messages
  _mqttClient.loop();

  // Publish weather data at regular intervals regardless of connectivity
  if (currentMillis - _lastWeatherPublish >= WEATHER_PUBLISH_INTERVAL)
  {
    _lastWeatherPublish = currentMillis;
    if (_isDeviceActivated)
    {
      // Try to publish if connected
      if (WiFi.status() == WL_CONNECTED && _mqttClient.connected())
      {
        publishWeatherData();
        Serial.println("Weather data published to MQTT");
      }
      // Otherwise save locally if SD card is available
      else if (_sdCard.isAvailable())
      {
        // Always generate data
        String weatherData = generateWeatherDataJson();

        if (_sdCard.saveWeatherData(weatherData.c_str()))
        {
          Serial.println("Weather data saved to SD card (offline mode)");
        }
      }
    }
  }

  // Check and synchronize offline data if we're back online
  checkAndSyncData();
}