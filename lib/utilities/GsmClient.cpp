#include "GsmClient.h"

#include <Arduino.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <Update.h>

#include "secrets.h"
#include "SensorManager.h"

#define UART_BAUD 115200
#define PIN_DTR 25
#define PIN_TX 26
#define PIN_RX 27
#define PWR_PIN 4
#define PIN_RI 33
#define RESET 5

HardwareSerial SerialAT(1);

unsigned long lastEpoch = 0; // Store last valid epoch timestamp

// Updates baud rate to the required one
void GsmClient::updateBaudRate()
{
    static uint32_t rates[] = {115200, 9600, 57600, 38400, 19200, 74400, 74880,
                               230400, 460800, 2400, 4800, 14400, 28800};
    for (uint8_t i = 0; i < sizeof(rates) / sizeof(rates[0]); i++)
    {
        uint32_t rate = rates[i];
        SerialAT.updateBaudRate(rate);
        delay(10);
        for (int j = 0; j < 10; j++)
        {
            SerialAT.print("AT\r\n");
            String input = SerialAT.readString();
            if (input.indexOf("OK") >= 0)
            {
                return;
            }
        }
    }
    SerialAT.updateBaudRate(115200);
}

void GsmClient::connectGsm()
{
    Serial.println("Initializing GSM modem...");

    // A7670-GSM Reset
    pinMode(RESET, OUTPUT);
    digitalWrite(RESET, LOW);
    delay(100);
    digitalWrite(RESET, HIGH);
    delay(100);
    digitalWrite(RESET, LOW);
    delay(100);

    // A7670-GSM Power
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    delay(100);
    digitalWrite(PWR_PIN, HIGH);
    delay(100);
    digitalWrite(PWR_PIN, LOW);
    delay(1000); // Increased delay

    Serial.println("Starting Serial Communications...");
    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
    _modem.init();
    updateBaudRate();
    delay(2000); // Wait for the modem to initialize

    Serial.println("Connecting to cellular network...");
    _modem.gprsConnect(APN);
    unsigned long gsmStart = millis();
    while (!_modem.isNetworkConnected() && (millis() - gsmStart < 10000))
    {
        Serial.print(".");
        delay(1000);
    }
    if (_modem.isNetworkConnected())
    {
        Serial.println("\nGSM connected!");
        _gsmRetryCount = 0;
    }
    else
    {
        _gsmRetryCount++;
        Serial.printf("\nGSM connection failed. Attempt %d/%d\n", _gsmRetryCount, MAX_CONNECT_RETRIES);

        // Instead of restarting after maxRetries, collect and store data locally
        if (_gsmRetryCount >= MAX_CONNECT_RETRIES)
        {
            Serial.println("Maximum GSM retries reached. Continuing in offline mode.");

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
            _gsmRetryCount = 0;
            delay(60000); // Wait a minute before trying again
        }
    }

    updateDateTime();
    Serial.printf("Date and Time: %s\n", _dateTime.c_str());
    Serial.println("---------------------------------");
}

void GsmClient::connectToAws()
{
    if (_modem.isNetworkConnected())
    {
        Serial.println("Connecting to AWS IoT...");
        _mqttClient.setServer(AWS_IOT_ENDPOINT, AWS_IOT_PORT);
        _mqttClient.setCallback([this](char *topic, byte *payload, unsigned int length)
                                { handleMqttMessage(topic, payload, length); });

        if (!_sslClient.connected())
        {
            // Only set certs if SSL connection is fresh
            _sslClient.setCACert(AWS_CERT_CA);
            _sslClient.setCertificate(AWS_CERT_CRT);
            _sslClient.setPrivateKey(AWS_CERT_PRIVATE);
        }

        if (!_mqttClient.connected())
        {
            _mqttRetryCount++;
            Serial.printf("Connecting to AWS IoT. Attempt %d/%d\n", _mqttRetryCount, MAX_CONNECT_RETRIES);

            if (_mqttClient.connect(_config.getDeviceId()))
            {
                Serial.println("Connected!");
                _mqttRetryCount = 0;

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
void GsmClient::handleMqttMessage(char *topic, byte *payload, unsigned int length)
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
        const char *command = doc["command"];
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
            Serial.printf("Device activated successfully at %s.\n", _dateTime.c_str());
            publishUpdateStatus("Activated", "Device activated successfully");
        }
        // Handle device deactivation
        else if (command && strcmp(command, "deactivate") == 0)
        {
            // Deactivate the device
            activateDevice(false);
            Serial.printf("Device deactivated successfully at %s.\n", _dateTime.c_str());
            publishUpdateStatus("Deactivated", "Device deactivated successfully");
        }
        // Commands that require activation
        else if (!_isDeviceActivated)
        {
            Serial.printf("Command '%s' rejected: Device not activated\n", command ? command : "unknown");
            publishUpdateStatus("Rejected", "Device not activated - command ignored");
        }
        else
        {
            // Handle device reset
            if (command && strcmp(command, "reset") == 0)
            {
                updateDateTime();
                StaticJsonDocument<128> resetDoc;
                resetDoc["recorded_at"] = _dateTime.asStr();
                resetDoc["command"] = "reset";
                String resetJson;
                serializeJson(resetDoc, resetJson);
                _mqttClient.publish(_config.getAwsIotDeviceCommandTopic(), resetJson.c_str());

                // Reset the device
                Serial.printf("Reset command received at %s. Restarting device...\n", _dateTime.c_str());
                publishUpdateStatus("Resetting", "Device restarting per command request");
                delay(1000);
                ESP.restart();
            }
            // Handle OTA update command
            else if (command && strcmp(command, "update") == 0)
            {
                updateDateTime();
                StaticJsonDocument<128> updateDoc;
                updateDoc["recorded_at"] = _dateTime.asStr();
                updateDoc["command"] = "update";
                String updateJson;
                serializeJson(updateDoc, updateJson);
                _mqttClient.publish(_config.getAwsIotDeviceCommandTopic(), updateJson.c_str());

                if (doc["url"].is<const char *>())
                {
                    handleUpdateCommand(doc);
                    Serial.printf("OTA update command processed at %s.", _dateTime.c_str());
                    publishUpdateStatus("OTA Update", "OTA update command processed");
                }
                else
                {
                    Serial.printf("Error: Missing URL for OTA update at %s.", _dateTime.c_str());
                    publishUpdateStatus("Error", "Missing url for update");
                }
            }
            // Handle force sync command
            else if (command && strcmp(command, "sync") == 0)
            {
                static unsigned long lastSyncCommandTime = 0;
                unsigned long now = millis();
                // Only allow sync command to be processed if at least 10 seconds have passed since last one
                if (now - lastSyncCommandTime > 10000 || lastSyncCommandTime == 0)
                {
                    lastSyncCommandTime = now;

                    updateDateTime();
                    StaticJsonDocument<128> syncDoc;
                    syncDoc["recorded_at"] = _dateTime.asStr();
                    syncDoc["command"] = "sync";
                    String syncJson;
                    serializeJson(syncDoc, syncJson);
                    _mqttClient.publish(_config.getAwsIotDeviceCommandTopic(), syncJson.c_str());

                    // Force sync command received
                    Serial.printf("Force sync command received at %s.", _dateTime.c_str());
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
                    _lastSyncAttempt = millis(); // Update last sync attempt time
                }
                else
                {
                    Serial.println("Sync command ignored: too soon since last sync.");
                }
            }
            // Handle force collect data command
            else if (command && strcmp(command, "data") == 0)
            {
                // Respond with current weather data when requested
                Serial.println("Weather data requested.");
                String jsonString = generateWeatherDataJson();
                _mqttClient.publish(_config.getAwsIotDeviceCommandTopic(), jsonString.c_str());
                publishUpdateStatus("Weather Data", "Current weather data sent");
            }
        }
    }
    Serial.println("---------------------------------");
}

// Handle Update Command
void GsmClient::handleUpdateCommand(const JsonDocument &doc)
{
    // Check if device is activated
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

    publishUpdateStatus("Starting", "Beginning OTA update...");
    performOTAUpdate(url.c_str());
}

void parseURL(const String &url, String &host, int &port, String &path)
{
    if (!url.startsWith("https://"))
        return;

    int index = 8; // after "https://"
    int slashIndex = url.indexOf('/', index);
    String hostPort = url.substring(index, slashIndex);
    int colonIndex = hostPort.indexOf(':');

    if (colonIndex != -1)
    {
        host = hostPort.substring(0, colonIndex);
        port = hostPort.substring(colonIndex + 1).toInt();
    }
    else
    {
        host = hostPort;
        port = 443;
    }
    path = url.substring(slashIndex);
}

// Add this function to your code to test the connection before attempting OTA
bool GsmClient::testServerConnection(const String &host, int port)
{
    Serial.printf("[OTA] Testing connection to %s:%d\n", host.c_str(), port);

    // Reset SSL client state
    _sslClient.stop();
    delay(500);

    // Set certificate
    _sslClient.setCACert(AWS_CERT_CA);

    unsigned long startTime = millis();
    bool connected = false;

    // Print network info
    Serial.print("[OTA] IP Address: ");
    Serial.println(_modem.localIP());
    Serial.print("[OTA] Signal Quality: ");
    Serial.println(_modem.getSignalQuality());

    // Try to connect with timeout
    for (int attempt = 1; attempt <= 3; attempt++)
    {
        Serial.printf("[OTA] Connection attempt %d...\n", attempt);

        if (_sslClient.connect(host.c_str(), port))
        {
            connected = true;
            Serial.printf("[OTA] Connected successfully in %lu ms\n", millis() - startTime);
            break;
        }

        Serial.println("[OTA] Connection failed, retrying...");
        delay(1000 * attempt); // Exponential backoff
    }

    if (!connected)
    {
        Serial.printf("[OTA] Connection failed after %lu ms\n", millis() - startTime);
        return false;
    }

    // Send a simple HEAD request to check if server responds
    _sslClient.println("HEAD / HTTP/1.1");
    _sslClient.println("Host: " + String(host));
    _sslClient.println("Connection: close");
    _sslClient.println();

    // Wait for response with timeout
    unsigned long timeout = millis();
    while (millis() - timeout < 5000)
    {
        if (_sslClient.available())
        {
            String line = _sslClient.readStringUntil('\n');
            Serial.println("[OTA] Server response: " + line);
            _sslClient.stop();
            return true;
        }
        delay(10);
    }

    Serial.println("[OTA] No response from server");
    _sslClient.stop();
    return false;
}

// Function to perform OTA update using GSM
bool GsmClient::performOTAUpdate(const String &url)
{
    String host, path;
    int port;

    publishUpdateStatus("OTA", "Starting OTA update process");

    // Parse the URL
    parseURL(url, host, port, path);
    if (host.length() == 0 || path.length() == 0)
    {
        Serial.println("[OTA] Invalid OTA URL");
        publishUpdateStatus("Error", "Invalid OTA URL format");
        return false;
    }

    Serial.printf("[OTA] Connecting to: %s:%d\n", host.c_str(), port);
    publishUpdateStatus("OTA", "Connecting to update server");

    // Test server connection before proceeding
    if (!testServerConnection(host, port))
    {
        publishUpdateStatus("Error", "Failed to establish connection to update server");
        return false;
    }

    // Reset the SSL client before creating HTTP client to ensure fresh connection
    _sslClient.stop();
    delay(1000);

    // Set the certificates for the OTA server specifically
    _sslClient.setCACert(AWS_CERT_CA);

    HttpClient http(_sslClient, host, port);
    http.setTimeout(10000); // 10 seconds timeout

    http.connectionKeepAlive();
    publishUpdateStatus("OTA", "Downloading firmware");
    Serial.println("[OTA] Sending HTTP GET request");

    // Send GET request with retry mechanism
    int maxRetries = 3;
    int err = -1;

    for (int retry = 0; retry < maxRetries; retry++)
    {
        err = http.get(path);
        if (err == 0)
            break;

        Serial.printf("[OTA] HTTP GET failed (attempt %d/%d): %d\n", retry + 1, maxRetries, err);
        delay(1000 * (retry + 1)); // Exponential backoff
    }

    if (err != 0)
    {
        String errorMsg = "HTTP request failed: " + String(err);
        publishUpdateStatus("Error", errorMsg.c_str());
        return false;
    }

    int status = http.responseStatusCode();
    Serial.printf("[OTA] HTTP status code: %d\n", status);

    if (status <= 0)
    {
        String errorMsg = "Invalid HTTP status: " + String(status);
        publishUpdateStatus("Error", errorMsg.c_str());
        return false;
    }

    if (status != 200)
    {
        String errorMsg = "HTTP error: " + String(status);
        publishUpdateStatus("Error", errorMsg.c_str());
        return false;
    }

    int contentLength = http.contentLength();
    if (contentLength <= 0)
    {
        Serial.println("[OTA] Invalid content length");
        publishUpdateStatus("Error", "Invalid content length");
        return false;
    }

    Serial.printf("[OTA] Update size: %d bytes\n", contentLength);

    if (!Update.begin(contentLength))
    {
        Serial.println("[OTA] Update.begin() failed");
        return false;
    }

    Serial.println("[OTA] Starting update...");
    publishUpdateStatus("OTA", "Writing firmware to flash");

    // Significantly increase buffer size (use 4KB or 8KB if memory allows)
    const size_t bufferSize = 4096; // 4KB buffer instead of 512 bytes
    uint8_t *buff = (uint8_t *)malloc(bufferSize);

    if (!buff)
    {
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

    while (http.connected() && written < contentLength)
    {
        // Check for read timeout
        if (millis() - lastRead > readTimeout)
        {
            Serial.println("[OTA] Read timeout");
            publishUpdateStatus("Error", "Download timeout");
            free(buff);
            Update.abort();
            return false;
        }

        esp_task_wdt_reset(); // Reset watchdog during long download

        int available = http.available();
        if (available > 0)
        {
            lastRead = millis(); // Reset timeout counter

            // Read up to buffer size
            size_t toRead = min(available, (int)bufferSize);
            int readBytes = http.read(buff, toRead);

            if (readBytes > 0)
            {
                // Write to Update
                if (Update.write(buff, readBytes) != readBytes)
                {
                    Serial.printf("[OTA] Write failed: %s\n", Update.errorString());
                    free(buff);
                    Update.abort();
                    return false;
                }

                written += readBytes;

                // Report progress every 5%
                uint32_t progress = (written * 100) / contentLength;
                if (progress - lastProgress >= 5 || progress == 100)
                {
                    lastProgress = progress;
                    Serial.printf("[OTA] Progress: %d%%\n", progress);
                }

                // Small delay to allow background tasks (WiFi/GSM stack, etc.)
                delay(1);
            }
        }
        else if (available == 0)
        {
            // No data available, give the GSM modem time to process
            delay(10);
        }
    }

    free(buff); // Free the buffer

    uint32_t updateTime = (millis() - startTime) / 1000;
    Serial.printf("[OTA] Download completed in %d seconds\n", updateTime);

    if (written != contentLength)
    {
        Serial.printf("[OTA] Size mismatch: %d != %d\n", written, contentLength);
        publishUpdateStatus("Error", "Size mismatch in downloaded firmware");
        Update.abort();
        return false;
    }

    Serial.println("[OTA] Finishing update...");
    publishUpdateStatus("OTA", "Finalizing firmware update");

    if (!Update.end())
    {
        Serial.printf("[OTA] Update.end() failed: %s\n", Update.errorString());
        return false;
    }

    if (!Update.isFinished())
    {
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

// Generate sensor status JSON string
String GsmClient::generateSensorStatusJSON()
{
    updateDateTime();
    StaticJsonDocument<256> doc;
    doc["recorded_at"] = _dateTime.asStr();

    // Get sensor status from SensorManager
    SensorStatus status = _sensorManager.getSensorStatus();
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

// Generate status information JSON string
String GsmClient::generateStatusInfoJSON()
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
    int SIGNAL_QUALITY = _modem.getSignalQuality();

    // SD Card status and storage
    const char *SD_CARD = _sdCard.isAvailable() ? "Available" : "Not available";
    int PENDING_RECORDS = 0;
    if (_sdCard.isAvailable())
    {
        PENDING_RECORDS = _sdCard.getPendingRecords();
    }

    updateDateTime(); // Ensure we have the latest time

    StaticJsonDocument<512> doc;
    doc["rec_at"] = _dateTime.asStr();
    doc["device_id"] = _config.getDeviceId();
    doc["firmware"] = Config::FIRMWARE_VERSION;
    doc["activated"] = _isDeviceActivated;
    doc["ram"] = RAM_USAGE;
    doc["flash"] = FLASH_USAGE;
    doc["sd"] = SD_CARD;
    doc["pending"] = PENDING_RECORDS;

    // Serialize to JSON
    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
}

// Generate weather data JSON string
String GsmClient::generateWeatherDataJson()
{
    // Get time
    updateDateTime();

    // Create JSON document
    StaticJsonDocument<256> doc;
    doc["recorded_at"] = _dateTime.asStr();

    // Collect sensor data
    SensorReadings readings = _sensorManager.readAllSensors();
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
void GsmClient::publishWeatherData()
{
    String jsonString = generateWeatherDataJson();

    if (_modem.isNetworkConnected() && _mqttClient.connected())
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
void GsmClient::publishStatusReport()
{
    String jsonString = generateStatusInfoJSON();

    // Publish to device-specific status topic
    _mqttClient.publish(_config.getAwsIotDeviceCommandTopic(), jsonString.c_str());
}

// Function to publish sensor status
void GsmClient::publishSensorStatus()
{
    String jsonString = generateSensorStatusJSON();

    // Publish to device-specific status topic
    _mqttClient.publish(_config.getAwsIotDeviceCommandTopic(), jsonString.c_str());
}

// Function to publish status updates
void GsmClient::publishUpdateStatus(const char *status, const char *message)
{
    StaticJsonDocument<200> doc;
    doc["status"] = status;
    doc["message"] = message;

    Serial.printf("Published status: %s - %s\n", status, message);
}

// Function to convert date/time to UNIX timestamp
unsigned long convertToEpoch(const String &year, const String &month, const String &day, const String &hour, const String &minute, const String &second)
{
    struct tm t;
    t.tm_year = year.toInt() + 2000 - 1900;
    t.tm_mon = month.toInt() - 1;
    t.tm_mday = day.toInt();
    t.tm_hour = hour.toInt();
    t.tm_min = minute.toInt();
    t.tm_sec = second.toInt();
    return mktime(&t);
}

int GsmClient::updateDateTime()
{
    String response = "";
    SerialAT.print("AT+CCLK?\r\n");
    delay(100);
    response = SerialAT.readString();

    if (response == "")
    {
        return -1;
    }

    int startIndex = response.indexOf("+CCLK: \"");
    int endIndex = response.indexOf("\"", startIndex + 8);

    if (startIndex == -1 || endIndex == -1)
    {
        return -1;
    }

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
    if (plusIndex != -1)
    {
        second = second.substring(0, plusIndex);
    }

    // Convert to epoch time
    unsigned long newEpoch = convertToEpoch(year, month, day, hour, minute, second);

    // Filtering: Ignore large jumps
    if (lastEpoch == 0 || abs((long)newEpoch - (long)lastEpoch) <= TIME_THRESHOLD)
    {
        lastEpoch = newEpoch;
        _dateTime = DateTime("20" + year, month, day, hour, minute, second);
    }
    else
    {
        Serial.print("Time jump detected (");
        Serial.print(abs((long)newEpoch - (long)lastEpoch));
        Serial.println("s), ignoring...");
    }

    return 0;
}

GsmClient::GsmClient()
    : _modem(SerialAT),
      _gsmClient(_modem),
      _sslClient(&_gsmClient),
      BaseClient(_sslClient)
{
}

void GsmClient::beginInner()
{
    _sensorManager.begin();
}

void GsmClient::loop()
{
    esp_task_wdt_reset();
    unsigned long currentMillis = millis();

    // Maintain GSM connection
    if (!_modem.isNetworkConnected())
    {
        Serial.println("Network disconnected. Reconnecting...");
        connectGsm();
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
            if (_modem.isNetworkConnected() && _mqttClient.connected())
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
        else
        {
            // Device is deactivated - only store locally if SD card is available
            if (_sdCard.isAvailable())
            {
                String weatherData = generateWeatherDataJson();
                if (_sdCard.saveWeatherData(weatherData.c_str()))
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