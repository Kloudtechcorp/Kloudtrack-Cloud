#include "MqttUtils.h"

MqttUtils::MqttUtils(SSLClient* sslClient, TinyGsm* modem)
    : _sslClient(sslClient),
    _mqttClient(*sslClient),
    _serialAT(1),
    _modem(modem),
    _currentLogLevel(LOG_INFO),
    _isActivated(false)
{
    // Initialize config and other components
    _config.begin();
    _sdCard.begin();

    // Setup MQTT client
    _mqttClient.setServer(_config.getAwsIotEndpoint(), _config.getAwsIotPort());
    _mqttClient.setCallback([this](char* topic, byte* payload, unsigned int length) {
        this->messageHandler(topic, payload, length);
    });
}

void MqttUtils::loop() {
    // Service the MQTT client to handle incoming messages and maintain connection
    _mqttClient.loop();
}

void MqttUtils::messageHandler(char *topic, byte *payload, unsigned int length) {
    esp_task_wdt_reset();
    SerialMon.print("Message received on topic: ");
    SerialMon.println(topic);

    const unsigned int MAX_PAYLOAD_SIZE = 2048;
    if (length > MAX_PAYLOAD_SIZE) {
        SerialMon.printf("Payload too large (%d bytes), ignoring message\n", length);
        return;
    }

    char *payloadStr = new char[length + 1];
    if (!payloadStr) {
        SerialMon.println("Failed to allocate memory for payload");
        return;
    }
    memcpy(payloadStr, payload, length);
    payloadStr[length] = '\0';
    SerialMon.println(payloadStr);

    StaticJsonDocument<300> doc;
    DeserializationError error = deserializeJson(doc, payloadStr);
    delete[] payloadStr;
    esp_task_wdt_reset();

    if (error) {
        SerialMon.print("DeserializationJson() failed: ");
        SerialMon.println(error.f_str());
        return;
    }

    // Get MQTT Pub/Sub Topics
    const char *awsDeviceCommandTopic = _config.getAwsIotDeviceCommandTopic();
    const char *awsDeviceResponseTopic = _config.getAwsIotDeviceResponseTopic();
    const char *awsDeviceDataTopic = _config.getAwsIotDeviceDataTopic();

    if (strcmp(topic, awsDeviceCommandTopic) == 0) {
        const char* command = doc["command"];
        
        // Commands that does not require activation
        if (!_isActivated || _isActivated) {
            
            // Handle status information
            if (command && strcmp(command, "info") == 0) {
                publishStatusReport();
                SerialMon.println("Device information requested.");
                publishUpdateStatus("Status", "Station information sent successfully.");
            }

            // Handle sensor status
            if (command && strcmp(command, "sensor") == 0) {
                publishSensorStatus();
                SerialMon.println("Sensors status requested.");
                publishUpdateStatus("Sensor status", "Sensors status sent succesfully.");
            }

            // Handle device activation
            if (command && strcmp(command, "activate") == 0) {
                activateDevice(true);
                SerialMon.printf("Device activation requested.");
                publishUpdateStatus("Activated", "Device activated succesfully.");
            }

            // Handle device deactivation
            if (command && strcmp(command, "deactivate") == 0) {
                activateDevice(false);
                SerialMon.printf("Device deactivation requested.");
                publishUpdateStatus("Deactivated", "Device deactivated succesfully.");
            }
        }
        
        if (_isActivated) {
            // Handle device reset
            if (command && strcmp(command, "reset") == 0) {
                SerialMon.println("Device reset requested.");
                publishUpdateStatus("Resetting", "Device restarting after 1 second.");
                delay(1000);
                ESP.restart();
            }

            // Handle OTA update command
            if (command && strcmp(command, "update") == 0) {
                esp_task_wdt_reset();
                if (doc.containsKey("url")) {
                    handleUpdateCommand(doc);
                    SerialMon.println("OTA update requested.");
                    publishUpdateStatus("OTA Update", "Starting OTA update.");
                }
                else {
                    SerialMon.println("Error: Missing URL for OTA update.");
                    publishUpdateStatus("Error", "Missing url for update");
                }
            }

            // Handle force syncing of weather data
            if (command && strcmp(command, "sync") == 0) {
                static unsigned long lastSyncCommandTime = 0;
                unsigned long now = millis();

                if (now - lastSyncCommandTime > 10000 || lastSyncCommandTime == 0) {
                    lastSyncCommandTime = now;
                    SerialMon.println("Force sync requested.");
                    publishUpdateStatus("Sync", "Starting force syncing of weather data.");

                    if (syncOfflineData()) {
                        SerialMon.println("Offline data synchronized successfully.");
                        publishUpdateStatus("Synced", "Offline data synchronized successfully.");
                    } else {
                        SerialMon.println("Failed to synchronize offline data.");
                        publishUpdateStatus("Sync failed", "Failed to synchronize offline data.");
                    }
                } else {
                    SerialMon.println("Sync command ignored: too soon since last sync.");
                }
            }

            // Handle force collection of weather data
            if (command && strcmp(command, "data") == 0) {
                String jsonString = generateWeatherDataJson();
                _mqttClient.publish(awsDeviceResponseTopic, jsonString.c_str());
                SerialMon.println("Weather data requested.");
                publishUpdateStatus("Weather data", "Current weather data sent successfully.");
            }

            // Handle network health information
            if (command && strcmp(command, "network") == 0) {
                String jsonString = generateNetworkHealthJson();
                _mqttClient.publish(awsDeviceResponseTopic, jsonString.c_str());
                SerialMon.println("Network health requested.");
                publishUpdateStatus("Network health", "Network health status sent successfully.");
            }

            // Credential management commands
            // Handle setting up of Wifi credentials
            if (command && strcmp(command, "set_wifi") == 0) {
                if (doc.containsKey("ssid") && doc.containsKey("password")) {
                    String ssid = doc["ssid"].as<String>();
                    String password = doc["password"].as<String>();

                    // Get credentials from Preferences
                    bool wifiCredentials = _config.setWifiCredentials(ssid.c_str(), password.c_str());

                    if (wifiCredentials) {
                        SerialMon.printf("WiFi credentials updated: %s\n", ssid.c_str());
                        publishUpdateStatus("WiFi Updated", "WiFi credentials updated successfully");
                    } else {
                        SerialMon.println("Failed to update WiFi credentials");
                        publishUpdateStatus("WiFi Error", "Failed to update WiFi credentials");
                    } 
                } else {
                    SerialMon.println("Missing SSID or password for WiFi update");
                    publishUpdateStatus("WiFi Error", "Missing SSID or password");
                }
            }

            // Handle setting up of GSM credentials
            if (command && strcmp(command, "set_gsm") == 0) {
                if (doc.containsKey("apn")) {
                    String apn = doc["apn"].as<String>();

                    // Get credentials from Preferences
                    bool gsmCredentials = _config.setGsmCredentials(apn.c_str());
                    
                    if (gsmCredentials) {
                        SerialMon.printf("GSM APN updated: %s\n", apn.c_str());
                        publishUpdateStatus("GSM Updated", "GSM APN updated successfully");
                    } else {
                        SerialMon.println("Failed to update GSM APN");
                        publishUpdateStatus("GSM Error", "Failed to update GSM APN");
                    }
                } else {
                    SerialMon.println("Missing APN for GSM update");
                    publishUpdateStatus("GSM Error", "Missing APN");
                }
            }

            // Handle setting up of AWS credentials
            if (command && strcmp(command, "set_aws") == 0) {
                if (doc.containsKey("endpoint") && doc.containsKey("port")) {
                    String endpoint = doc["endpoint"].as<String>();
                    int port = doc["port"].as<int>();
                
                    // Get AWS Endpoint and Port
                    const char *awsEndpoint = _config.getAwsIotEndpoint();
                    int awsPort = _config.getAwsIotPort();
                    bool awsCredentials =
                    _config.setAwsCredentials(endpoint.c_str(), port);

                    if (awsCredentials) {
                        SerialMon.printf("AWS credentials updated: %s:%d\n", endpoint.c_str(), port);
                        publishUpdateStatus("AWS Updated", "AWS credentials updated successfully");
                    } else {
                        SerialMon.println("Failed to update AWS credentials");
                        publishUpdateStatus("AWS Error", "Failed to update AWS credentials");
                    }
                } else {
                    SerialMon.println("Missing endpoint or port for AWS update");
                    publishUpdateStatus("AWS Error", "Missing endpoint or port");
                }
            }

            // Handle sending of current credentials
            if (command && strcmp(command, "get_credentials") == 0) {
                StaticJsonDocument<300> credDoc;
                credDoc["command"] = "credentials_status";
                credDoc["gsm_configured"] = _config.hasGsmCredentials();
                credDoc["aws_configured"] = _config.hasAwsCredentials();
                
                String credJson;
                serializeJson(credDoc, credJson);
                _mqttClient.publish(awsDeviceResponseTopic, credJson.c_str());
                publishUpdateStatus("Credentials", "Current credentials status sent");
            }

            // Handle clearing of credentials
            if (command && strcmp(command, "clear_credentials") == 0) {
                _config.clearCredentials();
                SerialMon.println("All credentials cleared, using defaults");
                publishUpdateStatus("Credentials Cleared", "All credentials cleared, using defaults");
            }
        }

        if (!_isActivated && command && strcmp(command, "reset") == 0) {
            SerialMon.printf("Command '%s' rejected: Device not  activated\n", command ? command : "unknown");
            publishUpdateStatus("Rejected", "Device not activated - command ignored");
        }
    }
    SerialMon.println("---------------------------------");
}

void MqttUtils::logMessage(LogLevel level, const char* module, const char* message) {
    if (level < _currentLogLevel) return;

    const char* levelStr[] = {"DEBUG", "INFO", "WARN", "ERROR", "CRITICAL"};
    unsigned long timestamp = millis();

    Serial.printf("[%lu] [%s] [%s] %s\n", timestamp, levelStr[level], module, message);

    // For critical errors, also publish via MQTT if connected
    if (level >= LOG_ERROR && _mqttClient.connected()) {
        StaticJsonDocument<200> logDoc;
        logDoc["timestamp"] = timestamp;
        logDoc["level"] = levelStr[level];
        logDoc["module"] = module;
        logDoc["message"] = message;

        String logJson;
        serializeJson(logDoc, logJson);

        _mqttClient.publish(_config.getAwsIotDeviceResponseTopic(),      
        logJson.c_str());
    }
}

void MqttUtils::publishUpdateStatus(const char *status, const char *message) {
    _dateTime.begin(_serialAT);
    StaticJsonDocument<160> doc; 
    doc["status"] = status;
    doc["message"] = message;
    doc["recorded_at"] = _dateTime.asStr();

    String jsonStr;
    serializeJson(doc, jsonStr);
    _mqttClient.publish(_config.getAwsIotDeviceCommandTopic(), jsonStr.c_str());
    SerialMon.printf("Published status: %s - %s\n", status, message);
}

bool MqttUtils::isDeviceActivated() {
    _preferences.begin("kloudtrack", false);
    bool activated = _preferences.getBool("activated", false);
    _preferences.end();
    return activated;
}

void MqttUtils::activateDevice(bool activate) {
    _preferences.begin("kloudtrack", false);
    _preferences.putBool("activated", activate);
    _preferences.end();
    _isActivated = activate;
    _dateTime.begin(_serialAT);

    StaticJsonDocument<160> doc;
    doc["device_id"] = _config.getDeviceId();
    doc["firmware_version"] = Config::FIRMWARE_VERSION;
    doc["activated"] = activate;
    doc["recorded_at"] = _dateTime.asStr();

    String jsonStr;
    serializeJson(doc, jsonStr);

    // Publish to device-specific response topic
    _mqttClient.publish(_config.getAwsIotDeviceResponseTopic(), jsonStr.c_str());
    SerialMon.printf("Device %s\n", activate ? "ACTIVATED" : "DEACTIVATED");
}

void MqttUtils::parseURL(const String &url, String &host, int &port, String &path) {
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

bool MqttUtils::testServerConnection(const String &host, int port) {
    Serial.printf("[OTA] Testing connection to %s:%d\n", host.c_str(), port);

    // Reset SSL client state
    _sslClient->stop();
    delay(500);

    // Set certificate
    _sslClient->setCACert(_config.getCaCert());

    unsigned long startTime = millis();
    bool connected = false;

    // Print network info
    Serial.print("[OTA] IP Address: ");
    Serial.println(_modem->localIP());
    Serial.print("[OTA] Signal Quality: ");
    Serial.println(_modem->getSignalQuality());

    // Try to connect with timeout
    for (int attempt = 1; attempt <= 3; attempt++) {
        Serial.printf("[OTA] Connection attempt %d...\n", attempt);

        if (_sslClient->connect(host.c_str(), port)) {
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
    _sslClient->println("HEAD / HTTP/1.1");
    _sslClient->println("Host: " + String(host));
    _sslClient->println("Connection: close");
    _sslClient->println();

    // Wait for response with timeout
    unsigned long timeout = millis();
    while (millis() - timeout < 5000) {
        if (_sslClient->available()) {
            String line = _sslClient->readStringUntil('\n');     
            Serial.println("[OTA] Server response: " + line);    
            _sslClient->stop();
            return true;
        }
        delay(10);
    }

    Serial.println("[OTA] No response from server");
    _sslClient->stop();
    return false;
}

bool MqttUtils::performOTAUpdate(const String &url) {
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
    esp_task_wdt_reset();
    if (!testServerConnection(host, port)) {
        publishUpdateStatus("Error", "Failed to establish connection to update server");
        return false;
    }

    _sslClient->stop();
    delay(1000);

    _sslClient->setCACert(_config.getCaCert());

    HttpClient http(*_sslClient, host, port);
    http.setTimeout(10000); // Long timeout for GSM
    http.connectionKeepAlive();
    publishUpdateStatus("OTA", "Downloading firmware - status updates suspended");
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
    Serial.printf("[OTA] Content-Length header: %d\n", contentLength);

    if (contentLength <= 0) {
        Serial.println("[OTA] Invalid or missing content length, trying without it");
        // Some servers don't send Content-Length, we'll try chunked transfer
        contentLength = 2000000; // Set a reasonable maximum for firmware
    }

    Serial.printf("[OTA] Update size: %d bytes\n", contentLength);

    if (!Update.begin(contentLength)) {
        Serial.println("[OTA] Update.begin() failed");
        publishUpdateStatus("Error", "Failed to begin update");
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

void MqttUtils::handleUpdateCommand(const JsonDocument& doc) {
    // Check if device is activated
    if (!_isActivated) {
        SerialMon.println("OTA update rejected: Device not activated");
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

    // Note: The actual OTA update implementation would need to be added here
    // For now, we'll just log that an update was requested      
    SerialMon.printf("OTA update requested for URL: %s\n", url.c_str());
    if (!newVersion.isEmpty()) {
        SerialMon.printf("Target version: %s\n", newVersion.c_str());
    }
    SerialMon.printf("Force update: %s\n", forceUpdate ? "YES" : "NO");

    // TODO: Implement actual OTA update functionality
    // Call the actual OTA update function
    bool otaResult = performOTAUpdate(url);
    if (!otaResult) {
        publishUpdateStatus("Error", "OTA update failed");
    }
}

void MqttUtils::publishSensorStatus() {
    String jsonStr = generateSensorStatusJSON();    
    // Publish to device-specific response topic    
    _mqttClient.publish(_config.getAwsIotDeviceResponseTopic(), jsonStr.c_str());
}

void MqttUtils::publishWeatherData() {
    String jsonString = generateWeatherDataJson();

    if (_modem->isNetworkConnected() && _mqttClient.connected()) {
        // Publish to device-specific weather topic
        _mqttClient.publish(_config.getAwsIotDeviceDataTopic(), jsonString.c_str());
    } else {
        // Save to SD card if network or MQTT is not available
        if (_sdCard.isAvailable()) {
            if (_sdCard.saveWeatherData(jsonString.c_str())) {
                Serial.println("Weather data saved to SD card (offline mode)");
            } else {
                Serial.println("Failed to save weather data to SD card");
            }
        } else {
            Serial.println("No connectivity and SD card not available - data lost");
        }
    }
}

void MqttUtils::publishStatusReport() {
    String jsonString = generateStatusInfoJSON();

    // Publish to device-specific response topic
    _mqttClient.publish(_config.getAwsIotDeviceResponseTopic(), jsonString.c_str());
}

void MqttUtils::updateNetworkHealth() {
    static unsigned long lastUpdate = 0;
    static bool lastGsmState = false;
    static bool lastMqttState = false;
    unsigned long now = millis();

    // Update every 10 seconds
    if (now - lastUpdate < 10000) return;
    lastUpdate = now;

    bool gsmConnected = _modem->isNetworkConnected();
    bool mqttConnected = _mqttClient.connected();

    // Track GSM connection state changes
    if (gsmConnected != lastGsmState) {
        if (!gsmConnected) {
            _networkHealth.lastGsmDisconnect = now;
            _networkHealth.gsmReconnectCount++;
            logMessage(LOG_WARNING, "NETWORK", "GSM connection lost");
        } else {
            logMessage(LOG_INFO, "NETWORK", "GSM connection restored");
        }
        lastGsmState = gsmConnected;
    }

    // Track MQTT connection state changes
    if (mqttConnected != lastMqttState) {
        if (!mqttConnected) {
            _networkHealth.lastMqttDisconnect = now;
            _networkHealth.mqttReconnectCount++;
            logMessage(LOG_WARNING, "NETWORK", "MQTT connection lost");
        } else {
            logMessage(LOG_INFO, "NETWORK", "MQTT connection restored");
        }
        lastMqttState = mqttConnected;
    }

    // Update uptime counters
    if (gsmConnected) {
        _networkHealth.gsmUptime = now;
    }
    if (mqttConnected) {
        _networkHealth.mqttUptime = now;
    }

    // Update signal quality
    _networkHealth.signalQuality = _modem->getSignalQuality();

    // Determine overall network health
    unsigned long gsmDowntime = gsmConnected ? 0 : (now - _networkHealth.lastGsmDisconnect);
    unsigned long mqttDowntime = mqttConnected ? 0 : (now - _networkHealth.lastMqttDisconnect);

    _networkHealth.networkHealthy = (gsmConnected && mqttConnected &&
                                    _networkHealth.signalQuality > 10 &&
                                    gsmDowntime < 300000 && //  Less than 5 minutes downtime
                                    mqttDowntime < 300000);       

    // Log critical network issues
    if (!_networkHealth.networkHealthy) {
        char msg[100];
        snprintf(msg, sizeof(msg), "Network unhealthy - GSM:%s MQTT:%s Signal:%d",
                gsmConnected ? "OK" : "DOWN",
                mqttConnected ? "OK" : "DOWN",
                _networkHealth.signalQuality);
        logMessage(LOG_WARNING, "NETWORK", msg);
    }
}

String MqttUtils::generateSensorStatusJSON() {
    _dateTime.begin(_serialAT);

    StaticJsonDocument<200> doc;
    doc["recorded_at"] = _dateTime.asStr();

    // Get sensor status from TestSensorManager
    SensorStatus status = _testSensorManager.getSensorStatus();
    JsonObject sensors = doc.createNestedObject("sensors");
    sensors["BME280"] = status.bmeAvailable ? "OK" : "ERROR";
    sensors["BMP180"] = status.bmpAvailable ? "OK" : "ERROR";
    sensors["SHT30"] = status.shtAvailable ? "OK" : "ERROR";
    sensors["BH1750"] = status.lightAvailable ? "OK" : "ERROR";
    sensors["AS5600"] = status.asAvailable ? "OK" : "ERROR";
    sensors["SLAVE"] = status.slaveAvailable ? "OK" : "ERROR";
    sensors["GUVAS12SD"] = status.guvaAvailable ? "OK" : "ERROR";

    String jsonStr;
    jsonStr.reserve(250);
    serializeJson(doc, jsonStr);
    return jsonStr;
}

String MqttUtils::generateWeatherDataJson() {
    _dateTime.begin(_serialAT);

    StaticJsonDocument<300> doc;
    doc["recorded_at"] = _dateTime.asStr();
    _testSensorManager.updateAllSensors();

    // Get sensor readings
    doc["temperature"] = _testSensorManager.getAverageTemperature();
    doc["humidity"] = _testSensorManager.getAverageHumidity();
    doc["pressure"] = _testSensorManager.getAveragePressure();
    doc["light_intensity"] = _testSensorManager.getLux();        
    doc["uv_index"] = _testSensorManager.getUvIndex();
    doc["wind_direction"] = _testSensorManager.getWindDirection();
    doc["wind_speed"] = _testSensorManager.getWindSpeed();       
    doc["precipitation"] = _testSensorManager.getPrecipitation();

    // Serialize to JSON
    String jsonString;
    jsonString.reserve(350);
    serializeJson(doc, jsonString);
    return jsonString;
}

String MqttUtils::generateStatusInfoJSON() {        
    const float TOTAL_RAM = 327680.0;
    const float TOTAL_FLASH = 1310720.0;

    // RAM usage percent (used = total - free heap)
    float used_ram = TOTAL_RAM - ESP.getFreeHeap();
    float RAM_USAGE = used_ram * 100.0 / TOTAL_RAM;

    // Flash usage percent
    float flash_used = ESP.getSketchSize();
    float FLASH_USAGE = flash_used * 100.0 / TOTAL_FLASH;

    // SD Card status and storage
    const char* SD_CARD = _sdCard.isAvailable() ? "Available" : "Not available";
    int PENDING_RECORDS = 0;
    if (_sdCard.isAvailable()) {
        PENDING_RECORDS = _sdCard.getPendingRecords();
    }

    _dateTime.begin(_serialAT);

    StaticJsonDocument<300> doc;
    doc["rec_at"] = _dateTime.asStr();
    doc["device_id"] = _config.getDeviceId();       
    doc["firmware"] = Config::FIRMWARE_VERSION;     
    doc["activated"] = _isActivated;
    doc["ram"] = RAM_USAGE;
    doc["flash"] = FLASH_USAGE;
    doc["sd"] = SD_CARD;
    doc["pending"] = PENDING_RECORDS;

    // Serialize to JSON
    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
}

String MqttUtils::generateNetworkHealthJson() {
    StaticJsonDocument<300> doc;
    unsigned long now = millis();

    doc["timestamp"] = now;
    doc["gsm_connected"] = _modem->isNetworkConnected();
    doc["mqtt_connected"] = _mqttClient.connected();
    doc["signal_quality"] = _networkHealth.signalQuality;        
    doc["gsm_uptime_ms"] = _networkHealth.gsmUptime;
    doc["mqtt_uptime_ms"] = _networkHealth.mqttUptime;
    doc["gsm_reconnects"] = _networkHealth.gsmReconnectCount;    
    doc["mqtt_reconnects"] = _networkHealth.mqttReconnectCount;
    doc["network_healthy"] = _networkHealth.networkHealthy;      

    if (_networkHealth.lastGsmDisconnect > 0) {
        doc["last_gsm_disconnect"] = _networkHealth.lastGsmDisconnect;
    }
    if (_networkHealth.lastMqttDisconnect > 0) {
        doc["last_mqtt_disconnect"] = _networkHealth.lastMqttDisconnect;
    }

    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
}

void MqttUtils::checkAndSyncData() {
    if (_modem->isNetworkConnected() && _mqttClient.connected() && _sdCard.getPendingRecords() > 0) {
        unsigned long currentMillis = millis();
        if (currentMillis - _lastSyncAttempt >= SYNC_INTERVAL) {
            _lastSyncAttempt = currentMillis;
            Serial.println("Auto-syncing offline data...");      
            syncOfflineData();
        }
    }
}

bool MqttUtils::syncOfflineData() {
    esp_task_wdt_reset(); // Reset at start

    SerialMon.println("Starting sync diagnostics...");
    SerialMon.printf("SD Card available: %s\n", _sdCard.isAvailable() ? "YES" : "NO");
    SerialMon.printf("MQTT connected: %s\n",_mqttClient.connected() ? "YES" : "NO");
    SerialMon.printf("Pending records: %d\n",_sdCard.isAvailable() ? _sdCard.getPendingRecords() : 0);        

    if (!_sdCard.isAvailable()) {
        SerialMon.println("SD card not available for syncing");
        return false;
    }

    if (!_mqttClient.connected()) {
        SerialMon.println("MQTT connection not available for syncing");
        return false;
    }

    SerialMon.println("Starting data synchronization...");       

    int syncedCount = 0;
    File root = _sdCard.openDataDir();
    if (!root) {
        SerialMon.println("Failed to open data directory");      
        return false;
    }

    if (!root.isDirectory()) {
        SerialMon.println("Data path is not a directory");       
        root.close();
        return false;
    }

    // Process up to 10 files at a time to avoid blocking        
    int processLimit = 10;
    int filesProcessed = 0;
    File file = root.openNextFile();
    while (file && processLimit > 0) {
        processLimit--;
        filesProcessed++;

        if (!file.isDirectory()) {
            String filename = file.name();
            String data = "";
            while (file.available()) {
                data += (char)file.read();
            }
            file.close();

            SerialMon.printf("Attempting to publish file: %s (%d bytes)\n", filename.c_str(), data.length());

            // Try publishing to weather data topic
            const char* weatherTopic = _config.getAwsIotDeviceDataTopic();
            bool publishResult = _mqttClient.publish(weatherTopic, data.c_str());

            if (publishResult) {
                SerialMon.printf("Successfully published %s, attempting to delete...\n", filename.c_str());
                if (_sdCard.deleteDataFile(filename.c_str())) {
                    syncedCount++;
                    SerialMon.printf("File %s deleted successfully\n", filename.c_str());
                } else {
                    SerialMon.printf("Failed to delete file %s after successful publish\n", filename.c_str());
                }
            } else {
                SerialMon.printf("Failed to publish data from file: %s\n", filename.c_str());
                delay(100);
            }
        } else {
            file.close();
        }

        file = root.openNextFile();

        // Reset watchdog every few files
        if (filesProcessed % 3 == 0) {
            esp_task_wdt_reset();
        }
    }

    root.close();
    SerialMon.printf("Synchronized %d files\n", syncedCount);    
    esp_task_wdt_reset();

    return syncedCount > 0;
}