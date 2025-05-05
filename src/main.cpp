#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <Update.h>
#include "secrets.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WiFiClientSecure.h>

// MQTT Topics
char AWS_IOT_DEVICE_COMMAND_TOPIC[50];
char AWS_IOT_DEVICE_WEATHER_TOPIC[50];
char AWS_IOT_DEVICE_ACTIVATION_TOPIC[50];

// Current firmware version
#define FIRMWARE_VERSION "1.3.0"

// Activation key that admin must provide (should be unique per device)
#define ACTIVATION_KEY "KT-SECURE-KEY-12345"

// Weather data parameters
#define WEATHER_PUBLISH_INTERVAL 60000  // Publish weather data every minute
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

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);
Preferences preferences;
bool deviceActivated = false;
int wifiRetryCount = 0;
int mqttRetryCount = 0;
int maxRetries = 5;

// Add exponential backoff for connection attempts
unsigned long reconnectDelay = 1000; // Start with 1 second
const unsigned long maxReconnectDelay = 60000; // Cap at 1 minute

// Device ID will be set based on MAC address
char DEVICE_ID[20];

// Function to get MAC address as a string
void getMacAddress(char* macStr) {
    uint64_t mac = ESP.getEfuseMac();
    sprintf(macStr, "KT%08X", (uint32_t)mac);
}

// Function to check if device is activated
bool isDeviceActivated() {
    preferences.begin("kloudtrack", false);
    bool activated = preferences.getBool("activated", false);
    preferences.end();
    return activated;
}

// Function to activate the device
void activateDevice(bool activate) {
    preferences.begin("kloudtrack", false);
    preferences.putBool("activated", activate);
    preferences.end();
    deviceActivated = activate;
    
    // Publish activation status
    StaticJsonDocument<200> doc;
    doc["device_id"] = DEVICE_ID;
    doc["firmware_version"] = FIRMWARE_VERSION;
    doc["activated"] = activate;
    
    char jsonBuffer[256];
    serializeJson(doc, jsonBuffer);
    
    // Publish to device-specific activation topic
    sprintf(AWS_IOT_DEVICE_ACTIVATION_TOPIC, "kloudtrack/%s/activation", DEVICE_ID);
    client.publish(AWS_IOT_DEVICE_ACTIVATION_TOPIC, jsonBuffer);
    
    Serial.printf("Device %s\n", activate ? "ACTIVATED" : "DEACTIVATED");
}

void getMemoryStats(JsonObject& memoryStats) {
    // Set total sizes to match PlatformIO values
    const float TOTAL_RAM = 327680.0;       // bytes
    const float TOTAL_FLASH = 1310720.0;    // bytes

    // RAM usage percent (used = total - free heap)
    float used_ram = TOTAL_RAM - ESP.getFreeHeap();
    memoryStats["ram_usage_percent"] = used_ram * 100.0 / TOTAL_RAM;

    // Flash usage percent
    float flash_used = ESP.getSketchSize();
    memoryStats["flash_usage_percent"] = flash_used * 100.0 / TOTAL_FLASH;
}


// Function to publish status updates
void publishUpdateStatus(const char* status, const char* message) {
    StaticJsonDocument<200> doc;
    doc["device_id"] = DEVICE_ID;
    doc["firmware_version"] = FIRMWARE_VERSION;
    doc["status"] = status;
    doc["message"] = message;
    doc["activated"] = deviceActivated;
    
    // Add memory statistics
    JsonObject memoryStats = doc.createNestedObject("memory");
    getMemoryStats(memoryStats);
    
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);
    
    // Publish to device-specific status topic
    sprintf(AWS_IOT_DEVICE_COMMAND_TOPIC, "kloudtrack/%s/command", DEVICE_ID);
    client.publish(AWS_IOT_DEVICE_COMMAND_TOPIC, jsonBuffer);
    
    Serial.printf("Published status: %s - %s\n", status, message);
}

// Function to generate random weather data and publish it
void publishWeatherData() {
    // Generate random weather data
    float temperature = random(MIN_TEMPERATURE * 100, MAX_TEMPERATURE * 100) / 100.0;
    float humidity = random(MIN_HUMIDITY * 100, MAX_HUMIDITY * 100) / 100.0;
    float pressure = random(MIN_PRESSURE * 10, MAX_PRESSURE * 10) / 10.0;
    float windSpeed = random(MIN_WIND_SPEED * 100, MAX_WIND_SPEED * 100) / 100.0;
    
    // Get random wind direction (N, NE, E, SE, S, SW, W, NW)
    const char* windDirections[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    const char* windDirection = windDirections[random(0, 8)];
    
    // Get random weather condition
    const char* weatherConditions[] = {"Sunny", "Partly Cloudy", "Cloudy", "Rainy", "Thunderstorm", "Foggy", "Snowy"};
    const char* weatherCondition = weatherConditions[random(0, 7)];
    
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
    size_t msgLen = measureJson(doc) + 1;
    char* jsonBuffer = new char[msgLen];
    serializeJson(doc, jsonBuffer, msgLen);
    
    // Publish to device-specific weather topic
    sprintf(AWS_IOT_DEVICE_WEATHER_TOPIC, "kloudtrack/%s/data", DEVICE_ID);
    client.publish(AWS_IOT_DEVICE_WEATHER_TOPIC, jsonBuffer);
    delete[] jsonBuffer;
}

// Function to perform OTA update
bool performOTAUpdate(const char* url, const char* expectedChecksum) {
    // Check if device is activated
    if (!deviceActivated) {
        Serial.println("OTA update rejected: Device not activated");
        publishUpdateStatus("Rejected", "Device not activated");
        return false;
    }

    HTTPClient http;
    bool success = false;
    
    // Start the OTA update process
    Serial.println("Attempting to download firmware...");
    publishUpdateStatus("Downloading", "Starting firmware download");

    http.begin(net, url);
    http.addHeader("Content-Type", "application/octet-stream");

    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
        WiFiClient* stream = http.getStreamPtr();
        size_t total = http.getSize();

        // Prepare for OTA update
        if (Update.begin(total)) {
            size_t written = 0;
            
            // Read and write the stream
            uint8_t buffer[1024];
            while (http.connected() && (written < total)) {
                size_t available = stream->available();
                if (available) {
                    size_t bytesRead = stream->readBytes(buffer, min(available, sizeof(buffer)));
                    size_t bytesWritten = Update.write(buffer, bytesRead);
                    if (bytesWritten > 0) {
                        written += bytesWritten;
                        // Log progress (every 10%)
                        if (written % (total / 10) < 1024) {
                            Serial.printf("Progress: %d%%\n", (written * 100) / total);
                            char progressMsg[32];
                            sprintf(progressMsg, "Progress: %d%%", (written * 100) / total);
                            publishUpdateStatus("Downloading", progressMsg);
                        }
                    } else {
                        Serial.println("Error writing update");
                        publishUpdateStatus("Failed", "Error writing update");
                        break;
                    }
                }
                delay(1);
            }

            // Check if the update is complete
            if (written == total) {
                Serial.println("Firmware download complete, verifying...");
                publishUpdateStatus("Verifying", "Firmware download complete, verifying");
                
                // Verify update before finalizing
                if (Update.end()) {
                    if (Update.isFinished()) {
                        Serial.println("OTA Update successful, restarting...");
                        publishUpdateStatus("Success", "Update successful, restarting");
                        success = true;
                        // Give MQTT message time to send before restart
                        delay(1000);
                    } else {
                        Serial.println("OTA Update not finished");
                        publishUpdateStatus("Failed", "Update not finished properly");
                    }
                } else {
                    Serial.printf("Error during update finalization: %d\n", Update.getError());
                    publishUpdateStatus("Failed", "Error during update finalization");
                }
            } else {
                Serial.println("Firmware size mismatch");
                publishUpdateStatus("Failed", "Firmware size mismatch");
                Update.abort();
            }
        } else {
            Serial.println("Not enough space for update");
            publishUpdateStatus("Failed", "Not enough space for update");
        }
    } else {
        Serial.printf("HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
        char errorMsg[64];
        sprintf(errorMsg, "HTTP error: %d", httpCode);
        publishUpdateStatus("Failed", errorMsg);
    }

    http.end();
    return success;
}

// Handle activation commands
void handleActivationCommand(const JsonDocument& doc) {
    const char* action = doc["action"];
    const char* key = doc["key"];
    
    if (!action) {
        Serial.println("Invalid activation command: missing action");
        return;
    }
    
    if (strcmp(action, "activate") == 0) {
        // Check if key is correct
        if (!key || strcmp(key, ACTIVATION_KEY) != 0) {
            Serial.println("Activation rejected: Invalid key");
            publishUpdateStatus("Access_denied", "Invalid activation key");
            return;
        }
        
        activateDevice(true);
        Serial.println("Device activated successfully");
        publishUpdateStatus("Activated", "Device activated successfully");
    } 
    else if (strcmp(action, "deactivate") == 0) {
        // Check if key is correct
        if (!key || strcmp(key, ACTIVATION_KEY) != 0) {
            Serial.println("Deactivation rejected: Invalid key");
            publishUpdateStatus("Access_denied", "Invalid activation key");
            return;
        }
        
        activateDevice(false);
        Serial.println("Device deactivated");
        publishUpdateStatus("Deactivated", "Device deactivated");
    }
    else {
        Serial.println("Unknown activation action");
    }
    Serial.println("---------------------------------");
}

// Handle update commands
void handleUpdateCommand(const JsonDocument& doc) {
    // Check if device is activated before proceeding
    if (!deviceActivated) {
        Serial.println("Update command rejected: Device not activated");
        publishUpdateStatus("Rejected", "Device not activated");
        return;
    }
    
    const char* version = doc["version"];
    const char* url = doc["url"];
    const char* checksum = doc["checksum"];
    bool force = doc["force"] | false;
    
    // Check if we need to update
    if (!version || !url) {
        publishUpdateStatus("Error", "Missing version or URL in update command");
        return;
    }
    
    // Compare versions
    if (strcmp(version, FIRMWARE_VERSION) == 0 && !force) {
        Serial.println("Already running this version, update skipped");
        publishUpdateStatus("Skipped", "Already running the requested version");
        return;
    }
    
    Serial.printf("Starting update to version %s from %s\n", version, url);
    publishUpdateStatus("Started", "Starting firmware update");
    
    // Perform the update
    if (performOTAUpdate(url, checksum)) {
        ESP.restart();
    }
}

// MQTT message handler
void messageHandler(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message received on topic: ");
    Serial.println(topic);

    // Create a null-terminated string from the payload
    char* payloadStr = new char[length + 1];
    memcpy(payloadStr, payload, length);
    payloadStr[length] = '\0';
    Serial.println(payloadStr);

    // Parse JSON
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payloadStr);
    delete[] payloadStr;
    
    if (error) {
        Serial.print("DeserializeJson() failed: ");
        Serial.println(error.f_str());
        return;
    }

    // Handle based on topic
    if (strcmp(topic, AWS_IOT_DEVICE_ACTIVATION_TOPIC) == 0) {
        handleActivationCommand(doc);
    }
    // else if (strcmp(topic, AWS_IOT_DEVICE_WEATHER_TOPIC) == 0) {
    //     // Force publish weather data when requested
    //     publishWeatherData();
    //     // publishUpdateStatus("Info", "Weather data published on demand");
    // }
    else if (strcmp(topic, AWS_IOT_DEVICE_COMMAND_TOPIC) == 0) {
        const char* command = doc["command"];
        
        if (command && strcmp(command, "update") == 0) {
            handleUpdateCommand(doc);
        } 
        else if (command && strcmp(command, "status") == 0) {
            // Respond with current status when requested
            publishUpdateStatus("Info", "Status report requested");
        }
        else if (command && strcmp(command, "reset") == 0) {
            // Publish notification that device is restarting
            publishUpdateStatus("Resetting", "Device restarting per command request");
            
            Serial.println("Reset command received. Restarting device...");
            
            // Give time for the MQTT message to be sent
            delay(1000);
            
            // Restart the ESP32
            ESP.restart();
        }
        else {
            Serial.println("No command received");
        }
    }
    Serial.println("---------------------------------");
}

void connectWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Connecting to WiFi");
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        unsigned long wifiStart = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart < 10000)) {
          delay(500);
          Serial.print(".");
        }
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("\nWiFi connected!");
          wifiRetryCount = 0;
        } else {
          wifiRetryCount++;
          Serial.printf("\nWiFi connection failed. Attempt %d/%d\n", wifiRetryCount, maxRetries);
        }
        if (wifiRetryCount >= maxRetries) {
          Serial.println("Maximum WiFi retries reached. Restarting ESP32.");
          ESP.restart();
        }
    }
    Serial.println("---------------------------------");
}

void connectToAWS() {
    // Configure WiFiClientSecure to use the AWS IoT device credentials
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    // Connect to the MQTT broker on AWS IoT
    client.setServer(AWS_IOT_ENDPOINT, 8883);
    client.setCallback(messageHandler);

    Serial.println("Connecting to AWS IoT Core...");
    
    if (!client.connected()) {
        mqttRetryCount++;
        Serial.printf("Connecting to AWS IoT. Attempt %d/%d\n", mqttRetryCount, maxRetries);

        if (client.connect(DEVICE_ID)) {
            Serial.println("Connected!");
            mqttRetryCount = 0;
            reconnectDelay = 1000;
            
            // Subscribe to all relevant topics
            client.subscribe(AWS_IOT_DEVICE_ACTIVATION_TOPIC);
            client.subscribe(AWS_IOT_DEVICE_COMMAND_TOPIC);
            client.subscribe(AWS_IOT_DEVICE_WEATHER_TOPIC);
            
            // Check activation status
            deviceActivated = isDeviceActivated();
            
            // Publish a startup message
            if (deviceActivated) {
                publishUpdateStatus("Online", "Device connected and ready for updates");
            } else {
                publishUpdateStatus("Inactive", "Device connected but requires activation");
            }
        } else {
            // Exponential backoff
            Serial.printf("Failed, rc=%d. Retrying in %d ms\n", client.state(), reconnectDelay);
            delay(reconnectDelay);
            reconnectDelay = min(reconnectDelay * 2, maxReconnectDelay);
        }
    }
    if (mqttRetryCount >= maxRetries) {
        Serial.println("Maximum MQTT retries reached. Restarting ESP32.");
        ESP.restart();
    }
    Serial.println("---------------------------------");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Generate unique device ID based on MAC address
    getMacAddress(DEVICE_ID);
    
    // Configure device-specific topics
    sprintf(AWS_IOT_DEVICE_ACTIVATION_TOPIC, "kloudtrack/%s/activation", DEVICE_ID);
    sprintf(AWS_IOT_DEVICE_COMMAND_TOPIC, "kloudtrack/%s/command", DEVICE_ID);
    sprintf(AWS_IOT_DEVICE_WEATHER_TOPIC, "kloudtrack/%s/data", DEVICE_ID);

    Serial.println("\n\n---------------------------------");
    Serial.println("ESP32 Weather Station with AWS IoT 2");
    Serial.printf("Device ID: %s\n", DEVICE_ID);
    Serial.printf("Current Firmware Version: %s\n", FIRMWARE_VERSION);
    Serial.println("---------------------------------");

    // Connect to WiFi
    connectWiFi();

    // Check activation status
    deviceActivated = isDeviceActivated();
    Serial.printf("Device activation status: %s\n", deviceActivated ? "ACTIVATED" : "NOT ACTIVATED");

    // Connect to AWS IoT
    connectToAWS();
    lastWeatherPublish = millis();
}

void loop() {
    unsigned long currentMillis = millis();

    // Maintain WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
        connectWiFi();
    }

    // Maintain MQTT connection
    if (!client.connected()) {
        connectToAWS();
    }
    
    // Process MQTT messages
    client.loop();

    // Publish weather data at regular intervals
    if (currentMillis - lastWeatherPublish >= WEATHER_PUBLISH_INTERVAL) {
        lastWeatherPublish = currentMillis;
        if (deviceActivated) {
            publishWeatherData();
        }
    }
    delay(10);
}