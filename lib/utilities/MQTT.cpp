#include "MQTTClient.h"

// Static instance pointer for callback
MQTTClient* MQTTClient::_instance = nullptr;

MQTTClient::MQTTClient(Config* config, DeviceManager* deviceManager, OTAUpdater* otaUpdater, WeatherSensor* weatherSensor) :
    _config(config),
    _deviceManager(deviceManager),
    _otaUpdater(otaUpdater),
    _weatherSensor(weatherSensor),
    _client(_net),
    _retryCount(0),
    _maxRetries(config->getMaxRetries()),
    _reconnectDelay(config->getInitialReconnectDelay()),
    _maxReconnectDelay(config->getMaxReconnectDelay()) {
    
    // Store instance for static callback
    _instance = this;
}

void MQTTClient::connect() {
    // Configure WiFiClientSecure to use the AWS IoT device credentials
    _net.setCACert(_config->getAWSCertCA());
    _net.setCertificate(_config->getAWSCertCRT());
    _net.setPrivateKey(_config->getAWSCertPrivateKey());

    // Connect to the MQTT broker on AWS IoT
    _client.setServer(_config->getAWSEndpoint(), 8883);
    _client.setCallback(staticCallback);

    Serial.print("Connecting to AWS IoT Core...");
    
    if (!_client.connected()) {
        _retryCount++;
        Serial.printf("Connecting to AWS IoT. Attempt %d/%d\n", _retryCount, _maxRetries);

        if (_client.connect(_config->getDeviceId())) {
            Serial.println("Connected!");
            _retryCount = 0;
            _reconnectDelay = _config->getInitialReconnectDelay();
            
            // Subscribe to all relevant topics
            _client.subscribe(_config->getSubscribeTopic());
            _client.subscribe(_config->getActivationTopic());
            _client.subscribe(_config->getDeviceTopic());
            _client.subscribe(_config->getAllTopic());
            
            // Publish a startup message
            if (_deviceManager->isActivated()) {
                publishStatus("Online", "Device connected and ready for updates");
            } else {
                publishStatus("Inactive", "Device connected but requires activation");
            }
        } else {
            // Exponential backoff
            Serial.printf("Failed, rc=%d. Retrying in %d ms\n", _client.state(), _reconnectDelay);
            delay(_reconnectDelay);
            _reconnectDelay = min(_reconnectDelay * 2, _maxReconnectDelay);
        }
    }
    
    if (_retryCount >= _maxRetries) {
        Serial.println("Maximum MQTT retries reached. Restarting ESP32.");
        ESP.restart();
    }
}

bool MQTTClient::publish(const char* topic, const char* payload) {
    return _client.publish(topic, payload);
}

void MQTTClient::publishStatus(const char* status, const char* message) {
    StaticJsonDocument<200> doc;
    doc["device_id"] = _config->getDeviceId();
    doc["firmware_version"] = _config->getFirmwareVersion();
    doc["status"] = status;
    doc["message"] = message;
    doc["activated"] = _deviceManager->isActivated();
    doc["mac"] = _config->getDeviceId();
    
    char jsonBuffer[256];
    serializeJson(doc, jsonBuffer);
    
    // Publish to both general status topic and device-specific status topic
    _client.publish(_config->getStatusTopic(), jsonBuffer);
    
    char deviceStatusTopic[50];
    sprintf(deviceStatusTopic, "Kloudtrack/device/%s/status", _config->getDeviceId());
    _client.publish(deviceStatusTopic, jsonBuffer);
    
    Serial.printf("Published status: %s - %s\n", status, message);
}

// Static callback wrapper
void MQTTClient::staticCallback(char* topic, byte* payload, unsigned int length) {
    if (_instance) {
        _instance->messageHandler(topic, payload, length);
    }
}

void MQTTClient::messageHandler(char* topic, byte* payload, unsigned int length) {
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
    if (strcmp(topic, _config->getActivationTopic()) == 0) {
        _deviceManager->handleActivationCommand(doc, this);
    }
    else if (strcmp(topic, _config->getSubscribeTopic()) == 0 || 
             strcmp(topic, _config->getDeviceTopic()) == 0 || 
             strcmp(topic, _config->getAllTopic()) == 0) {
        handleCommand(doc);
    }
}

void MQTTClient::handleCommand(const JsonDocument& doc) {
    const char* command = doc["command"];
    
    if (!command) {
        Serial.println("Command field missing in message");
        return;
    }
    
    if (strcmp(command, "update") == 0) {
        _otaUpdater->handleUpdateCommand(doc, this);
    } 
    else if (strcmp(command, "status") == 0) {
        // Respond with current status when requested
        publishStatus("Info", "Status report requested");
    }
    else if (strcmp(command, "weather") == 0) {
        // Force publish weather data when requested
        _weatherSensor->publishData(this);
        publishStatus("Info", "Weather data published on demand");
    }
    else if (strcmp(command, "reset") == 0) {
        // Publish notification that device is restarting
        publishStatus("Resetting", "Device restarting per command request");
        
        Serial.println("Reset command received. Restarting device...");
        
        // Give time for the MQTT message to be sent
        delay(1000);
        
        // Restart the ESP32
        ESP.restart();
    }
    else {
        Serial.println("Unknown command received");
    }
}