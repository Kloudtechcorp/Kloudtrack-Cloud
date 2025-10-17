#include "AwsConnection.h"

AwsConnection::AwsConnection(SSLClient* sslClient, TinyGsm* modem)
      : _sslClient(sslClient),
        _modem(modem),
        _mqttClient(*sslClient),
        _isConnected(false)
    {
        // Initialize config and other components
        _config.begin();
    }



void AwsConnection::setUpAws() {
    esp_task_wdt_reset();

    // Get AWS Endpoint and Port
    const char *awsEndpoint = _config.getAwsIotEndpoint();
    int awsPort = _config.getAwsIotPort();

    SerialMon.println("-----------------------------------");
    SerialMon.println("Setting up AWS IoT connection...");
    SerialMon.printf("AWS Endpoint: %s:%d\n", awsEndpoint, awsPort);
    SerialMon.printf("Device ID: %s\n", _config.getDeviceId());

    // Verify GSM network is connected
    if (!_modem->isNetworkConnected()) {
        SerialMon.println("ERROR: GSM network not connected, cannot setup AWS");
        return;
    }

    // Configure MQTT server
    _mqttClient.setServer(awsEndpoint, awsPort);

    // Configure MQTT parameters for GSM connections
    _mqttClient.setSocketTimeout(20); // 20 second socket timeout for GSM
    _mqttClient.setKeepAlive(60); // 60 second keep alive
    _mqttClient.setBufferSize(1024); // Increase buffer size

    SerialMon.println("AWS IoT setup complete");
    SerialMon.println("-----------------------------------");

    esp_task_wdt_reset();
}

void AwsConnection::awsConnect() {
    setUpAws();

    // Verify GSM network is connected
    if (!_modem->isNetworkConnected()) {
        SerialMon.println("ERROR: GSM network not connected, cannot connect to AWS");
        return;
    }

    // Connect to AWS with retries
    const int maxRetries = 5;
    int awsRetryCount = 0;

    while (!_mqttClient.connected() && awsRetryCount < maxRetries) {
        awsRetryCount++;
        SerialMon.printf("AWS connection attempt %d/%d\n", awsRetryCount, maxRetries);
        esp_task_wdt_reset();

        if (_mqttClient.connect(_config.getDeviceId())) {
            SerialMon.println("AWS connected successfully!");
            _isConnected = true;
            break;
        } else {
            int state = _mqttClient.state();
            SerialMon.printf("AWS connection failed. MQTT state: %d - ", state);

            // Print specific error messages
            switch(state) {
                case -4: SerialMon.println("MQTT_CONNECTION_TIMEOUT"); break;
                case -3: SerialMon.println("MQTT_CONNECTION_LOST"); break;
                case -2: SerialMon.println("MQTT_CONNECT_FAILED"); break;
                case -1: SerialMon.println("MQTT_DISCONNECTED"); break;
                case 1: SerialMon.println("MQTT_CONNECT_BAD_PROTOCOL"); break;
                case 2: SerialMon.println("MQTT_CONNECT_BAD_CLIENT_ID"); break;
                case 3: SerialMon.println("MQTT_CONNECT_UNAVAILABLE"); break;
                case 4: SerialMon.println("MQTT_CONNECT_BAD_CREDENTIALS"); break;
                case 5: SerialMon.println("MQTT_CONNECT_UNAUTHORIZED"); break;
            }

            // Get SSL error details if available
            char sslErrorBuf[256];
            int sslErrorCode = _sslClient->lastError(sslErrorBuf, sizeof(sslErrorBuf));
            if (sslErrorCode != 0) {
                SerialMon.printf("SSL Error Code: %d - %s\n", sslErrorCode, sslErrorBuf);
            }

            esp_task_wdt_reset();
            if (awsRetryCount < maxRetries) {
                SerialMon.println("Waiting 5 seconds before retry...");
                delay(5000);
            }
        }
    }

    if (!_mqttClient.connected()) {
        SerialMon.println("Failed to connect to AWS after all retries!");
        SerialMon.println("Restarting ESP32 in 2 seconds...");
        delay(2000);
        ESP.restart();
    }

    esp_task_wdt_reset();

    // Subscribe to all relevant topics
    SerialMon.println("-----------------------------------");
    SerialMon.println("Subscribing to MQTT topics...");

    bool cmdSub = _mqttClient.subscribe(_config.getAwsIotDeviceCommandTopic());
    SerialMon.printf("Command topic: %s\n", cmdSub ? "SUCCESS" : "FAILED");

    bool weatherSub = _mqttClient.subscribe(_config.getAwsIotDeviceDataTopic());
    SerialMon.printf("Weather topic: %s\n", weatherSub ? "SUCCESS" : "FAILED");

    bool respSub = _mqttClient.subscribe(_config.getAwsIotDeviceResponseTopic());
    SerialMon.printf("Response topic: %s\n", respSub ? "SUCCESS" : "FAILED");

    SerialMon.println("-----------------------------------");

    esp_task_wdt_reset();
}

void AwsConnection::awsDisconnect() {
    if (_mqttClient.connected()) {
        _mqttClient.disconnect();
        SerialMon.println("AWS disconnected.");
    } else {
        SerialMon.println("AWS was not connected.");
    }
}

AwsConnection::~AwsConnection() {
    awsDisconnect();
    SerialMon.println("AWS connection closed.");
}

bool AwsConnection::isAwsConnected() {
    _isConnected = _mqttClient.connected();
    return _isConnected;
}