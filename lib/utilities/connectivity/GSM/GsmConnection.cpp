#include "GsmConnection.h"

GsmConnection::GsmConnection()
    : SerialAT(1),
      _modem(SerialAT),
      _baseClient(_modem),
      _sslClient(&_baseClient),
      _isConnected(false)
{}

uint32_t GsmConnection::AutoBaud() {
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

void GsmConnection::setupModem() {
    esp_task_wdt_reset(); // Reset watchdog at start
    SerialMon.println("Initializing GSM modem...");
    
    // A7670-GSM Reset
    pinMode(RESET_PIN, OUTPUT);
    digitalWrite(RESET_PIN, LOW); delay(100);
    esp_task_wdt_reset();
    digitalWrite(RESET_PIN, HIGH); delay(100);
    digitalWrite(RESET_PIN, LOW); delay(100);

    // A7670-GSM Power
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, LOW); delay(100);
    digitalWrite(POWER_PIN, HIGH); delay(100);
    digitalWrite(POWER_PIN, LOW); delay(1000);  // Increased delay
    esp_task_wdt_reset();

    Serial.println("Starting Serial Communications...");
    SerialAT.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
    _modem.init();
    esp_task_wdt_reset();
    
    AutoBaud();
    delay(2000); // Wait for the modem to initialize
    esp_task_wdt_reset();
}

void GsmConnection::gsmConnect() {
    setupModem();

    // Get APN from config with fallback
    const char *apn = _config.getApn();

    // If APN is empty or null, use "internet" as fallback
    if (!apn || strlen(apn) == 0) {
        apn = "internet";
        SerialMon.println("No APN configured, using fallback: 'internet'");
    }

    SerialMon.println("Connecting to cellular network...");
    SerialMon.printf("Using APN: '%s'\n", apn);

    // Get modem info for debugging
    SerialMon.println("Getting modem information...");
    String modemInfo = _modem.getModemInfo();
    SerialMon.printf("Modem Info: %s\n", modemInfo.c_str());

    String imei = _modem.getIMEI();
    SerialMon.printf("IMEI: %s\n", imei.c_str());

    // Check network registration
    SerialMon.println("Checking network registration...");
    int regStatus = _modem.getRegistrationStatus();
    SerialMon.printf("Registration Status: %d\n", regStatus);

    // Check signal quality
    int signalQuality = _modem.getSignalQuality();
    SerialMon.printf("Signal Quality: %d dBm\n", signalQuality);

    // Connect to network with retries
    const int maxRetries = 5;
    int gsmRetryCount = 0;

    while (!_modem.isNetworkConnected() && gsmRetryCount < maxRetries) {
        gsmRetryCount++;
        SerialMon.printf("GSM connection attempt %d/%d\n", gsmRetryCount, maxRetries);

        // Wait for network registration
        SerialMon.println("Waiting for network registration...");
        if (_modem.waitForNetwork(30000)) {
            SerialMon.println("Network registered successfully");

            SerialMon.printf("Attempting GPRS connection with APN: %s\n", apn);
            if (_modem.gprsConnect(apn)) {
                SerialMon.println("GPRS connected successfully!");

                // Get IP address
                String localIP = _modem.getLocalIP();
                SerialMon.printf("Local IP: %s\n", localIP.c_str());

                _isConnected = true;
                break;
            } else {
                SerialMon.println("GPRS connection failed");
            }
        } else {
            SerialMon.println("Failed to register on cellular network");
        }

        esp_task_wdt_reset();
        if (gsmRetryCount < maxRetries) {
            SerialMon.println("Waiting before retry...");
            delay(5000);
        }
    }

    if (!_modem.isNetworkConnected()) {
        SerialMon.println("Failed to connect to cellular network after all retries! Restarting after 2 seconds...");
        delay(2000);
        ESP.restart();
    }

    // Configure SSL certificates after successful GSM connection
    configureSSL();
}

void GsmConnection::configureSSL() {
    SerialMon.println("-----------------------------------");
    SerialMon.println("Configuring SSL certificates...");

    const char* caCert = _config.getCaCert();
    const char* deviceCert = _config.getDeviceCert();
    const char* privateKey = _config.getPrivateKey();

    // Validate certificates
    bool certsValid = true;
    if (!caCert || strlen(caCert) < 100) {
        SerialMon.println("ERROR: CA certificate invalid or missing");
        certsValid = false;
    } else {
        SerialMon.printf("CA certificate: %d bytes\n", strlen(caCert));
    }

    if (!deviceCert || strlen(deviceCert) < 100) {
        SerialMon.println("ERROR: Device certificate invalid or missing");
        certsValid = false;
    } else {
        SerialMon.printf("Device certificate: %d bytes\n", strlen(deviceCert));
    }

    if (!privateKey || strlen(privateKey) < 100) {
        SerialMon.println("ERROR: Private key invalid or missing");
        certsValid = false;
    } else {
        SerialMon.printf("Private key: %d bytes\n", strlen(privateKey));
    }

    if (!certsValid) {
        SerialMon.println("Certificate validation failed - SSL connections will fail");
        return;
    }

    // Stop any existing SSL connection
    _sslClient.stop();
    delay(500);

    // Configure SSL client settings
    _sslClient.setHandshakeTimeout(30000); // 30 second timeout for GSM

    // Set certificates
    SerialMon.println("Setting CA certificate...");
    _sslClient.setCACert(caCert);

    SerialMon.println("Setting device certificate...");
    _sslClient.setCertificate(deviceCert);

    SerialMon.println("Setting private key...");
    _sslClient.setPrivateKey(privateKey);

    SerialMon.println("SSL certificates configured successfully");
    SerialMon.println("-----------------------------------");
}

void GsmConnection::gsmDisconnect() {
    if (_modem.isGprsConnected()) {
        _modem.gprsDisconnect();
        SerialMon.println("GSM disconnected.");
    } else {
        SerialMon.println("GSM was not connected.");
    }
}

GsmConnection::~GsmConnection() {
    gsmDisconnect();
    SerialAT.end();
    SerialMon.println("GSM connection closed.");
}

bool GsmConnection::isGsmConnected() {
    _isConnected = _modem.isGprsConnected();
    return _isConnected;
}