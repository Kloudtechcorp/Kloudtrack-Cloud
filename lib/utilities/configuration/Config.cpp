#include "Config.h"
#include "secrets.h"
#include <esp_task_wdt.h>

void Config::updateDeviceIdFromMac()
{
    uint64_t mac = ESP.getEfuseMac();
    sprintf(_deviceId, "KT-%04X%08X",
            (uint16_t)(mac >> 32),
            (uint32_t)mac);
}

void Config::updateAwsIotDeviceTopics()
{
    snprintf(_awsIotDeviceCommandTopic, 50, "kloudtrack/%s/command", _deviceId);
    snprintf(_awsIotDeviceDataTopic, 50, "kloudtrack/%s/data", _deviceId);
    snprintf(_awsIotDeviceResponseTopic, 50, "kloudtrack/%s/response", _deviceId);
}

void Config::loadDefaultCredentials()
{
    // Load default credentials from secrets.h
    strncpy(_wifiSsid, WIFI_SSID, sizeof(_wifiSsid) - 1);
    _wifiSsid[sizeof(_wifiSsid) - 1] = '\0';
    
    strncpy(_wifiPassword, WIFI_PASSWORD, sizeof(_wifiPassword) - 1);
    _wifiPassword[sizeof(_wifiPassword) - 1] = '\0';
    
    strncpy(_apn, APN, sizeof(_apn) - 1);
    _apn[sizeof(_apn) - 1] = '\0';
    
    strncpy(_awsIotEndpoint, AWS_IOT_ENDPOINT, sizeof(_awsIotEndpoint) - 1);
    _awsIotEndpoint[sizeof(_awsIotEndpoint) - 1] = '\0';
    
    _awsIotPort = AWS_IOT_PORT;
}

void Config::loadCredentialsFromPreferences()
{
    // Try to load WiFi credentials from preferences
    String storedSsid = _preferences.getString("wifi_ssid", "");
    String storedPassword = _preferences.getString("wifi_password", "");
    
    if (storedSsid.length() > 0 && storedPassword.length() > 0) {
        strncpy(_wifiSsid, storedSsid.c_str(), sizeof(_wifiSsid) - 1);
        _wifiSsid[sizeof(_wifiSsid) - 1] = '\0';
        strncpy(_wifiPassword, storedPassword.c_str(), sizeof(_wifiPassword) - 1);
        _wifiPassword[sizeof(_wifiPassword) - 1] = '\0';
        Serial.println("WiFi credentials loaded from preferences");
    } else {
        // Fallback to default credentials
        strncpy(_wifiSsid, WIFI_SSID, sizeof(_wifiSsid) - 1);
        _wifiSsid[sizeof(_wifiSsid) - 1] = '\0';
        strncpy(_wifiPassword, WIFI_PASSWORD, sizeof(_wifiPassword) - 1);
        _wifiPassword[sizeof(_wifiPassword) - 1] = '\0';
        Serial.println("Using default WiFi credentials");
    }
    
    // Try to load GSM credentials from preferences
    String storedApn = _preferences.getString("gsm_apn", "");
    if (storedApn.length() > 0) {
        strncpy(_apn, storedApn.c_str(), sizeof(_apn) - 1);
        _apn[sizeof(_apn) - 1] = '\0';
        Serial.println("GSM APN loaded from preferences");
    } else {
        // Fallback to default APN
        strncpy(_apn, APN, sizeof(_apn) - 1);
        _apn[sizeof(_apn) - 1] = '\0';
        Serial.println("Using default GSM APN");
    }
    
    // Try to load AWS credentials from preferences
    String storedEndpoint = _preferences.getString("aws_endpoint", "");
    int storedPort = _preferences.getInt("aws_port", -1);
    
    if (storedEndpoint.length() > 0 && storedPort > 0) {
        strncpy(_awsIotEndpoint, storedEndpoint.c_str(), sizeof(_awsIotEndpoint) - 1);
        _awsIotEndpoint[sizeof(_awsIotEndpoint) - 1] = '\0';
        _awsIotPort = storedPort;
        Serial.println("AWS credentials loaded from preferences");
    } else {
        // Fallback to default AWS credentials
        strncpy(_awsIotEndpoint, AWS_IOT_ENDPOINT, sizeof(_awsIotEndpoint) - 1);
        _awsIotEndpoint[sizeof(_awsIotEndpoint) - 1] = '\0';
        _awsIotPort = AWS_IOT_PORT;
        Serial.println("Using default AWS credentials");
    }
}

bool Config::setWifiCredentials(const char* ssid, const char* password)
{
    // Enhanced validation
    if (!ssid || !password) {
        Serial.println("WiFi credentials cannot be null");
        return false;
    }
    
    size_t ssidLen = strlen(ssid);
    size_t passLen = strlen(password);
    
    if (ssidLen == 0 || ssidLen > 32) {
        Serial.println("WiFi SSID must be 1-32 characters");
        return false;
    }
    
    if (passLen < 8 || passLen > 63) {
        Serial.println("WiFi password must be 8-63 characters");
        return false;
    }
    
    // Check for invalid characters in SSID
    for (size_t i = 0; i < ssidLen; i++) {
        if (ssid[i] < 32 || ssid[i] > 126) {
            Serial.println("WiFi SSID contains invalid characters");
            return false;
        }
    }
    
    // Store in preferences
    bool success = _preferences.putString("wifi_ssid", ssid) && 
                   _preferences.putString("wifi_password", password);
    
    if (success) {
        // Update current values
        strncpy(_wifiSsid, ssid, sizeof(_wifiSsid) - 1);
        _wifiSsid[sizeof(_wifiSsid) - 1] = '\0';
        strncpy(_wifiPassword, password, sizeof(_wifiPassword) - 1);
        _wifiPassword[sizeof(_wifiPassword) - 1] = '\0';
        Serial.printf("WiFi credentials updated: %s\n", ssid);
    } else {
        Serial.println("Failed to store WiFi credentials in preferences");
    }
    
    return success;
}

bool Config::setGsmCredentials(const char* apn)
{
    if (!apn) {
        Serial.println("GSM APN cannot be null");
        return false;
    }
    
    size_t apnLen = strlen(apn);
    if (apnLen == 0 || apnLen > 31) {
        Serial.println("GSM APN must be 1-31 characters");
        return false;
    }
    
    // Check for invalid characters in APN (should be alphanumeric, dots, hyphens)
    for (size_t i = 0; i < apnLen; i++) {
        char c = apn[i];
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || 
              (c >= '0' && c <= '9') || c == '.' || c == '-')) {
            Serial.println("GSM APN contains invalid characters (use alphanumeric, dots, hyphens only)");
            return false;
        }
    }
    
    // Store in preferences
    bool success = _preferences.putString("gsm_apn", apn);
    
    if (success) {
        // Update current values
        strncpy(_apn, apn, sizeof(_apn) - 1);
        _apn[sizeof(_apn) - 1] = '\0';
        Serial.printf("GSM APN updated: %s\n", apn);
    } else {
        Serial.println("Failed to store GSM APN in preferences");
    }
    
    return success;
}

bool Config::setAwsCredentials(const char* endpoint, int port)
{
    if (!endpoint) {
        Serial.println("AWS endpoint cannot be null");
        return false;
    }
    
    size_t endpointLen = strlen(endpoint);
    if (endpointLen == 0 || endpointLen > 127) {
        Serial.println("AWS endpoint must be 1-127 characters");
        return false;
    }
    
    if (port <= 0 || port > 65535) {
        Serial.println("AWS port must be between 1-65535");
        return false;
    }
    
    // Check for basic endpoint format (should contain dots and be hostname-like)
    bool hasDot = false;
    for (size_t i = 0; i < endpointLen; i++) {
        char c = endpoint[i];
        if (c == '.') {
            hasDot = true;
        } else if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || 
                    (c >= '0' && c <= '9') || c == '-')) {
            Serial.println("AWS endpoint contains invalid characters");
            return false;
        }
    }
    
    if (!hasDot) {
        Serial.println("AWS endpoint should be a valid hostname");
        return false;
    }
    
    // Store in preferences
    bool success = _preferences.putString("aws_endpoint", endpoint) && 
                   _preferences.putInt("aws_port", port);
    
    if (success) {
        // Update current values
        strncpy(_awsIotEndpoint, endpoint, sizeof(_awsIotEndpoint) - 1);
        _awsIotEndpoint[sizeof(_awsIotEndpoint) - 1] = '\0';
        _awsIotPort = port;
        Serial.printf("AWS credentials updated: %s:%d\n", endpoint, port);
    } else {
        Serial.println("Failed to store AWS credentials in preferences");
    }
    
    return success;
}

bool Config::hasWifiCredentials() const
{
    return strlen(_wifiSsid) > 0 && strlen(_wifiPassword) > 0;
}

bool Config::hasGsmCredentials() const
{
    return strlen(_apn) > 0;
}

bool Config::hasAwsCredentials() const
{
    return strlen(_awsIotEndpoint) > 0 && _awsIotPort > 0;
}

void Config::clearCredentials()
{
    _preferences.remove("wifi_ssid");
    _preferences.remove("wifi_password");
    _preferences.remove("gsm_apn");
    _preferences.remove("aws_endpoint");
    _preferences.remove("aws_port");
    
    // Reload default credentials
    loadDefaultCredentials();
    
    Serial.println("All credentials cleared, using defaults");
}

char* Config::loadCertificateFromFile(const char* filename) {
    File file = SPIFFS.open(filename, "r");
    if (!file) {
        return nullptr;
    }
    
    size_t size = file.size();
    if (size == 0 || size > 4096) { // Reasonable certificate size limit
        file.close();
        return nullptr;
    }
    
    char* buffer = (char*)malloc(size + 1);
    if (!buffer) {
        file.close();
        return nullptr;
    }
    
    size_t bytesRead = file.readBytes(buffer, size);
    buffer[bytesRead] = '\0';
    file.close();
    
    return buffer;
}

void Config::loadCertificatesFromSPIFFS()
{
    esp_task_wdt_reset(); // Reset at start of certificate loading
    _certsFromSPIFFS = false;
    
    // Try to load CA certificate from SPIFFS - check both old and new paths
    if (SPIFFS.exists("/ca_cert.pem")) {
        _caCert = loadCertificateFromFile("/ca_cert.pem");
        if (_caCert) {
            Serial.println("CA certificate loaded from SPIFFS");
            _certsFromSPIFFS = true;
        } else {
            Serial.println("Failed to load CA certificate from SPIFFS, using default");
            _caCert = (char*)AWS_CERT_CA;
        }
    } else {
        Serial.println("CA certificate not found in SPIFFS, using default");
        _caCert = (char*)AWS_CERT_CA;
    }
    esp_task_wdt_reset();
    
    // Try to load device certificate from SPIFFS - check new path first, then old path
    if (SPIFFS.exists("/cert/device.pem")) {
        _deviceCert = loadCertificateFromFile("/cert/device.pem");
        if (_deviceCert) {
            Serial.println("Device certificate loaded from SPIFFS (/cert/device.pem)");
            _certsFromSPIFFS = true;
        } else {
            Serial.println("Failed to load device certificate from /cert/device.pem");
            _deviceCert = (char*)AWS_CERT_CRT;
        }
    } else if (SPIFFS.exists("/device_cert.pem")) {
        _deviceCert = loadCertificateFromFile("/device_cert.pem");
        if (_deviceCert) {
            Serial.println("Device certificate loaded from SPIFFS (/device_cert.pem)");
            _certsFromSPIFFS = true;
        } else {
            Serial.println("Failed to load device certificate from SPIFFS, using default");
            _deviceCert = (char*)AWS_CERT_CRT;
        }
    } else {
        Serial.println("Device certificate not found in SPIFFS, using default");
        _deviceCert = (char*)AWS_CERT_CRT;
    }
    esp_task_wdt_reset();
    
    // Try to load private key from SPIFFS - check new path first, then old path
    if (SPIFFS.exists("/cert/private.pem")) {
        _privateKey = loadCertificateFromFile("/cert/private.pem");
        if (_privateKey) {
            Serial.println("Private key loaded from SPIFFS (/cert/private.pem)");
            _certsFromSPIFFS = true;
        } else {
            Serial.println("Failed to load private key from /cert/private.pem");
            _privateKey = (char*)AWS_CERT_PRIVATE;
        }
    } else if (SPIFFS.exists("/private_key.pem")) {
        _privateKey = loadCertificateFromFile("/private_key.pem");
        if (_privateKey) {
            Serial.println("Private key loaded from SPIFFS (/private_key.pem)");
            _certsFromSPIFFS = true;
        } else {
            Serial.println("Failed to load private key from SPIFFS, using default");
            _privateKey = (char*)AWS_CERT_PRIVATE;
        }
    } else {
        Serial.println("Private key not found in SPIFFS, using default");
        _privateKey = (char*)AWS_CERT_PRIVATE;
    }
    esp_task_wdt_reset(); // Reset at end of certificate loading
}

void Config::printCredentials()
{
    Serial.println("---------------------------------");
    Serial.println("Current Credentials");
    Serial.printf("WiFi SSID: %s\n", _wifiSsid);
    Serial.printf("WiFi Password: %s\n", strlen(_wifiPassword) > 0 ? "***SET***" : "NOT SET");
    Serial.printf("GSM APN: %s\n", _apn);
    Serial.printf("AWS Endpoint: %s\n", _awsIotEndpoint);
    Serial.printf("AWS Port: %d\n", _awsIotPort);
    Serial.printf("CA Certificate: %s\n", _caCert ? "LOADED" : "NOT SET");
    Serial.printf("Device Certificate: %s\n", _deviceCert ? "LOADED" : "NOT SET");
    Serial.printf("Private Key: %s\n", _privateKey ? "LOADED" : "NOT SET");
}

void Config::freeCertificates()
{
    if (_certsFromSPIFFS) {
        if (_caCert && _caCert != AWS_CERT_CA) {
            free(_caCert);
            _caCert = nullptr;
        }
        if (_deviceCert && _deviceCert != AWS_CERT_CRT) {
            free(_deviceCert);
            _deviceCert = nullptr;
        }
        if (_privateKey && _privateKey != AWS_CERT_PRIVATE) {
            free(_privateKey);
            _privateKey = nullptr;
        }
    }
    _certsFromSPIFFS = false;
}

Config::Config()
{
    // Initialize with default values
    memset(_wifiSsid, 0, sizeof(_wifiSsid));
    memset(_wifiPassword, 0, sizeof(_wifiPassword));
    memset(_apn, 0, sizeof(_apn));
    memset(_awsIotEndpoint, 0, sizeof(_awsIotEndpoint));
    _awsIotPort = 0;
    
    // Initialize certificate pointers
    _caCert = nullptr;
    _deviceCert = nullptr;
    _privateKey = nullptr;
    _certsFromSPIFFS = false;
}

Config::~Config()
{
    freeCertificates();
}