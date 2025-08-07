#include "Config.h"
#include "secrets.h"

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
    snprintf(_awsIotDeviceWeatherTopic, 50, "kloudtrack/%s/data", _deviceId);
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
    if (!ssid || !password || strlen(ssid) == 0 || strlen(password) == 0) {
        Serial.println("Invalid WiFi credentials provided");
        return false;
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
    if (!apn || strlen(apn) == 0) {
        Serial.println("Invalid GSM APN provided");
        return false;
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
    if (!endpoint || strlen(endpoint) == 0 || port <= 0) {
        Serial.println("Invalid AWS credentials provided");
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

void Config::printCredentials()
{
    Serial.println("=== Current Credentials ===");
    Serial.printf("WiFi SSID: %s\n", _wifiSsid);
    Serial.printf("WiFi Password: %s\n", strlen(_wifiPassword) > 0 ? "***SET***" : "NOT SET");
    Serial.printf("GSM APN: %s\n", _apn);
    Serial.printf("AWS Endpoint: %s\n", _awsIotEndpoint);
    Serial.printf("AWS Port: %d\n", _awsIotPort);
    Serial.println("==========================");
}

Config::Config()
{
    // Initialize with default values
    memset(_wifiSsid, 0, sizeof(_wifiSsid));
    memset(_wifiPassword, 0, sizeof(_wifiPassword));
    memset(_apn, 0, sizeof(_apn));
    memset(_awsIotEndpoint, 0, sizeof(_awsIotEndpoint));
    _awsIotPort = 0;
}