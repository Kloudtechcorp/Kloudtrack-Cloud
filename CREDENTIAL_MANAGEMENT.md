# Credential Management System

This document explains how to use the new credential management system that allows you to fetch WiFi and GSM credentials from the preferences library instead of hardcoded values.

## Overview

The system now supports storing and retrieving credentials from the ESP32's non-volatile storage (preferences library) with automatic fallback to hardcoded default values if no custom credentials are stored.

## Features

- **WiFi Credentials**: Store and retrieve WiFi SSID and password
- **GSM Credentials**: Store and retrieve GSM APN settings
- **AWS Credentials**: Store and retrieve AWS IoT endpoint and port
- **Automatic Fallback**: Uses hardcoded defaults if no custom credentials are stored
- **Remote Management**: Update credentials via MQTT commands
- **Persistent Storage**: Credentials survive device restarts

## MQTT Commands

### Set WiFi Credentials
```json
{
  "command": "set_wifi",
  "ssid": "YourWiFiSSID",
  "password": "YourWiFiPassword"
}
```

### Set GSM APN
```json
{
  "command": "set_gsm",
  "apn": "internet"
}
```

### Set AWS Credentials
```json
{
  "command": "set_aws",
  "endpoint": "your-aws-endpoint.iot.region.amazonaws.com",
  "port": 8883
}
```

### Get Current Credentials
```json
{
  "command": "get_credentials"
}
```

Response:
```json
{
  "command": "credentials_status",
  "wifi_configured": true,
  "gsm_configured": true,
  "aws_configured": true,
  "wifi_ssid": "YourWiFiSSID",
  "gsm_apn": "internet",
  "aws_endpoint": "your-aws-endpoint.iot.region.amazonaws.com",
  "aws_port": 8883
}
```

### Clear All Credentials
```json
{
  "command": "clear_credentials"
}
```

## Usage Examples

### 1. Setting WiFi Credentials
Publish this message to your device's command topic:
```json
{
  "command": "set_wifi",
  "ssid": "MyHomeWiFi",
  "password": "MySecurePassword123"
}
```

### 2. Setting GSM APN for Different Carriers
For different carriers, you might need different APN settings:

**Globe (Philippines):**
```json
{
  "command": "set_gsm",
  "apn": "internet.globe.com.ph"
}
```

**Smart (Philippines):**
```json
{
  "command": "set_gsm",
  "apn": "internet"
}
```

**Generic:**
```json
{
  "command": "set_gsm",
  "apn": "internet"
}
```

### 3. Setting AWS IoT Endpoint
```json
{
  "command": "set_aws",
  "endpoint": "a68bn74ibyvu1-ats.iot.ap-southeast-1.amazonaws.com",
  "port": 8883
}
```

## Implementation Details

### Config Class Methods

The `Config` class now includes these new methods:

- `setWifiCredentials(const char* ssid, const char* password)`: Store WiFi credentials
- `setGsmCredentials(const char* apn)`: Store GSM APN
- `setAwsCredentials(const char* endpoint, int port)`: Store AWS credentials
- `getWifiSsid()`, `getWifiPassword()`: Retrieve WiFi credentials
- `getApn()`: Retrieve GSM APN
- `getAwsIotEndpoint()`, `getAwsIotPort()`: Retrieve AWS credentials
- `hasWifiCredentials()`, `hasGsmCredentials()`, `hasAwsCredentials()`: Check if credentials are configured
- `clearCredentials()`: Clear all stored credentials
- `printCredentials()`: Print current credentials to Serial (for debugging)

### Storage

Credentials are stored in the ESP32's non-volatile storage using the Preferences library with the namespace "credentials":

- `wifi_ssid`: WiFi SSID
- `wifi_password`: WiFi password
- `gsm_apn`: GSM APN
- `aws_endpoint`: AWS IoT endpoint
- `aws_port`: AWS IoT port

### Fallback Mechanism

If no custom credentials are stored in preferences, the system automatically falls back to the hardcoded values from `secrets.h`:

- `WIFI_SSID` and `WIFI_PASSWORD` for WiFi
- `APN` for GSM
- `AWS_IOT_ENDPOINT` and `AWS_IOT_PORT` for AWS

## Security Considerations

1. **Password Storage**: WiFi passwords are stored in plain text in the ESP32's flash memory. While this is not ideal for high-security environments, it's typical for IoT devices.

2. **MQTT Security**: All credential updates are sent via MQTT. Ensure your MQTT connection is properly secured with TLS/SSL.

3. **Device Activation**: Credential management commands require the device to be activated first.

## Troubleshooting

### Check Current Credentials
Use the `get_credentials` command to see what credentials are currently configured.

### Reset to Defaults
Use the `clear_credentials` command to remove all custom credentials and revert to hardcoded defaults.

### Serial Debug Output
During startup, the device prints the current credentials to the Serial monitor for debugging purposes.

## Migration from Hardcoded Credentials

If you're upgrading from the previous version that used only hardcoded credentials:

1. The system will automatically use the hardcoded values as defaults
2. You can then use the MQTT commands to set custom credentials
3. The device will use custom credentials if available, otherwise fall back to hardcoded values

## Version History

- **v2.3.4**: Added preferences-based credential management with MQTT commands
- **v2.3.3**: Previous version with hardcoded credentials only 