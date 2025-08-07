# Kloudtrack Cloud

## Building

1. Install VSCode and the PlatformIO extension (`platformio.platformio-ide`).
2. Run `Git: Clone` command in VSCode with `git@github.com:Kloudtechcorp/Kloudtrack-Cloud.git` as the repository URL to clone the repository.
3. Once the project is loaded and the dependencies from the `platformio.ini` file are installed, run `PlatformIO: Build` to build the project.

*Note: When getting "Please configure IDF framework to include mbedTLS -> Enable pre-shared-key ciphersuites and activate at least one cipher" error when compiling, see [this commit](https://github.com/gravitech-engineer/AIS_IoT_4G/pull/8/commits/11a26867f73f45a54e46d8132b264b4eb5ff93ad).*

## Uploading

1. Make sure the device is connected to the computer and detectable by PlatformIO.
2. Run `PlatformIO: Upload` or `PlatformIO: Upload and Monitor` in VSCode to upload the firmware to the device.

## Features

### Credential Management (v2.3.4+)

The system now supports dynamic credential management through the preferences library with MQTT commands:

- **WiFi Credentials**: Set and retrieve WiFi SSID and password
- **GSM Credentials**: Set and retrieve GSM APN settings  
- **AWS Credentials**: Set and retrieve AWS IoT endpoint and port
- **Automatic Fallback**: Uses hardcoded defaults if no custom credentials are stored
- **Remote Management**: Update credentials via MQTT commands
- **Persistent Storage**: Credentials survive device restarts

For detailed usage instructions, see [CREDENTIAL_MANAGEMENT.md](CREDENTIAL_MANAGEMENT.md).

### Quick Start - Setting Credentials

1. Activate your device first
2. Use MQTT commands to set credentials:

```json
// Set WiFi
{"command": "set_wifi", "ssid": "YourWiFi", "password": "YourPassword"}

// Set GSM APN
{"command": "set_gsm", "apn": "internet"}

// Set AWS
{"command": "set_aws", "endpoint": "your-endpoint.iot.region.amazonaws.com", "port": 8883}

// Get current credentials
{"command": "get_credentials"}
```

See the test script `test_credentials.py` for complete examples.
