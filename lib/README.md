# ESP32 Weather Station with AWS IoT

This is a modular implementation of an ESP32-based weather station that connects to AWS IoT Core. The code has been structured using object-oriented programming principles to improve maintainability, readability, and extensibility.

## Architecture

The code is organized into the following modules:

### `main.cpp`
The entry point of the application. It initializes all components, handles setup, and manages the main loop.

### `Config.h/cpp`
Manages device configuration, including:
- Device ID generation based on MAC address
- MQTT topics
- AWS IoT endpoint and credentials
- Weather data parameters

### `DeviceManager.h/cpp`
Handles device activation state:
- Stores activation state in non-volatile memory
- Processes activation/deactivation commands
- Provides activation status to other components

### `WiFiManager.h/cpp`
Manages WiFi connectivity:
- Connects to configured WiFi network
- Implements retry logic with exponential backoff
- Handles reconnection when connection is lost

### `MQTTClient.h/cpp`
Manages MQTT communication with AWS IoT:
- Establishes secure connection to AWS IoT Core
- Subscribes to relevant topics
- Handles message routing
- Provides methods for publishing messages and status updates

### `OTAUpdater.h/cpp`
Handles over-the-air firmware updates:
- Processes update commands
- Downloads and verifies new firmware
- Applies updates and manages restarts

### `WeatherSensor.h/cpp`
Simulates weather data (in a real application, would interface with actual sensors):
- Generates simulated weather readings
- Formats data into JSON payloads
- Publishes data at configured intervals

### `secrets.h`
Stores sensitive configuration:
- WiFi credentials
- AWS IoT endpoint
- AWS IoT certificates and keys

## Dependencies

- Arduino framework
- WiFi library
- PubSubClient for MQTT
- ArduinoJson for JSON processing
- HTTPClient for OTA updates
- Update library for OTA processing
- Preferences library for persistent storage
- WiFiClientSecure for secure connections

## Usage

1. Copy `secrets.h.template` to `secrets.h` and fill in your credentials
2. Configure device-specific parameters in `Config.h`
3. Compile and upload to your ESP32 device
4. Activate the device by publishing to the activation topic with the correct key

## Features

- Secure connection to AWS IoT Core
- Device activation system
- Over-the-air (OTA) firmware updates
- Weather data publishing
- Status reporting
- Remote device management
- Resilient connection handling with retry mechanisms