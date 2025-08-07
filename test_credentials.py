#!/usr/bin/env python3
"""
Test script for the new credential management system.
This script demonstrates how to use MQTT commands to manage device credentials.
"""

import json
import time
from paho.mqtt import client as mqtt_client

# Configuration - Update these values for your setup
BROKER = "your-mqtt-broker.com"
PORT = 8883
CLIENT_ID = "credential_test_client"
USERNAME = "your-username"
PASSWORD = "your-password"
DEVICE_TOPIC = "kloudtrack/KT-12345678/command"  # Update with your device ID

def connect_mqtt():
    """Connect to MQTT broker"""
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print(f"Failed to connect, return code {rc}")

    client = mqtt_client.Client(CLIENT_ID)
    client.username_pw_set(USERNAME, PASSWORD)
    client.on_connect = on_connect
    client.connect(BROKER, PORT)
    return client

def publish_command(client, command_data):
    """Publish a command to the device"""
    message = json.dumps(command_data)
    result = client.publish(DEVICE_TOPIC, message)
    status = result[0]
    if status == 0:
        print(f"✓ Command sent: {command_data['command']}")
    else:
        print(f"✗ Failed to send command: {command_data['command']}")
    return result

def test_credential_management():
    """Test the credential management functionality"""
    client = connect_mqtt()
    client.loop_start()
    
    print("=== Credential Management Test ===\n")
    
    # 1. Get current credentials
    print("1. Getting current credentials...")
    publish_command(client, {"command": "get_credentials"})
    time.sleep(2)
    
    # 2. Set WiFi credentials
    print("\n2. Setting WiFi credentials...")
    wifi_credentials = {
        "command": "set_wifi",
        "ssid": "TestWiFiNetwork",
        "password": "TestPassword123"
    }
    publish_command(client, wifi_credentials)
    time.sleep(2)
    
    # 3. Set GSM APN
    print("\n3. Setting GSM APN...")
    gsm_credentials = {
        "command": "set_gsm",
        "apn": "internet.globe.com.ph"
    }
    publish_command(client, gsm_credentials)
    time.sleep(2)
    
    # 4. Set AWS credentials
    print("\n4. Setting AWS credentials...")
    aws_credentials = {
        "command": "set_aws",
        "endpoint": "test-endpoint.iot.ap-southeast-1.amazonaws.com",
        "port": 8883
    }
    publish_command(client, aws_credentials)
    time.sleep(2)
    
    # 5. Get updated credentials
    print("\n5. Getting updated credentials...")
    publish_command(client, {"command": "get_credentials"})
    time.sleep(2)
    
    # 6. Clear all credentials (optional - uncomment to test)
    # print("\n6. Clearing all credentials...")
    # publish_command(client, {"command": "clear_credentials"})
    # time.sleep(2)
    
    print("\n=== Test Complete ===")
    print("Check the device's serial output and MQTT responses for results.")
    
    client.loop_stop()
    client.disconnect()

def test_individual_commands():
    """Test individual credential commands"""
    client = connect_mqtt()
    client.loop_start()
    
    print("=== Individual Command Tests ===\n")
    
    # Test WiFi only
    print("Testing WiFi credentials...")
    publish_command(client, {
        "command": "set_wifi",
        "ssid": "MyHomeWiFi",
        "password": "SecurePassword123"
    })
    time.sleep(1)
    
    # Test GSM only
    print("Testing GSM APN...")
    publish_command(client, {
        "command": "set_gsm",
        "apn": "internet"
    })
    time.sleep(1)
    
    # Test AWS only
    print("Testing AWS credentials...")
    publish_command(client, {
        "command": "set_aws",
        "endpoint": "a68bn74ibyvu1-ats.iot.ap-southeast-1.amazonaws.com",
        "port": 8883
    })
    time.sleep(1)
    
    # Get status
    print("Getting credential status...")
    publish_command(client, {"command": "get_credentials"})
    
    client.loop_stop()
    client.disconnect()

if __name__ == "__main__":
    print("Credential Management Test Script")
    print("=================================")
    print("Make sure to update the configuration variables at the top of this script.")
    print("Also ensure your device is activated before running these tests.\n")
    
    # Uncomment the test you want to run:
    test_credential_management()
    # test_individual_commands() 