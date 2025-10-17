#pragma once

#include <Arduino.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <FS.h>
#include <SPIFFS.h>
#include <esp_task_wdt.h>
#include "configuration/secrets.h"
#include "configuration/Config.h"
#include "time/DateTime.h"
#include "storage/SdCard.h"
#include "sensors/TestSensorManager.h"
// #include "sensors/SensorManager.h"
#include "connectivity/GSM/GsmConnection.h"
#include "connectivity/AWS/AwsConnection.h"
#include "MQTT/MqttUtils.h"