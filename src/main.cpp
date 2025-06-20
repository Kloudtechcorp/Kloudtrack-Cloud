/*
  Version 2.3.4
  - Store credentials in Preferences
*/

// #define USE_WIFI // Uncomment to use WiFi, comment to use GSM 

#ifdef USE_WIFI
#include "WifiClient.h"
WifiClient client;
#else
#include "GsmClient.h"
GsmClient client;
#endif

#include <esp_task_wdt.h>

void setup()
{
  esp_task_wdt_init(60, true); // 30s watchdog
  Serial.begin(115200);
  delay(1000);

  client.begin();
}

void loop()
{
  client.loop();
}