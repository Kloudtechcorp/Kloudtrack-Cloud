#include "SdCard.h"

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

// SD Card pins for A7670G module TF card interface
#define SD_MISO_PIN 2
#define SD_MOSI_PIN 15
#define SD_SCLK_PIN 14
#define SD_CS_PIN 13

#define SD_DATA_DIR "/kloudtrack"

SdCard::SdCard()
{
}

bool SdCard::beginInner()
{
    Serial.println("Initializing SD card...");

    // Configure SPI pins explicitly for this board (A7670G)
    SPI.begin(SD_SCLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

    // Initialize SD card with the CS pin
    SD.end();
    if (!SD.begin(SD_CS_PIN))
    {
        Serial.println("SD Card initialization failed!");
        return false;
    }

    Serial.println("SD Card initialized successfully");

    // Create data directory if it doesn't exist
    if (!SD.exists(SD_DATA_DIR))
    {
        if (SD.mkdir(SD_DATA_DIR))
        {
            Serial.printf("Created directory: %s\n", SD_DATA_DIR);
        }
        else
        {
            Serial.printf("Failed to create directory: %s\n", SD_DATA_DIR);
            return false;
        }
    }

    // Count existing data files
    _pendingRecords = 0;
    File root = SD.open(SD_DATA_DIR);
    if (root)
    {
        File file = root.openNextFile();
        while (file)
        {
            _pendingRecords++;
            file.close();
            file = root.openNextFile();
        }
        root.close();
    }

    Serial.printf("Found %d pending data records\n", _pendingRecords);
    return true;
}

bool SdCard::begin()
{
    if (_isInitialized)
    {
        return true;
    }

    _isInitialized = beginInner();
    return _isInitialized;
}

File SdCard::openDataDir()
{
    if (!_isInitialized)
    {
        return File();
    }

    return SD.open(SD_DATA_DIR);
}

bool SdCard::deleteDataFile(const String &filename)
{
    if (!_isInitialized)
    {
        return false;
    }

    String path = String(SD_DATA_DIR) + "/" + filename;
    if (!SD.remove(path))
    {
        Serial.printf("Failed to delete data file: %s\n", filename.c_str());
        return false;
    }

    Serial.printf("Deleted file: %s\n", filename.c_str());
    _pendingRecords--;

    return true;
}

// Generate a unique filename for data storage
String getNextDataFilename()
{
    char filename[32];
    sprintf(filename, "%s/data_%lu.json", SD_DATA_DIR, millis());
    return String(filename);
}

bool SdCard::saveWeatherData(const char *jsonData)
{
    if (!_isInitialized)
    {
        return false;
    }

    // Generate a filename
    String filename = getNextDataFilename();

    // Save the data
    File dataFile = SD.open(filename, FILE_WRITE);
    if (!dataFile)
    {
        Serial.printf("Failed to open file for writing: %s\n", filename.c_str());
        return false;
    }

    if (!dataFile.print(jsonData))
    {
        dataFile.close();
        Serial.printf("Failed to write to %s\n", filename.c_str());
        return false;
    }

    dataFile.close();

    Serial.printf("Data saved to %s\n", filename.c_str());
    _pendingRecords++;

    return true;
}
