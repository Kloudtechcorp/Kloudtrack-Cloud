#pragma once

#include <FS.h>
#include <WString.h>

class SdCard
{
private:
    bool _isInitialized = false;

    int _pendingRecords = 0;

    bool beginInner();

public:
    SdCard();

    // Initializes SD card.
    //
    // Returns true if the SD card was initialized successfully.
    bool begin();

    // Returns if the SD card is available.
    inline bool isAvailable() const
    {
        return _isInitialized;
    }

    // Returns the number of pending records.
    inline int getPendingRecords() const
    {
        return _pendingRecords;
    }

    // Opens the data directory on the SD card.
    //
    // Returns a File object representing the data directory.
    File openDataDir();

    // Deletes the given file.
    //
    // Returns true if the file was deleted successfully.
    bool deleteDataFile(const String &filename);

    // Saves weather data to the SD card.
    //
    // Returns true if the data was saved successfully.
    bool saveWeatherData(const char *jsonData);
};
