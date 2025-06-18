#include "DateTime.h"

#include <Arduino.h>

unsigned long lastEpoch = 0; // Store last valid epoch timestamp

DateTime::DateTime() {}

// Function to convert date/time to UNIX timestamp
unsigned long convertToEpoch(const String& year, const String& month, const String& day, const String& hour, const String& minute, const String& second)
{
    struct tm t;
    t.tm_year = year.toInt() + 2000 - 1900;
    t.tm_mon = month.toInt() - 1;
    t.tm_mday = day.toInt();
    t.tm_hour = hour.toInt();
    t.tm_min = minute.toInt();
    t.tm_sec = second.toInt();
    return mktime(&t);
}

int DateTime::begin(HardwareSerial &serialAT)
{
    String response = "";
    serialAT.print("AT+CCLK?\r\n");
    delay(100);
    response = serialAT.readString();

    if (response == "")
    {
        return -1;
    }

    int startIndex = response.indexOf("+CCLK: \"");
    int endIndex = response.indexOf("\"", startIndex + 8);

    if (startIndex == -1 || endIndex == -1)
    {
        return -1;
    }

    String dateTimeString = response.substring(startIndex + 8, endIndex);

    int dayIndex = dateTimeString.indexOf("/");
    int monthIndex = dateTimeString.indexOf("/", dayIndex + 1);
    int yearIndex = dateTimeString.indexOf(",");

    String year = dateTimeString.substring(0, dayIndex);
    String month = dateTimeString.substring(dayIndex + 1, monthIndex);
    String day = dateTimeString.substring(monthIndex + 1, yearIndex);

    String timeString = dateTimeString.substring(yearIndex + 1);

    int hourIndex = timeString.indexOf(":");
    int minuteIndex = timeString.indexOf(":", hourIndex + 1);

    String hour = timeString.substring(0, hourIndex);
    String minute = timeString.substring(hourIndex + 1, minuteIndex);
    String second = timeString.substring(minuteIndex + 1);

    int plusIndex = second.indexOf("+");
    if (plusIndex != -1)
    {
        second = second.substring(0, plusIndex);
    }

    // Convert to epoch time
    unsigned long newEpoch = convertToEpoch(year, month, day, hour, minute, second);

    // Filtering: Ignore large jumps
    if (lastEpoch == 0 || abs((long)newEpoch - (long)lastEpoch) <= TIME_THRESHOLD)
    {
        lastEpoch = newEpoch;
        _inner = "20" + year + "-" + month + "-" + day + " " + hour + ":" + minute + ":" + second;
    }
    else
    {
        Serial.print("Time jump detected (");
        Serial.print(abs((long)newEpoch - (long)lastEpoch));
        Serial.println("s), ignoring...");
    }

    return 0;
}
