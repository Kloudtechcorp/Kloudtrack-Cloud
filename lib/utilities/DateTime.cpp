#include "DateTime.h"

#include <Arduino.h>

String dateTimeStringFromParts(const String &year, const String &month, const String &day, const String &hour, const String &minute, const String &second)
{
    return year + "-" + month + "-" + day + " " + hour + ":" + minute + ":" + second;
}

DateTime::DateTime() : _inner("") {}

DateTime::DateTime(const String &year, const String &month, const String &day, const String &hour, const String &minute, const String &second) : _inner(dateTimeStringFromParts(year, month, day, hour, minute, second))
{
}

DateTime::DateTime(int year, int month, int day, int hour, int minute, int second) : _inner(dateTimeStringFromParts(String(year), String(month), String(day), String(hour), String(minute), String(second)))
{
}
