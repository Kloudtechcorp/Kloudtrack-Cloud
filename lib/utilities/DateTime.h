#pragma once

#include <Arduino.h>

class DateTime
{
private:
    String _inner;

public:
    DateTime();
    DateTime(const String &year, const String &month, const String &day, const String &hour, const String &minute, const String &second);
    DateTime(int year, int month, int day, int hour, int minute, int second);

    inline const char *c_str() const
    {
        return _inner.c_str();
    }

    inline const String &asStr() const
    {
        return _inner;
    }
};
