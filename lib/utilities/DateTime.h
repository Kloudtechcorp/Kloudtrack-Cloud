#pragma once

#include <Arduino.h>

class DateTime
{
private:
    static constexpr int TIME_THRESHOLD = 120; // Allow up to 120s jump

    String _inner = "";

public:
    DateTime();

    int begin(HardwareSerial &serial);

    inline const char *c_str() const
    {
        return _inner.c_str();
    }

    inline const String &asStr() const
    {
        return _inner;
    }
};
