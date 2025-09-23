// include/ISerial.h
#pragma once
#include <Arduino.h>

class ISerial {
public:
    virtual ~ISerial() {}
    // Non-blocking: returns true if a full line (ending with \n) is read
    virtual bool readLine(String& out) = 0;
    virtual void writeLine(const String& msg) = 0;
};