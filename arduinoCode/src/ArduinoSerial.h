// include/ArduinoSerial.h
#pragma once
#include <Arduino.h>
#include "ISerial.h"

class ArduinoSerial : public ISerial {
public:
    explicit ArduinoSerial(HardwareSerial& port, size_t bufSize = 64);
    void begin(unsigned long baud);
    bool readLine(String& out) override;
    void writeLine(const String& msg) override;

private:
    HardwareSerial& _port;
    size_t _bufSize;
    String _buffer;
};