// src/ArduinoSerial.cpp
#include "ArduinoSerial.h"

ArduinoSerial::ArduinoSerial(HardwareSerial& port, size_t bufSize)
  : _port(port), _bufSize(bufSize) {
    _buffer.reserve(_bufSize);
}

void ArduinoSerial::begin(unsigned long baud) {
    _port.begin(baud);
}

bool ArduinoSerial::readLine(String& out) {
    while (_port.available() > 0) {
        char c = static_cast<char>(_port.read());
        if (c == '\r') continue;
        if (c == '\n') {
            out = _buffer;
            _buffer = "";
            return true;
        }
        if (_buffer.length() + 1 < _bufSize) {
            _buffer += c;
        } else {
            _buffer = ""; // overflow protection
        }
    }
    return false;
}

void ArduinoSerial::writeLine(const String& msg) {
    _port.println(msg);
}