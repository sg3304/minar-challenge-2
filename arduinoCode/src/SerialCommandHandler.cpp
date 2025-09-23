// src/SerialCommandHandler.cpp
#include "SerialCommandHandler.h"

SerialCommandHandler::SerialCommandHandler(ISerial& serial, MotorSystem& system)
  : _serial(serial), _system(system) {}

void SerialCommandHandler::poll() {
    String line;
    if (_serial.readLine(line)) {
        parseAndApply(line);
    }
}

String SerialCommandHandler::trim(const String& s) {
    int start = 0;
    while (start < (int)s.length() && isspace(s[start])) ++start;
    int end = s.length() - 1;
    while (end >= start && isspace(s[end])) --end;
    if (end < start) return String();
    return s.substring(start, end + 1);
}

void SerialCommandHandler::parseAndApply(const String& cmd) {
    String s = trim(cmd);
    if (s.length() == 0) return;

    int p1 = s.indexOf(' ');
    String op = (p1 < 0) ? s : s.substring(0, p1);
    op.toUpperCase();

    if (op == "SET") {
        int p2 = s.indexOf(' ', p1 + 1);
        if (p1 < 0 || p2 < 0) {
            _serial.writeLine("ERR Bad SET");
            return;
        }
        int axis = s.substring(p1 + 1, p2).toInt();
        float sp = s.substring(p2 + 1).toFloat();
        _system.setSetpoint(axis, sp);
        _serial.writeLine("OK");
    } else if (op == "GET") {
        int p2 = s.indexOf(' ', p1 + 1);
        if (p1 < 0 || p2 < 0) {
            _serial.writeLine("ERR Bad GET");
            return;
        }
        int axis = s.substring(p1 + 1, p2).toInt();
        String what = s.substring(p2 + 1);
        what.toUpperCase();
        if (what == "VEL") {
            _serial.writeLine(String(_system.getVelocity(axis), 6));
        } else {
            _serial.writeLine("ERR Unknown GET");
        }
    } else if (op == "ENABLE") {
        _system.enableAll();
        _serial.writeLine("OK");
    } else if (op == "DISABLE") {
        _system.disableAll();
        _serial.writeLine("OK");
    } else {
        _serial.writeLine("ERR Unknown");
    }
}