// include/SerialCommandHandler.h
#pragma once
#include <Arduino.h>
#include "ISerial.h"
#include "MotorSystem.h"

// Protocol (lines terminated with '\n'):
//  - SET <axis> <sp_rad_s>
//  - GET <axis> VEL
//  - ENABLE ALL
//  - DISABLE ALL
class SerialCommandHandler {
public:
    SerialCommandHandler(ISerial& serial, MotorSystem& system);
    void poll(); // non-blocking

private:
    ISerial& _serial;
    MotorSystem& _system;

    void parseAndApply(const String& cmd);
    static String trim(const String& s);
};