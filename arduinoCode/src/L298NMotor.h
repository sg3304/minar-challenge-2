//
// Created by miket on 18-9-2025.
//

// include/L298NMotor.h
#pragma once
#include <Arduino.h>
#include "IMotor.h"

// H-bridge motor (e.g., L298N/L293D):
// - enPin: PWM speed control (ENA/ENB)
// - in1Pin, in2Pin: direction control
class L298NMotor : public IMotor {
public:
    L298NMotor(uint8_t enPin, uint8_t in1Pin, uint8_t in2Pin, float supplyVoltage, bool invert = false);
    void begin() override;
    void enable() override;
    void disable() override;
    void setVoltage(float volts) override;

private:
    uint8_t _enPin;
    uint8_t _in1Pin;
    uint8_t _in2Pin;
    float _supplyVoltage;
    bool _invert;
    bool _enabled;

    void writePWM(float duty); // 0..1
};