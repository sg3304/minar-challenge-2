//
// Created by miket on 18-9-2025.
//

// src/L298NMotor.cpp
#include "L298NMotor.h"
#include <math.h>

L298NMotor::L298NMotor(uint8_t enPin, uint8_t in1Pin, uint8_t in2Pin, float supplyVoltage, bool invert)
  : _enPin(enPin), _in1Pin(in1Pin), _in2Pin(in2Pin),
    _supplyVoltage(supplyVoltage), _invert(invert), _enabled(false) {}

void L298NMotor::begin() {
    pinMode(_enPin, OUTPUT);
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);
    // Brake low and PWM 0
    digitalWrite(_in1Pin, LOW);
    digitalWrite(_in2Pin, LOW);
    writePWM(0.0f);
}

void L298NMotor::enable() {
    _enabled = true;
}

void L298NMotor::disable() {
    // Brake and PWM 0
    digitalWrite(_in1Pin, LOW);
    digitalWrite(_in2Pin, LOW);
    writePWM(0.0f);
    _enabled = false;
}

void L298NMotor::setVoltage(float volts) {
    if (!_enabled) {
        // Keep braked and no PWM
        digitalWrite(_in1Pin, LOW);
        digitalWrite(_in2Pin, LOW);
        writePWM(0.0f);
        return;
    }

    // Clamp to supply limits
    float v = volts;
    if (v > _supplyVoltage) v = _supplyVoltage;
    if (v < -_supplyVoltage) v = -_supplyVoltage;

    bool forward = (v >= 0.0f);
    if (_invert) forward = !forward;

    // Direction
    if (fabsf(v) < 1e-6f) {
        // Brake (both low) for zero command
        digitalWrite(_in1Pin, LOW);
        digitalWrite(_in2Pin, LOW);
        writePWM(0.0f);
        return;
    } else if (forward) {
        digitalWrite(_in1Pin, HIGH);
        digitalWrite(_in2Pin, LOW);
    } else {
        digitalWrite(_in1Pin, LOW);
        digitalWrite(_in2Pin, HIGH);
    }

    // PWM magnitude
    float duty = fabsf(v) / _supplyVoltage; // 0..1
    writePWM(duty);
}

void L298NMotor::writePWM(float duty) {
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    uint8_t value = static_cast<uint8_t>(duty * 255.0f + 0.5f);
    analogWrite(_enPin, value);
}