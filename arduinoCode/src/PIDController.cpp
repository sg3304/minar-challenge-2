#include "PIDController.h"
// Removed <algorithm> to improve compatibility with some Arduino cores

PIDController::PIDController(float kp, float ki, float kd, float dtSeconds)
  : _kp(kp), _ki(ki), _kd(kd), _dt(dtSeconds),
    _setpoint(0.0f), _integral(0.0f), _prevError(0.0f),
    _outMin(-12.0f), _outMax(12.0f) {}

void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp; _ki = ki; _kd = kd;
}

void PIDController::setOutputLimits(float minOut, float maxOut) {
    // Avoid std::swap to prevent including <algorithm>
    if (minOut > maxOut) {
        float tmp = minOut;
        minOut = maxOut;
        maxOut = tmp;
    }
    _outMin = minOut;
    _outMax = maxOut;
}

void PIDController::setSetpoint(float spRadPerSec) {
    _setpoint = spRadPerSec;
}

float PIDController::getSetpoint() const {
    return _setpoint;
}

void PIDController::reset() {
    _integral = 0.0f;
    _prevError = 0.0f;
}

float PIDController::compute(float measuredRadPerSec) {
    float error = _setpoint - measuredRadPerSec;
    _integral += error * _dt;
    float derivative = (error - _prevError) / _dt;

    float out = _kp * error + _ki * _integral + _kd * derivative;

    if (out > _outMax) out = _outMax;
    if (out < _outMin) out = _outMin;

    _prevError = error;
    return out;
}