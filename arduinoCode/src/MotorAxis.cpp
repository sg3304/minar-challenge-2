//
// Created by miket on 18-9-2025.
//

// src/MotorAxis.cpp
#include "MotorAxis.h"

MotorAxis::MotorAxis(IMotor& motor, IEncoder& encoder, PIDController& pid)
  : _motor(motor), _encoder(encoder), _pid(pid), _lastVelocity(0.0f) {}

void MotorAxis::setSetpointRadPerSec(float sp) {
  _pid.setSetpoint(sp);
}

float MotorAxis::getSetpointRadPerSec() const {
  return _pid.getSetpoint();
}

void MotorAxis::update() {
  float vel = _encoder.getAngularVelocity();
  _lastVelocity = vel;
  float cmdVolts = _pid.compute(vel);
  _motor.setVoltage(cmdVolts);
}

void MotorAxis::enable() {
  _motor.enable();
}

void MotorAxis::disable() {
  _motor.disable();
}

float MotorAxis::getVelocity() const {
  return _lastVelocity;
}

float MotorAxis::getPosition() const {
  return _encoder.getPosition();
}