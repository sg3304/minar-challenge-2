// include/MotorAxis.h
#pragma once
#include "IMotor.h"
#include "IEncoder.h"
#include "PIDController.h"

class MotorAxis {
public:
    MotorAxis(IMotor& motor, IEncoder& encoder, PIDController& pid);

    void setSetpointRadPerSec(float sp);
    float getSetpointRadPerSec() const;

    // Call at fixed sample time dt
    void update();

    void enable();
    void disable();

    float getVelocity() const; // rad/s
    float getPosition() const; // rad

private:
    IMotor& _motor;
    IEncoder& _encoder;
    PIDController& _pid;
    mutable float _lastVelocity;
};