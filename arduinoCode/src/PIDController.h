// include/PIDController.h
#pragma once

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float dtSeconds);
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float minOut, float maxOut);
    void setSetpoint(float spRadPerSec);
    float getSetpoint() const;
    void reset();
    // measured in rad/s; returns commanded voltage
    float compute(float measuredRadPerSec);

private:
    float _kp, _ki, _kd;
    float _dt;
    float _setpoint;
    float _integral;
    float _prevError;
    float _outMin, _outMax;
};