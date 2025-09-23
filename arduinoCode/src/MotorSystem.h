// include/MotorSystem.h
#pragma once
#include "MotorAxis.h"

class MotorSystem {
public:
    MotorSystem(MotorAxis& a0, MotorAxis& a1, MotorAxis& a2, MotorAxis& a3);

    void setSetpoint(int axis, float spRadPerSec);
    float getVelocity(int axis) const;

    void updateAll();
    void enableAll();
    void disableAll();

private:
    MotorAxis* _axes[4];
    bool isValidIndex(int axis) const;
};