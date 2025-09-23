// src/MotorSystem.cpp
#include "MotorSystem.h"

MotorSystem::MotorSystem(MotorAxis& a0, MotorAxis& a1, MotorAxis& a2, MotorAxis& a3) {
    _axes[0] = &a0;
    _axes[1] = &a1;
    _axes[2] = &a2;
    _axes[3] = &a3;
}

bool MotorSystem::isValidIndex(int axis) const {
    return axis >= 0 && axis < 4;
}

void MotorSystem::setSetpoint(int axis, float spRadPerSec) {
    if (isValidIndex(axis)) {
        _axes[axis]->setSetpointRadPerSec(spRadPerSec);
    }
}

float MotorSystem::getVelocity(int axis) const {
    if (isValidIndex(axis)) {
        return _axes[axis]->getVelocity();
    }
    return 0.0f;
}

void MotorSystem::updateAll() {
    for (int i = 0; i < 4; ++i) {
        _axes[i]->update();
    }
}

void MotorSystem::enableAll() {
    for (int i = 0; i < 4; ++i) {
        _axes[i]->enable();
    }
}

void MotorSystem::disableAll() {
    for (int i = 0; i < 4; ++i) {
        _axes[i]->disable();
    }
}