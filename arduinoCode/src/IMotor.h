// include/IMotor.h
#pragma once

class IMotor {
public:
    virtual ~IMotor() {}
    virtual void begin() {}         // optional, for pin setup
    virtual void enable() = 0;
    virtual void disable() = 0;
    // volts in [-supplyVoltage, +supplyVoltage]
    virtual void setVoltage(float volts) = 0;
};