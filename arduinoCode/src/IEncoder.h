// include/IEncoder.h
#pragma once

class IEncoder {
public:
    virtual ~IEncoder() {}
    virtual void begin() {} // optional, for pin/interrupt setup
    virtual void end() {}   // optional, to detach interrupts
    virtual float getAngularVelocity() = 0; // rad/s
    virtual float getPosition() = 0;        // rad
    virtual void reset() = 0;
};