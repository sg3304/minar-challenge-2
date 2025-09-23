//
// Created by miket on 18-9-2025.
//

// Example usage with a single-channel Hall encoder on pin 2
// 11 pulses per motor rev, 1/30 gearbox -> 330 pulses per output-shaft rev
#include <Arduino.h>
#include "HallEncoder.h"

// Choose your input pin
constexpr uint8_t HALL_PIN = 2;
// Pulses per output shaft revolution
constexpr uint16_t PPR_OUTPUT = 330;

HallEncoder encoder(HALL_PIN, PPR_OUTPUT, /*invert=*/false);

void setup() {
    Serial.begin(115200);
    encoder.begin();

    // Set initial direction; update this when you change motor direction
    encoder.setDirection(+1);
}

void loop() {
    // Example: update direction according to how you drive your motor
    // encoder.setDirection(isForward ? +1 : -1);

    float pos_rad = encoder.getPosition();         // radians at output shaft
    float vel_rad_s = encoder.getAngularVelocity();// rad/s at output shaft

    Serial.print("pos(rad)="); Serial.print(pos_rad, 6);
    Serial.print("  vel(rad/s)="); Serial.println(vel_rad_s, 6);

    delay(50);
}