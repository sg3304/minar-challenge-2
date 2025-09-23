#include <Arduino.h>
#include "ArduinoSerial.h"
#include "DCMotor.h"
#include "L298NMotor.h"
#include "QuadratureEncoder.h"
#include "PIDController.h"
#include "MotorAxis.h"
#include "MotorSystem.h"
#include "SerialCommandHandler.h"

static constexpr float SUPPLY_VOLTAGE = 12.0f; // adjust to your hardware
static constexpr float PID_DT = 0.01f; // 10 ms = 100 Hz loop

// Example pin mapping (adjust to your board/wiring)
// Axis 0
static constexpr uint8_t M0_EN  = 6; // PWM pin driving motor speed for axis 0
static constexpr uint8_t M0_IN1 = 2; // Direction pin 1 for axis 0 motor
static constexpr uint8_t M0_IN2 = 3; // Direction pin 2 for axis 0 motor
static constexpr uint8_t E0_A   = 18; // Encoder channel A
static constexpr uint8_t E0_B   = -1; // Encoder channel B
static constexpr uint16_t E0_PPR = 11; // Encoder pulses per motor revolution

// Axis 1
static constexpr uint8_t M1_EN  = 9;  // PWM
static constexpr uint8_t M1_IN1 = 4;
static constexpr uint8_t M1_IN2 = 5;
static constexpr uint8_t E1_A   = 18;
static constexpr uint8_t E1_B   = 19;
static constexpr uint16_t E1_PPR = 11;

// Axis 2
static constexpr uint8_t M2_EN  = 10; // PWM
static constexpr uint8_t M2_IN1 = 7;
static constexpr uint8_t M2_IN2 = 8;
static constexpr uint8_t E2_A   = 20;
static constexpr uint8_t E2_B   = 21;
static constexpr uint16_t E2_PPR = 11;

// Axis 3
static constexpr uint8_t M3_EN  = 11; // PWM
static constexpr uint8_t M3_IN1 = 12;
static constexpr uint8_t M3_IN2 = 13;
static constexpr uint8_t E3_A   = 2;  // interrupt-capable
static constexpr uint8_t E3_B   = 5;
static constexpr uint16_t E3_PPR = 11;

ArduinoSerial SerialIO(Serial);

// Motors
L298NMotor motor0(M0_EN, M0_IN1, M0_IN2, SUPPLY_VOLTAGE, false);
L298NMotor motor1(M1_EN, M1_IN1, M1_IN2, SUPPLY_VOLTAGE, false);
L298NMotor motor2(M2_EN, M2_IN1, M2_IN2, SUPPLY_VOLTAGE, false);
L298NMotor motor3(M3_EN, M3_IN1, M3_IN2, SUPPLY_VOLTAGE, false);

// Encoders (quadrature x4)
QuadratureEncoder enc0(E0_A, E0_B, E0_PPR, false);
QuadratureEncoder enc1(E1_A, E1_B, E1_PPR, false);
QuadratureEncoder enc2(E2_A, E2_B, E2_PPR, false);
QuadratureEncoder enc3(E3_A, E3_B, E3_PPR, false);

// PID controllers (tune kp, ki, kd)
PIDController pid0(0.5f, 1.0f, 0.0f, PID_DT);
PIDController pid1(0.5f, 1.0f, 0.0f, PID_DT);
PIDController pid2(0.5f, 1.0f, 0.0f, PID_DT);
PIDController pid3(0.5f, 1.0f, 0.0f, PID_DT);

// Axes
MotorAxis axis0(motor0, enc0, pid0);
MotorAxis axis1(motor1, enc1, pid1);
MotorAxis axis2(motor2, enc2, pid2);
MotorAxis axis3(motor3, enc3, pid3);

// System and command handler
MotorSystem motorSystem(axis0, axis1, axis2, axis3);
SerialCommandHandler cmd(SerialIO, motorSystem);

void setup() {
    SerialIO.begin(9600);

    // Initialize hardware
    motor0.begin();
    motor1.begin();
    motor2.begin();
    motor3.begin();
    enc0.begin();
    enc1.begin();
    enc2.begin();
    enc3.begin();

    // Set safe defaults
    pid0.setOutputLimits(-SUPPLY_VOLTAGE, SUPPLY_VOLTAGE);
    pid1.setOutputLimits(-SUPPLY_VOLTAGE, SUPPLY_VOLTAGE);
    pid2.setOutputLimits(-SUPPLY_VOLTAGE, SUPPLY_VOLTAGE);
    pid3.setOutputLimits(-SUPPLY_VOLTAGE, SUPPLY_VOLTAGE);

    motorSystem.disableAll(); // start disabled; enable via serial "ENABLE"
}

void loop() {
    // Handle incoming serial commands
    cmd.poll();

    // Fixed-rate control loop at PID_DT
    static unsigned long lastMicros = micros();
    unsigned long now = micros();
    if ((now - lastMicros) >= static_cast<unsigned long>(PID_DT * 1e6f)) {
        lastMicros += static_cast<unsigned long>(PID_DT * 1e6f);
        motorSystem.updateAll();
    }
}
