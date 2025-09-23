#include <Arduino.h>

// Motor 1A control pins
#define MOTOR1A_IN1 2
#define MOTOR1A_IN2 3
#define MOTOR1A_ENA 6
int MOTOR1A_PWM = 0;

// Motor 1B control pins
#define MOTOR1B_IN3 4
#define MOTOR1B_IN4 5
#define MOTOR1B_ENB 7
int MOTOR1B_PWM = 0;

void setup() {
    Serial.begin(9600);
    // Initialize Motor 1A
    pinMode(MOTOR1A_IN1, OUTPUT);
    pinMode(MOTOR1A_IN2, OUTPUT);
    pinMode(MOTOR1A_ENA, OUTPUT);

    // Initialize Motor 1B
    pinMode(MOTOR1B_IN3, OUTPUT);
    pinMode(MOTOR1B_IN4, OUTPUT);
    pinMode(MOTOR1B_ENB, OUTPUT);

    // Set Motor 1A to move forward at half speed
    digitalWrite(MOTOR1A_IN1, HIGH);
    digitalWrite(MOTOR1A_IN2, LOW);
    analogWrite(MOTOR1A_ENA, MOTOR1A_PWM);

    // Set Motor 1B to move backward at half speed
    digitalWrite(MOTOR1B_IN3, LOW);
    digitalWrite(MOTOR1B_IN4, HIGH);
    analogWrite(MOTOR1B_ENB, MOTOR1B_PWM);
}

void loop() {
}
