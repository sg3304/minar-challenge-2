#include <Arduino.h>

// ---------- Motor pin maps ----------
struct MotorPins {
    uint8_t in1, in2, pwm, enc;
};

MotorPins motorPins[4] = {
    {6, 7, 2, 18},    // Motor A
    {8, 9, 3, 19},    // Motor B
    {36, 34, 4, 20},  // Motor C
    {32, 30, 5, 21}   // Motor D
};

// ---------- Encoder parameters ----------
static constexpr uint16_t PULSES_PER_MOTOR_REV = 11;
static constexpr float    GEAR_RATIO           = 30.0f;
static constexpr float    pulsesPerWheelRev    = float(PULSES_PER_MOTOR_REV) * GEAR_RATIO;

// ---------- PID controller ----------
struct PID {
    float kp = 22.0f;
    float ki = 5.0f;
    float kd = 0.1f;
    float integral = 0.0f;
    float prevMeas = 0.0f;
    float dFilt = 0.0f;  // derivative filter state
    float iMin = -200, iMax = 200;
};

struct MotorState {
    float setpoint = 0.0f; // rad/s
    float speed    = 0.0f; // filtered rad/s
    float speedRaw = 0.0f;
    PID pid;
    int pwmOut = 0;
};

MotorState motors[4];

// ---------- Encoder counters ----------
volatile uint32_t pulseCount[4] = {0, 0, 0, 0};
volatile uint32_t lastEdgeUs[4] = {0, 0, 0, 0};

// ---------- ISRs ----------
void encoderISR0() {
    uint32_t now = micros();
    if ((now - lastEdgeUs[0]) > 100) {
        pulseCount[0]++;
        lastEdgeUs[0] = now;
    }
}
void encoderISR1() {
    uint32_t now = micros();
    if ((now - lastEdgeUs[1]) > 100) {
        pulseCount[1]++;
        lastEdgeUs[1] = now;
    }
}
void encoderISR2() {
    uint32_t now = micros();
    if ((now - lastEdgeUs[2]) > 100) {
        pulseCount[2]++;
        lastEdgeUs[2] = now;
    }
}
void encoderISR3() {
    uint32_t now = micros();
    if ((now - lastEdgeUs[3]) > 100) {
        pulseCount[3]++;
        lastEdgeUs[3] = now;
    }
}

void (*isrFuncs[4])() = {encoderISR0, encoderISR1, encoderISR2, encoderISR3};

// ---------- Apply motor command ----------
void applyMotor(float u, const MotorPins &p, float setpoint) {
    int cmd = constrain((int)roundf(fabs(u)), 0, 255);

    if (fabs(setpoint) < 0.01f) {
        // Coast
        digitalWrite(p.in1, LOW);
        digitalWrite(p.in2, LOW);
        analogWrite(p.pwm, 0);
    } else if (setpoint > 0) {
        // Forward
        digitalWrite(p.in1, HIGH);
        digitalWrite(p.in2, LOW);
        analogWrite(p.pwm, cmd);
    } else {
        // Reverse
        digitalWrite(p.in1, LOW);
        digitalWrite(p.in2, HIGH);
        analogWrite(p.pwm, cmd);
    }
}

// ---------- PID runner ----------
float runPID(PID &c, MotorState &m, float meas, float dt) {
    float e = m.setpoint - meas; // signed error

    // Derivative of measurement
    float dMeas = (meas - c.prevMeas) / dt;
    c.prevMeas = meas;

    // Low-pass filter derivative
    c.dFilt = 0.7f * c.dFilt + 0.3f * dMeas;

    // Compute unsaturated control first
    float u_unsat = c.kp * e + c.ki * c.integral - c.kd * c.dFilt;

    // Saturate PWM to actuator limits
    float u = constrain(u_unsat, -255.0f, 255.0f);

    // Anti-windup: only integrate if actuator not saturated
    if ((u_unsat > -255 && u_unsat < 255) ||
        (u_unsat < -255 && e < 0) ||
        (u_unsat > 255 && e > 0))
    {
        c.integral += e * dt;
        c.integral = constrain(c.integral, c.iMin, c.iMax);
    }

    return u;
}

// ---------- Setup ----------
void setup() {
    Serial.begin(115200);

    for (int i = 0; i < 4; i++) {
        pinMode(motorPins[i].in1, OUTPUT);
        pinMode(motorPins[i].in2, OUTPUT);
        pinMode(motorPins[i].pwm, OUTPUT);
        digitalWrite(motorPins[i].in1, LOW);
        digitalWrite(motorPins[i].in2, LOW);
        analogWrite(motorPins[i].pwm, 0);

        pinMode(motorPins[i].enc, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(motorPins[i].enc), isrFuncs[i], RISING);
    }

    Serial.println("Ready. Use 'A 10', 'B -15', 'C 20', 'D -12' to set speeds (rad/s).");
}

// ---------- Loop ----------
void loop() {
    static uint32_t lastMs = 0;
    static uint32_t lastCount[4] = {0, 0, 0, 0};

    const uint32_t nowMs = millis();
    const uint32_t periodMs = 20; // 20 ms = 50 Hz loop

    // ---- Serial input ----
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() > 2) {
            char motorID = toupper(input.charAt(0));
            float value = input.substring(1).toFloat();
            int idx = motorID - 'A';
            if (idx >= 0 && idx < 4) {
                motors[idx].setpoint = value;
                motors[idx].pid.integral = 0; // reset integral on new command
                Serial.print("Motor ");
                Serial.print(motorID);
                Serial.print(" setpoint = ");
                Serial.print(value);
                Serial.println(" rad/s");
            }
        }
    }

    // ---- Periodic update ----
    if (nowMs - lastMs >= periodMs) {
        float dt = (nowMs - lastMs) / 1000.0f;

        for (int i = 0; i < 4; i++) {
            noInterrupts();
            uint32_t total = pulseCount[i];
            interrupts();

            uint32_t delta = total - lastCount[i];
            float pulsesPerSec = delta / dt;
            float wheelRps = pulsesPerSec / pulsesPerWheelRev;
            float raw = wheelRps * 2.0f * PI;

            // Low-pass filter on speed
            motors[i].speed = 0.8f * motors[i].speed + 0.2f * raw;
            motors[i].speedRaw = raw;

            float u = runPID(motors[i].pid, motors[i], motors[i].speed, dt);
            motors[i].pwmOut = (int)fabs(u);
            applyMotor(u, motorPins[i], motors[i].setpoint);

            lastCount[i] = total;
        }

        // ---- Debug print ----
        for (int i = 0; i < 4; i++) {
            Serial.print((char)('A' + i));
            Serial.print(": set=");
            Serial.print(motors[i].setpoint, 2);
            Serial.print(", meas=");
            Serial.print(motors[i].speed, 2);
            Serial.print(", PWM=");
            Serial.print(motors[i].pwmOut);
            if (i < 3) Serial.print(" | ");
        }
        Serial.println();

        lastMs = nowMs;
    }
}
