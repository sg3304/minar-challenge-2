#include <Arduino.h>

// ---------- Motor pin maps ----------
struct MotorPins {
    uint8_t in1, in2, pwm, enc;
};

MotorPins motorPins[4] = {
    {6, 7, 2, 18}, // Motor A
    {8, 9, 3, 19}, // Motor B
    {36, 34, 4, 20}, // Motor C
    {32, 30, 5, 21} // Motor D
};

// ---------- Encoder parameters ----------
static constexpr uint16_t PULSES_PER_MOTOR_REV = 11;
static constexpr float GEAR_RATIO = 30.0f;
static constexpr float pulsesPerWheelRev = float(PULSES_PER_MOTOR_REV) * GEAR_RATIO;

// ---------- PID controller ----------
struct PID {
    float kp = 18.0f;
    float ki = 7.0f;
    float kd = 0.2f;
    float integral = 0.0f;
    float prevMeas = 0.0f;
    float dFilt = 0.0f; // derivative filter state
    float iMin = -200, iMax = 200;
};

struct MotorState {
    float setpoint = 0.0f; // rad/s
    float speed = 0.0f; // filtered rad/s
    float speedRaw = 0.0f;
    PID pid;
    int pwmOut = 0;
};

MotorState motors[4];

// ---------- Helper: parse CSV floats from bracketed list ----------
// Expected format: [v0,v1,v2,v3]  (only 4 floats, velocities)
static inline bool parseBracketedFloats(const String &line, float out[4]) {
    if (line.length() < 3) return false;
    int lb = line.indexOf('[');
    int rb = line.lastIndexOf(']');
    if (lb < 0 || rb <= lb) return false;
    String body = line.substring(lb + 1, rb);
    int idx = 0;
    int start = 0;
    while (start <= body.length() && idx < 4) {
        int comma = body.indexOf(',', start);
        String token = (comma == -1) ? body.substring(start) : body.substring(start, comma);
        token.trim();
        if (token.length() == 0) return false;
        out[idx++] = token.toFloat();
        if (comma == -1) break;
        start = comma + 1;
    }
    return idx == 4;
}

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
void applyMotor(float u, const MotorPins &p) {
    int cmd = constrain((int)roundf(fabs(u)), 0, 255);

    if (fabs(u) < 1.0f) {
        // Coast near zero control effort
        digitalWrite(p.in1, LOW);
        digitalWrite(p.in2, LOW);
        analogWrite(p.pwm, 0);
    } else if (u > 0) {
        // Forward
        digitalWrite(p.in1, HIGH);
        digitalWrite(p.in2, LOW);
        analogWrite(p.pwm, cmd);
    } else {
        // Short-brake (for decel on L298N)
        digitalWrite(p.in1, HIGH);
        digitalWrite(p.in2, HIGH);
        analogWrite(p.pwm, cmd);
    }
}

// ---------- PID runner ----------
float runPID(PID &c, MotorState &m, float meas, float dt) {
    if (dt <= 0.0f) dt = 1e-3f;

    float e = m.setpoint - meas; // signed error

    // Derivative on measurement (avoids setpoint kick)
    float de = -(meas - c.prevMeas) / dt;
    c.prevMeas = meas;

    // Low-pass filter derivative
    c.dFilt = 0.7f * c.dFilt + 0.3f * de;

    // Provisional control
    float u_unsat = c.kp * e + c.ki * c.integral + c.kd * c.dFilt;

    // Saturate
    float u = constrain(u_unsat, -255.0f, 255.0f);

    // Anti-windup: integrate only if not saturated OR if integral helps desaturate
    bool notSat = (u == u_unsat);
    bool helpsOutHigh = (u_unsat > 255.0f) && (e < 0.0f);
    bool helpsOutLow = (u_unsat < -255.0f) && (e > 0.0f);
    if (notSat || helpsOutHigh || helpsOutLow) {
        c.integral += e * dt;
        c.integral = constrain(c.integral, c.iMin, c.iMax);
    } else {
        // bleed off when stuck saturated
        c.integral *= 0.95f;
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

    Serial.println("Ready. Send: [velA,velB,velC,velD] (vel in rad/s)");
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
        Serial.println(input);
        input.trim();

        float vals[4] = {0};
        bool hasList = parseBracketedFloats(input, vals);
        if (hasList) {
            for (int i = 0; i < 4; i++) {
                float newSp = vals[i];

                // Ramp setpoint gradually
                motors[i].setpoint = newSp;

                // Reset integral if command goes to zero or flips sign
                if ((newSp == 0.0f) ||
                    ((motors[i].setpoint > 0 && newSp < 0) || (motors[i].setpoint < 0 && newSp > 0))) {
                    motors[i].pid.integral = 0.0f;
                }
            }
            Serial.print("Setpoints [rad/s]: ");
            Serial.print(motors[0].setpoint, 3);
            Serial.print(", ");
            Serial.print(motors[1].setpoint, 3);
            Serial.print(", ");
            Serial.print(motors[2].setpoint, 3);
            Serial.print(", ");
            Serial.println(motors[3].setpoint, 3);
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
            float pulsesPerSec = (dt > 0.0f) ? (delta / dt) : 0.0f;
            float wheelRps = pulsesPerSec / pulsesPerWheelRev;
            float raw = wheelRps * 2.0f * PI;

            // Low-pass filter on speed
            motors[i].speed = 0.8f * motors[i].speed + 0.2f * raw;
            motors[i].speedRaw = raw;

            float u = runPID(motors[i].pid, motors[i], motors[i].speed, dt);

            motors[i].pwmOut = (int) fabs(u);
            applyMotor(u, motorPins[i]);

            lastCount[i] = total;
        }

        // ---- Debug print ----
        // for (int i = 0; i < 4; i++) {
        //     Serial.print((char)('A' + i));
        //     Serial.print(": set=");
        //     Serial.print(motors[i].setpoint, 2);
        //     Serial.print(", meas=");
        //     Serial.print(motors[i].speed, 2);
        //     Serial.print(", PWM=");
        //     Serial.print(motors[i].pwmOut);
        //     if (i < 3) Serial.print(" | ");
        // }
        // Serial.println();

        // ---- Reply measured RAD/S ----
        // Print measured speeds only, as [A,B,C,D] in rad/s
        Serial.print('[');
        Serial.print(motors[0].speed, 2);
        Serial.print(',');
        Serial.print(motors[1].speed, 2);
        Serial.print(',');
        Serial.print(motors[2].speed, 2);
        Serial.print(',');
        Serial.print(motors[3].speed, 2);
        Serial.println(']');

        lastMs = nowMs;
    }
}
