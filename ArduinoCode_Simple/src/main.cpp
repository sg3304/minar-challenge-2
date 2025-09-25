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

    //Serial.println("Ready. Send: [velA,velB,velC,velD] (vel in rad/s)");
}

// ---------- Loop ----------
void loop() {
    static uint32_t lastMs = 0;
    static uint32_t lastCount[4] = {0, 0, 0, 0};

    const uint32_t nowMs = millis();
    const uint32_t periodMs = 20; // 50 Hz loop

    // Median buffer: 10 samples per motor
    static float medBuf[4][10] = {0};
    static uint8_t medIdx = 0;
    static uint8_t medCount = 0;

    // ---- Serial input ----
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        float vals[4];
        if (parseBracketedFloats(input, vals)) {
            for (int i = 0; i < 4; i++) {
                float newSp = vals[i];
                // Ramp setpoint
                motors[i].setpoint = motors[i].setpoint * 0.8f + newSp * 0.2f;
                // Reset integral if command goes to zero or flips sign
                if ((newSp == 0.0f) ||
                    ((motors[i].setpoint > 0 && newSp < 0) || (motors[i].setpoint < 0 && newSp > 0))) {
                    motors[i].pid.integral = 0.0f;
                }
            }
        }
    }

    // ---- Periodic update ----
    if (nowMs - lastMs >= periodMs) {
        float dt = (nowMs - lastMs) / 1000.0f;
        if (dt <= 0.0f) dt = periodMs / 1000.0f;

        for (int i = 0; i < 4; i++) {
            noInterrupts();
            uint32_t total = pulseCount[i];
            interrupts();

            uint32_t delta = total - lastCount[i];
            lastCount[i] = total;

            float pulsesPerSec = (dt > 0.0f) ? (delta / dt) : 0.0f;
            float wheelRps = pulsesPerSec / pulsesPerWheelRev;
            float raw = wheelRps * 2.0f * PI;

            // Low-pass filter on speed
            motors[i].speed = 0.8f * motors[i].speed + 0.2f * raw;
            motors[i].speedRaw = raw;

            float u = runPID(motors[i].pid, motors[i], motors[i].speed, dt);
            motors[i].pwmOut = (int)fabs(u);
            applyMotor(u, motorPins[i]);

            // Store into median buffer
            medBuf[i][medIdx] = motors[i].speed;
        }

        // Advance buffer index
        medIdx = (uint8_t)((medIdx + 1) % 10);
        if (medCount < 10) medCount++;

        // ---- Median output every 10 samples ----
        if (medCount == 10 && medIdx == 0) {
            float tmp[10];

            auto median10 = [&](int m) -> float {
                for (int k = 0; k < 10; ++k) tmp[k] = medBuf[m][k];
                // Insertion sort (n=10)
                for (int a = 1; a < 10; ++a) {
                    float key = tmp[a];
                    int b = a - 1;
                    while (b >= 0 && tmp[b] > key) {
                        tmp[b + 1] = tmp[b];
                        --b;
                    }
                    tmp[b + 1] = key;
                }
                // Even count: average middle two
                return 0.5f * (tmp[4] + tmp[5]);
            };

            float mA = median10(0);
            float mB = median10(1);
            float mC = median10(2);
            float mD = median10(3);

            Serial.print('[');
            Serial.print(mA, 2); Serial.print(',');
            Serial.print(mB, 2); Serial.print(',');
            Serial.print(mC, 2); Serial.print(',');
            Serial.print(mD, 2);
            Serial.println(']');
        }

        lastMs = nowMs;
    }
}
