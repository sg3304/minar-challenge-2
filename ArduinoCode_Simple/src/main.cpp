#include <Arduino.h>

// ---------- Motor pin maps ----------
struct MotorPins {
    uint8_t in1, in2, pwm, encA, encB;
};

MotorPins motorPins[4] = {
    {6, 7, 2, 18, 22}, // Motor A
    {8, 9, 3, 19, 23}, // Motor B
    {36, 34, 4, 20, 48}, // Motor C
    {32, 30, 5, 21, 49} // Motor D
};

// ---------- Direction config ----------
// Set to +1 or -1 for each motor
// A and C are mirrored -> -1
int8_t encoderDir[4] = {-1, +1, -1, +1};
int8_t motorDir[4] = {-1, +1, -1, +1};

// ---------- Encoder parameters ----------
static constexpr uint16_t PULSES_PER_MOTOR_REV = 11;
static constexpr float GEAR_RATIO = 30.0f;
static constexpr float pulsesPerWheelRev = float(PULSES_PER_MOTOR_REV) * GEAR_RATIO;

// ---------- PID controller ----------
struct PID {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integral = 0.0f;
    float prevMeas = 0.0f;
    float dFilt = 0.0f;
    float iMin = -200, iMax = 200;
};

constexpr float KP[4] = {50.0f, 50.0f, 50.5f, 50.0f};
constexpr float KI[4] = {20.8f, 20.8f, 20.0f, 20.8f};
constexpr float KD[4] = {0.0f, 0.0f, 0.0f, 0.0f};

struct MotorState {
    float setpoint = 0.0f; // rad/s
    float speed = 0.0f; // filtered rad/s
    float speedRaw = 0.0f;
    PID pid;
    int pwmOut = 0;
};

MotorState motors[4];

// ---------- Helper: parse CSV floats ----------
static inline bool parseBracketedFloats(const String &line, float out[4]) {
    if (line.length() < 3)
        return false;
    int lb = line.indexOf('[');
    int rb = line.lastIndexOf(']');
    if (lb < 0 || rb <= lb)
        return false;
    String body = line.substring(lb + 1, rb);
    int idx = 0;
    size_t start = 0;
    while (start <= body.length() && idx < 4) {
        int comma = body.indexOf(',', start);
        String token = (comma == -1) ? body.substring(start) : body.substring(start, comma);
        token.trim();
        if (token.length() == 0)
            return false;
        out[idx++] = token.toFloat();
        if (comma == -1)
            break;
        start = comma + 1;
    }
    return idx == 4;
}

// ---------- Encoder counters ----------
volatile int32_t pulseCount[4] = {0, 0, 0, 0};
volatile uint32_t lastEdgeUs[4] = {0, 0, 0, 0};

static constexpr bool DIR_FLIP = false;

static inline int8_t dirFromB(int bLevel) {
    int8_t dir = (bLevel ? -1 : +1);
    return DIR_FLIP ? (int8_t) -dir : dir;
}

// ---------- ISRs ----------
void encoderISR_generic(uint8_t idx) {
    uint32_t now = micros();
    if ((now - lastEdgeUs[idx]) > 100) {
        // denoise
        int b = digitalRead(motorPins[idx].encB);
        pulseCount[idx] += dirFromB(b);
        lastEdgeUs[idx] = now;
    }
}

void encoderISR0() { encoderISR_generic(0); }
void encoderISR1() { encoderISR_generic(1); }
void encoderISR2() { encoderISR_generic(2); }
void encoderISR3() { encoderISR_generic(3); }

void (*isrFuncs[4])() = {encoderISR0, encoderISR1, encoderISR2, encoderISR3};

// ---------- Apply motor command ----------
void applyMotor(float u, const MotorPins &p) {
    int cmd = constrain((int)roundf(fabs(u)), 0, 255);

    if (fabs(u) < 1.0f) {
        digitalWrite(p.in1, LOW);
        digitalWrite(p.in2, LOW);
        analogWrite(p.pwm, 0);
    } else if (u > 0) {
        digitalWrite(p.in1, HIGH);
        digitalWrite(p.in2, LOW);
        analogWrite(p.pwm, cmd);
    } else {
        digitalWrite(p.in1, LOW);
        digitalWrite(p.in2, HIGH);
        analogWrite(p.pwm, cmd);
    }
}

// ---------- PID runner ----------
float runPID(PID &c, MotorState &m, float meas, float dt) {
    if (dt <= 0.0f)
        dt = 1e-3f;

    float e = m.setpoint - meas;
    float de = -(meas - c.prevMeas) / dt;
    c.prevMeas = meas;
    c.dFilt = 0.7f * c.dFilt + 0.3f * de;

    float u_unsat = c.kp * e + c.ki * c.integral + c.kd * c.dFilt;
    float u = constrain(u_unsat, -255.0f, 255.0f);

    bool notSat = (u == u_unsat);
    bool helpsOutHigh = (u_unsat > 255.0f) && (e < 0.0f);
    bool helpsOutLow = (u_unsat < -255.0f) && (e > 0.0f);
    if (notSat || helpsOutHigh || helpsOutLow) {
        c.integral += e * dt;
        c.integral = constrain(c.integral, c.iMin, c.iMax);
    } else {
        c.integral *= 0.95f;
    }

    return u;
}

// ---------- Setup ----------
void setup() {
    Serial.begin(115200);

    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    for (int i = 0; i < 4; i++) {
        pinMode(motorPins[i].in1, OUTPUT);
        pinMode(motorPins[i].in2, OUTPUT);
        pinMode(motorPins[i].pwm, OUTPUT);
        digitalWrite(motorPins[i].in1, LOW);
        digitalWrite(motorPins[i].in2, LOW);
        analogWrite(motorPins[i].pwm, 0);

        pinMode(motorPins[i].encA, INPUT_PULLUP);
        pinMode(motorPins[i].encB, INPUT_PULLUP);

        motors[i].pid.kp = KP[i];
        motors[i].pid.ki = KI[i];
        motors[i].pid.kd = KD[i];

        attachInterrupt(digitalPinToInterrupt(motorPins[i].encA), isrFuncs[i],
                        RISING);
    }
}

// ---------- Loop ----------
void loop() {
    static uint32_t lastMs = 0;
    static int32_t lastCount[4] = {0, 0, 0, 0};
    static uint32_t ledOffTime = 0;

    const uint32_t nowMs = millis();
    const uint32_t periodMs = 20; // 50 Hz

    static float medBuf[4][10] = {0};
    static uint8_t medIdx = 0;
    static uint8_t medCount = 0;

    // ---- LED error handling ----
    if (ledOffTime > 0 && nowMs >= ledOffTime) {
        digitalWrite(13, LOW);
        ledOffTime = 0;
    }

    // ---- Serial input ----
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        float vals[4];
        if (parseBracketedFloats(input, vals)) {
            for (int i = 0; i < 4; i++) {
                float newSp = vals[i];
                float oldSp = motors[i].setpoint;
                motors[i].setpoint = newSp;
                if ((newSp == 0.0f) || (oldSp * newSp < 0)) {
                    motors[i].pid.integral = 0.0f;
                }
            }
        } else {
            digitalWrite(13, HIGH);
            ledOffTime = nowMs + 500;
        }
    }

    // ---- Periodic update ----
    if (nowMs - lastMs >= periodMs) {
        float dt = (nowMs - lastMs) / 1000.0f;
        if (dt <= 0.0f)
            dt = periodMs / 1000.0f;

        for (int i = 0; i < 4; i++) {
            noInterrupts();
            int32_t total = pulseCount[i];
            interrupts();

            int32_t delta = (total - lastCount[i]) * encoderDir[i];
            lastCount[i] = total;

            float pulsesPerSec = (dt > 0.0f) ? (delta / dt) : 0.0f;
            float wheelRps = pulsesPerSec / pulsesPerWheelRev;
            float raw = wheelRps * 2.0f * PI;

            motors[i].speed = 0.8f * motors[i].speed + 0.2f * raw;
            motors[i].speedRaw = raw;

            float u = runPID(motors[i].pid, motors[i], raw, dt);
            motors[i].pwmOut = (int) u;

            // Apply motor direction correction
            applyMotor(u * motorDir[i], motorPins[i]);

            medBuf[i][medIdx] = motors[i].speed;
        }

        medIdx = (uint8_t) ((medIdx + 1) % 10);
        if (medCount < 10)
            medCount++;

        if (medCount == 10 && medIdx == 0) {
            float tmp[10];
            auto median10 = [&](int m) -> float {
                for (int k = 0; k < 10; ++k)
                    tmp[k] = medBuf[m][k];
                for (int a = 1; a < 10; ++a) {
                    float key = tmp[a];
                    int b = a - 1;
                    while (b >= 0 && tmp[b] > key) {
                        tmp[b + 1] = tmp[b];
                        --b;
                    }
                    tmp[b + 1] = key;
                }
                return 0.5f * (tmp[4] + tmp[5]);
            };

            if (Serial.availableForWrite() > 32) {
                Serial.print("[");
                Serial.print(median10(0), 2);
                Serial.print(",");
                Serial.print(median10(1), 2);
                Serial.print(",");
                Serial.print(median10(2), 2);
                Serial.print(",");
                Serial.print(median10(3), 2);
                Serial.println("]");
            }
        }

        lastMs = nowMs;
    }
}
