#include <Arduino.h>

// ---------- Motor 1A ----------
#define MOTOR1A_IN1 2
#define MOTOR1A_IN2 3
#define MOTOR1A_ENA 6
#define MOTOR1A_C1A 18   // encoder input

// ---------- Motor 1B ----------
#define MOTOR1B_IN3 4
#define MOTOR1B_IN4 5
#define MOTOR1B_ENB 7
#define MOTOR1B_C1A 19   // encoder input

// ---------- Motor 2A ----------
#define MOTOR2A_IN1 8
#define MOTOR2A_IN2 9
#define MOTOR2A_ENA 12
#define MOTOR2A_C1A 20   // encoder input

// ---------- Motor 2B ----------
#define MOTOR2B_IN3 10
#define MOTOR2B_IN4 11
#define MOTOR2B_ENB 13
#define MOTOR2B_C1A 21   // encoder input

// ---------- Encoder parameters ----------
static constexpr uint16_t PULSES_PER_MOTOR_REV = 11;
static constexpr float    GEAR_RATIO            = 30.0f;
static constexpr float    pulsesPerWheelRev     = float(PULSES_PER_MOTOR_REV) * GEAR_RATIO;

// ---------- PID parameters ----------
float Kp = 10.0f;
float Ki = 0.0f;
float Kd = 0.0f;

// ---------- State per motor ----------
struct MotorState {
  float setpoint = 0.0f;   // rad/s
  float speed = 0.0f;      // measured rad/s
  float integral = 0.0f;
  float prevError = 0.0f;
};

MotorState motorA, motorB, motorC, motorD;

// ---------- PWM values ----------
int motor1A_pwm = 0;
int motor1B_pwm = 0;
int motor2A_pwm = 0;
int motor2B_pwm = 0;

// ---------- Encoder counters ----------
volatile uint32_t motor1A_pulseCount = 0, motor1A_lastEdgeUs = 0;
volatile uint32_t motor1B_pulseCount = 0, motor1B_lastEdgeUs = 0;
volatile uint32_t motor2A_pulseCount = 0, motor2A_lastEdgeUs = 0;
volatile uint32_t motor2B_pulseCount = 0, motor2B_lastEdgeUs = 0;

// ---------- Interrupts ----------
void motor1A_isr() {
  uint32_t now = micros();
  if ((now - motor1A_lastEdgeUs) > 100) {
    motor1A_pulseCount++;
    motor1A_lastEdgeUs = now;
  }
}

void motor1B_isr() {
  uint32_t now = micros();
  if ((now - motor1B_lastEdgeUs) > 100) {
    motor1B_pulseCount++;
    motor1B_lastEdgeUs = now;
  }
}

void motor2A_isr() {
  uint32_t now = micros();
  if ((now - motor2A_lastEdgeUs) > 100) {
    motor2A_pulseCount++;
    motor2A_lastEdgeUs = now;
  }
}

void motor2B_isr() {
  uint32_t now = micros();
  if ((now - motor2B_lastEdgeUs) > 100) {
    motor2B_pulseCount++;
    motor2B_lastEdgeUs = now;
  }
}

// ---------- PID helper ----------
int runPID(MotorState &m, float measured, float dt) {
  float error = m.setpoint - measured;
  m.integral += error * dt;
  float derivative = (error - m.prevError) / dt;
  m.prevError = error;

  float output = Kp * error + Ki * m.integral + Kd * derivative;
  return constrain(int(output), 0, 255);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(9600);

  // Motor 1A
  pinMode(MOTOR1A_IN1, OUTPUT);
  pinMode(MOTOR1A_IN2, OUTPUT);
  pinMode(MOTOR1A_ENA, OUTPUT);
  digitalWrite(MOTOR1A_IN1, HIGH);
  digitalWrite(MOTOR1A_IN2, LOW);

  // Motor 1B
  pinMode(MOTOR1B_IN3, OUTPUT);
  pinMode(MOTOR1B_IN4, OUTPUT);
  pinMode(MOTOR1B_ENB, OUTPUT);
  digitalWrite(MOTOR1B_IN3, HIGH);
  digitalWrite(MOTOR1B_IN4, LOW);

  // Motor 2A
  pinMode(MOTOR2A_IN1, OUTPUT);
  pinMode(MOTOR2A_IN2, OUTPUT);
  pinMode(MOTOR2A_ENA, OUTPUT);
  digitalWrite(MOTOR2A_IN1, HIGH);
  digitalWrite(MOTOR2A_IN2, LOW);

  // Motor 2B
  pinMode(MOTOR2B_IN3, OUTPUT);
  pinMode(MOTOR2B_IN4, OUTPUT);
  pinMode(MOTOR2B_ENB, OUTPUT);
  digitalWrite(MOTOR2B_IN3, HIGH);
  digitalWrite(MOTOR2B_IN4, LOW);

  // Encoders
  pinMode(MOTOR1A_C1A, INPUT_PULLUP);
  pinMode(MOTOR1B_C1A, INPUT_PULLUP);
  pinMode(MOTOR2A_C1A, INPUT_PULLUP);
  pinMode(MOTOR2B_C1A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(MOTOR1A_C1A), motor1A_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR1B_C1A), motor1B_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR2A_C1A), motor2A_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR2B_C1A), motor2B_isr, RISING);

  Serial.println("Ready. Use 'A 10', 'B 15', 'C 20', 'D 12' to set speeds in rad/s");
}

// ---------- Loop ----------
void loop() {
  static uint32_t lastMs = 0;
  static uint32_t lastCountA = 0, lastCountB = 0, lastCountC = 0, lastCountD = 0;

  const uint32_t nowMs = millis();
  const uint32_t periodMs = 200;

  // ---- Serial input ----
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 2) {
      char motorID = toupper(input.charAt(0));
      float value = input.substring(1).toFloat();

      MotorState *target = nullptr;
      if (motorID == 'A') target = &motorA;
      else if (motorID == 'B') target = &motorB;
      else if (motorID == 'C') target = &motorC;
      else if (motorID == 'D') target = &motorD;

      if (target) {
        target->setpoint = value;
        Serial.print("Motor ");
        Serial.print(motorID);
        Serial.print(" setpoint = ");
        Serial.print(value);
        Serial.println(" rad/s");
      }
    }
  }

  // ---- Update every periodMs ----
  if (nowMs - lastMs >= periodMs) {
    float dt = (nowMs - lastMs) / 1000.0f;

    auto updateMotor = [&](MotorState &m, volatile uint32_t &pulseCount, uint32_t &lastCount, int &pwm, int pwmPin) {
      noInterrupts();
      uint32_t total = pulseCount;
      interrupts();
      uint32_t delta = total - lastCount;
      float pulsesPerSec = delta / dt;
      float wheelRps = pulsesPerSec / pulsesPerWheelRev;
      m.speed = wheelRps * 2.0f * PI;   // rad/s

      if (m.setpoint == 0.0f) {
        m.integral = 0.0f;
        m.prevError = 0.0f;
        pwm = 0;
        analogWrite(pwmPin, 0);
      } else {
        pwm = runPID(m, m.speed, dt);
        analogWrite(pwmPin, pwm);
      }
      lastCount = total;
    };

    // Update each motor
    updateMotor(motorA, motor1A_pulseCount, lastCountA, motor1A_pwm, MOTOR1A_ENA);
    updateMotor(motorB, motor1B_pulseCount, lastCountB, motor1B_pwm, MOTOR1B_ENB);
    updateMotor(motorC, motor2A_pulseCount, lastCountC, motor2A_pwm, MOTOR2A_ENA);
    updateMotor(motorD, motor2B_pulseCount, lastCountD, motor2B_pwm, MOTOR2B_ENB);

    // --- Debug print ---

    // The msgs should be structured as such:
    // https://docs.ros.org/en/noetic/api/std_msgs/html/msg/MultiArrayLayout.html
    // 'Publish as Float32MultiArray: [pos0,pos1,pos2,pos3,vel0,vel1,vel2,vel3]'
    // [wheel1,wheel2,wheel3,wheel4,vel1,vel2,vel3,vel4]
    Serial.print("A: set=");
    Serial.print(motorA.setpoint, 2);
    Serial.print(", meas=");
    Serial.print(motorA.speed, 2);
    Serial.print(", PWM=");
    Serial.print(motor1A_pwm);

    Serial.print(" | B: set=");
    Serial.print(motorB.setpoint, 2);
    Serial.print(", meas=");
    Serial.print(motorB.speed, 2);
    Serial.print(", PWM=");
    Serial.print(motor1B_pwm);

    Serial.print(" | C: set=");
    Serial.print(motorC.setpoint, 2);
    Serial.print(", meas=");
    Serial.print(motorC.speed, 2);
    Serial.print(", PWM=");
    Serial.print(motor2A_pwm);

    Serial.print(" | D: set=");
    Serial.print(motorD.setpoint, 2);
    Serial.print(", meas=");
    Serial.print(motorD.speed, 2);
    Serial.print(", PWM=");
    Serial.println(motor2B_pwm);

    lastMs = nowMs;
  }
}
