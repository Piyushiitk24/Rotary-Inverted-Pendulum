#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <FastAccelStepper.h>
#include <math.h>

// ============================================================
// MOTOR POSITION TRACKING TESTER (Position Sine / moveTo())
// ============================================================

// --- PINS ---
#define STEP_PIN 11
#define DIR_PIN  6
#define EN_PIN   7

#define SERIAL_BAUD 500000
#define TCA_ADDR 0x70

// --- MECHANICS ---
const float DEG_PER_STEP   = 0.225f;   // degrees per (micro)step
const float BASE_LIMIT_DEG = 80.0f;    // hard safety limit

// --- SIGN CONVENTION (IMPORTANT) ---
// Goal: Positive target_deg should produce positive actual_deg.
//
// SENSOR_SIGN:
//   +1 => angle = (rawDeg - zeroDeg)
//   -1 => angle = -(rawDeg - zeroDeg)
//
// STEP_SIGN:
//   +1 => positive target_deg -> positive step position (moveTo +steps)
//   -1 => positive target_deg -> negative step position (moveTo -steps)
const int SENSOR_SIGN = +1;
const int STEP_SIGN   = +1;

// --- STEPPER LIMITS (span-limited realistic) ---
const uint32_t MAX_SPEED_HZ = 2500;     // steps/s cap
const uint32_t ACCEL_HZ2    = 20000;    // steps/s^2

// --- TEST SETTINGS ---
const float AMPLITUDE_DEG   = 20.0f;    // keep within your physical travel
const float FREQ_HZ         = 1.0f;     // sine frequency
const float TEST_DURATION_S = 10.0f;

const uint32_t CONTROL_PERIOD_MS = 5;   // how often we update moveTo()
const uint32_t LOG_PERIOD_MS     = 5;  // telemetry rate

// --- STATE ---
FastAccelStepperEngine engine;
FastAccelStepper* stepper = nullptr;

AS5600 motorSensor(&Wire);
float motorZeroDeg = 0.0f;
float motorAngleDeg = 0.0f;

bool testRunning = false;
unsigned long testStartMs = 0;
unsigned long lastControlMs = 0;
unsigned long lastLogMs = 0;

// --- I2C MUX ---
void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// --- HELPERS ---
float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float readMotorAngleDeg() {
  tcaSelect(1);
  uint16_t raw = motorSensor.readAngle();
  float degRaw = (raw * 360.0f) / 4096.0f;
  float deg = SENSOR_SIGN * (degRaw - motorZeroDeg);
  return normalizeAngle(deg);
}

inline long angleDegToSteps(float angleDeg) {
  // Convert degrees -> steps, applying sign mapping for the stepper direction convention
  float stepsF = (angleDeg / DEG_PER_STEP) * (float)STEP_SIGN;
  return lroundf(stepsF);
}

inline float stepsToAngleDeg(long steps) {
  // Inverse of angleDegToSteps()
  return ((float)steps * DEG_PER_STEP) / (float)STEP_SIGN;
}

// ============================================================
// POSITION SINE LOOP
// ============================================================
void runTestLoop() {
  unsigned long now = millis();
  float t = (now - testStartMs) / 1000.0f;

  if (t > TEST_DURATION_S) {
    testRunning = false;
    stepper->stopMove();
    digitalWrite(EN_PIN, HIGH);
    Serial.println(F("[BALANCE_LOG_END],0,test_complete"));
    return;
  }

  // Update command at CONTROL_PERIOD_MS
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = now;

    // target angle command (position)
    float omega = 2.0f * (float)M_PI * FREQ_HZ;
    float targetAngleDeg = AMPLITUDE_DEG * sinf(omega * t);

    // Safety guard on the commanded target too (in case settings change)
    if (fabs(targetAngleDeg) > BASE_LIMIT_DEG) {
      testRunning = false;
      stepper->forceStop();
      digitalWrite(EN_PIN, HIGH);
      Serial.println(F("[BALANCE_LOG_END],0,target_limit"));
      return;
    }

    long targetSteps = angleDegToSteps(targetAngleDeg);
    stepper->moveTo(targetSteps);
  }

  // Read sensor (for safety + telemetry)
  motorAngleDeg = readMotorAngleDeg();

  if (fabs(motorAngleDeg) > BASE_LIMIT_DEG) {
    testRunning = false;
    stepper->forceStop();
    digitalWrite(EN_PIN, HIGH);
    Serial.println(F("[BALANCE_LOG_END],0,base_limit"));
    return;
  }

  // Telemetry
  if (now - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = now;

    float omega = 2.0f * (float)M_PI * FREQ_HZ;
    float targetAngleDeg = AMPLITUDE_DEG * sinf(omega * t);
    float targetVelDeg_s = AMPLITUDE_DEG * omega * cosf(omega * t);
    float targetHz_ff = (targetVelDeg_s / DEG_PER_STEP) * (float)STEP_SIGN;

    long targetSteps = angleDegToSteps(targetAngleDeg);
    long stepPos = stepper->getCurrentPosition();

    float expectedDegFromSteps = stepsToAngleDeg(stepPos);

    float errDeg = targetAngleDeg - motorAngleDeg;                 // tracking error vs target
    long  errSteps = targetSteps - stepPos;                        // stepper lag vs commanded target
    float stepSensorDegMismatch = expectedDegFromSteps - motorAngleDeg; // encoder vs step-derived angle

    Serial.print(t, 3); Serial.print(",");
    Serial.print(targetAngleDeg, 2); Serial.print(",");
    Serial.print(motorAngleDeg, 2); Serial.print(",");
    Serial.print(stepPos); Serial.print(",");
    Serial.print(targetSteps); Serial.print(",");
    Serial.print(expectedDegFromSteps, 2); Serial.print(",");
    Serial.print(errDeg, 2); Serial.print(",");
    Serial.print(errSteps); Serial.print(",");
    Serial.print(stepSensorDegMismatch, 2); Serial.print(",");
    Serial.print(targetHz_ff, 1); Serial.print(",");
    Serial.println(FREQ_HZ, 2);
  }
}

// ============================================================
// COMMANDS
// ============================================================
void handleCommand(char c) {
  switch (c) {
    case 'Z': { // Zero motor + zero stepper position
      tcaSelect(1);
      motorZeroDeg = (motorSensor.readAngle() * 360.0f) / 4096.0f;
      motorAngleDeg = 0.0f;

      if (stepper) stepper->setCurrentPosition(0);

      Serial.print(F("Zero Set (degRaw): "));
      Serial.println(motorZeroDeg, 3);
      Serial.println(F("Stepper position set to 0."));
      break;
    }

    case 'T': { // quick check
      float ang = readMotorAngleDeg();
      long steps = stepper ? stepper->getCurrentPosition() : 0;
      Serial.print(F("Angle(deg): ")); Serial.print(ang, 2);
      Serial.print(F(" | Steps: ")); Serial.println(steps);
      break;
    }

    case 'S': { // start
      Serial.println(F("[BALANCE_LOG_START],0,motor_position_sine_test"));
      Serial.println(F("[BALANCE_COLUMNS],time_s,target_deg,actual_deg,step_pos,target_steps,expected_deg_from_steps,err_deg,err_steps,step_sensor_deg_mismatch,target_hz_ff,freq_hz"));

      digitalWrite(EN_PIN, LOW);

      testStartMs = millis();
      lastControlMs = 0;
      lastLogMs = 0;
      testRunning = true;

      // Print settings (human-readable)
      Serial.print(F("[INFO],MAX_SPEED_HZ,")); Serial.println(MAX_SPEED_HZ);
      Serial.print(F("[INFO],ACCEL_HZ2,"));    Serial.println(ACCEL_HZ2);
      Serial.print(F("[INFO],AMPLITUDE_DEG,")); Serial.println(AMPLITUDE_DEG, 2);
      Serial.print(F("[INFO],FREQ_HZ,"));       Serial.println(FREQ_HZ, 2);
      Serial.print(F("[INFO],SENSOR_SIGN,"));   Serial.println(SENSOR_SIGN);
      Serial.print(F("[INFO],STEP_SIGN,"));     Serial.println(STEP_SIGN);
      break;
    }

    case 'X': { // stop
      testRunning = false;
      if (stepper) stepper->forceStop();
      digitalWrite(EN_PIN, HIGH);
      Serial.println(F("[BALANCE_LOG_END],0,user_abort"));
      break;
    }
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);

  tcaSelect(1);
  motorSensor.begin();

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); // disabled by default (EN active-low)

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(EN_PIN, true);
  stepper->setAutoEnable(false);

  stepper->setSpeedInHz(MAX_SPEED_HZ);
  stepper->setAcceleration(ACCEL_HZ2);

  Serial.println(F("Motor Position Sine Test (moveTo)."));
  Serial.println(F("Commands: Z=zero, T=angle, S=start, X=stop"));
}

void loop() {
  if (Serial.available()) {
    char c = toupper(Serial.read());
    if (strchr("SXZT", c)) handleCommand(c);
  }

  if (testRunning) {
    runTestLoop();
  }
}