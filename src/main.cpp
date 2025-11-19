#include <Arduino.h>
#include <Wire.h>
#include <SoftwareWire.h>
#include <AS5600.h>
#include <AccelStepper.h>
#include <math.h>
#include <ctype.h>

// ============================================================
//  ROTARY INVERTED PENDULUM - SENSOR-BASED CONTROL
// ============================================================
//
// V5.0 - DUAL SENSOR FEEDBACK
// - 24V Power Supply
// - TMC2209 Vref = 2.11V (for 1.5A RMS)
// - Pendulum Sensor: AS5600 (Hardware I2C)
// - Motor Sensor: AS5600 (Software I2C) - SOURCE OF TRUTH
//
// ============================================================

// Stepper Motor Pins
#define STEP_PIN 5
#define DIR_PIN  6
#define EN_PIN   7
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Pendulum Sensor on Hardware I2C (pins 20/21)
AS5600 as5600_pendulum(&Wire);

// Motor Sensor on Software I2C (pins 22/24)
constexpr uint8_t MOTOR_SDA_PIN = 22;
constexpr uint8_t MOTOR_SCL_PIN = 24;
constexpr uint8_t AS5600_ADDRESS = 0x36;
constexpr uint8_t AS5600_RAW_ANGLE_REG = 0x0C;
constexpr uint8_t AS5600_STATUS_REG    = 0x0B;
SoftwareWire motorWire(MOTOR_SDA_PIN, MOTOR_SCL_PIN);

// Virtual encoder configuration
const float ASSUMED_BASE_LIMIT_DEG = 85.0f;   // Expected +/- mechanical range for the base
const float DEFAULT_DEG_PER_STEP   = 0.1f;    // Fallback if limits not yet known
const float UPRIGHT_PENDULUM_DEG   = 63.63f;  // or whatever the measured average is

// Motor configuration
// --- 24V HIGH PERFORMANCE VALUES ---
// With 24V and 1.5A, the motor can now handle high speeds.
#define MOTOR_SPEED 2000      // 1.25 rev/s = 75 RPM; safe for calib
#define MOTOR_ACCEL 12000     // 7.5 rev/s²; quick but reliable
#define SWING_MAX_SPEED 8000  // 2.2 rev/s = 132 RPM; bursts for pump
#define SWING_ACCEL 25000     // 11.25 rev/s²; aggressive ramps
// Aggressive balance profile (adopt from working high-rate sketch)
#define BALANCE_MAX_SPEED 3000 // 0.94 rev/s = 56 RPM; torque retention for control
#define BALANCE_ACCEL 10000   // 4.7 rev/s²; responsive, below max post-screw
// ---
#define CALIBRATION_SPEED 1000
#define JOG_SPEED 2000

// System states
enum SystemState {
  STATE_STARTUP,
  STATE_ZEROING,
  STATE_CALIBRATION,
  STATE_IDLE,
  STATE_SWING_UP,
  STATE_BALANCE,
  STATE_EMERGENCY_STOP,
  STATE_MONITORING,   // New state for non-blocking monitor
  STATE_MOVING        // New state for non-blocking moves
};

SystemState currentState = STATE_STARTUP;

// Calibration data
float pendulumZeroAngle = 0.0;
float motorZeroAngle = 0.0;
long stepsAtZero = 0;
long stepsAtLeftLimit = 0;
long stepsAtRightLimit = 0;
float sensorLeftAbsDeg = 0.0f;
float sensorRightAbsDeg = 0.0f;
bool systemCalibrated = false;
bool leftLimitSet = false;   // Track if left limit was set
bool rightLimitSet = false;  // Track if right limit was set
bool zeroPositionSet = false;
float calibratedDegPerStep = DEFAULT_DEG_PER_STEP;

// Motor driver state tracking
bool motorDriverEnabled = false;
bool idleHoldEnabled = false;
bool motorCenteringEnabled = true;
bool calibrationTelemetryPending = false;

String serialCommandBuffer;

enum BalanceLogReason : uint8_t {
  BALANCE_LOG_REASON_FALL = 0,
  BALANCE_LOG_REASON_USER_STOP = 1,
  BALANCE_LOG_REASON_EMERGENCY = 2,
  BALANCE_LOG_REASON_OTHER = 3
};

bool balanceLogActive = false;
bool balanceLogPendingStart = false;
const char *balanceLogOriginLabel = "manual";
uint32_t balanceLogSessionCounter = 0;

extern float Kp_balance;
extern float Ki_balance;
extern float Kd_balance;
extern float Kp_motor;

void requestBalanceLogStart(const char *originLabel);
void cancelBalanceLogStart();
void startBalanceLogIfNeeded();
void endBalanceLog(BalanceLogReason reason);

void refreshCalibrationStatus() {
  systemCalibrated = zeroPositionSet && leftLimitSet && rightLimitSet;
}

float getBaseErrorSteps();
float getBaseErrorDegrees();
void updateSensors();
void resyncBaseFromSensor();
void setMotorEnabled(bool enable);
void applyIdleHoldIfNeeded();

// Current sensor readings
float pendulumAngle = 0.0;  // relative to zero (upright)
uint16_t pendulumRaw = 0;
float motorAngle = 0.0f;     // relative to zero (center)
uint16_t motorRaw = 0;

// Swing-up helper state
int swingTargetDir = 0;     // -1 = left, +1 = right
long swingTargetSteps = 0;

float normalizeAngle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

float signedAngleDelta(float fromDeg, float toDeg) {
  float delta = toDeg - fromDeg;
  while (delta > 180.0f) delta -= 360.0f;
  while (delta < -180.0f) delta += 360.0f;
  return delta;
}

// Handle angle wraparound for derivative calculation
float normalizeAngleDelta(float delta) {
  while (delta > 180.0f) delta -= 360.0f;
  while (delta < -180.0f) delta += 360.0f;
  return delta;
}

float getBaseErrorSteps() {
  return (float)stepper.currentPosition() - (float)stepsAtZero;
}

float getBaseErrorDegrees() {
  // Now returns the ACTUAL sensor angle
  return motorAngle;
}

void resyncBaseFromSensor() {
  // No-op for now, but could be used to correct stepper count if it drifts
}

void updateCalibratedScale() {
  long spanSteps = stepsAtRightLimit - stepsAtLeftLimit;
  if (spanSteps == 0) {
    calibratedDegPerStep = DEFAULT_DEG_PER_STEP;
    return;
  }
  
  float sensorSpanSigned = signedAngleDelta(sensorLeftAbsDeg, sensorRightAbsDeg);
  float sensorSpanAbs = fabs(sensorSpanSigned);
  float stepSpanAbs = fabs((float)spanSteps);
  
  if (stepSpanAbs > 0) {
    calibratedDegPerStep = sensorSpanAbs / stepSpanAbs;
  } else {
    calibratedDegPerStep = DEFAULT_DEG_PER_STEP;
  }
  
  // Sanity check
  if (calibratedDegPerStep < 0.01f || calibratedDegPerStep > 2.0f) {
      calibratedDegPerStep = DEFAULT_DEG_PER_STEP;
  }
}

void requestBalanceLogStart(const char *originLabel) {
  balanceLogOriginLabel = originLabel;
  balanceLogPendingStart = true;
}

void cancelBalanceLogStart() {
  balanceLogPendingStart = false;
}

static void printBalanceLogEndReason(BalanceLogReason reason) {
  switch (reason) {
    case BALANCE_LOG_REASON_FALL:
      Serial.println(F("pendulum_fell"));
      break;
    case BALANCE_LOG_REASON_USER_STOP:
      Serial.println(F("user_stop"));
      break;
    case BALANCE_LOG_REASON_EMERGENCY:
      Serial.println(F("emergency_stop"));
      break;
    default:
      Serial.println(F("other"));
      break;
  }
}

void startBalanceLogIfNeeded() {
  if (!balanceLogPendingStart || balanceLogActive) {
    return;
  }
  balanceLogPendingStart = false;
  balanceLogActive = true;
  balanceLogSessionCounter++;
  unsigned long now = millis();
  Serial.print(F("[BALANCE_LOG_START],"));
  Serial.print(balanceLogSessionCounter);
  Serial.print(F(","));
  Serial.print(now);
  Serial.print(F(","));
  Serial.println(balanceLogOriginLabel);
  Serial.print(F("[BALANCE_GAINS],"));
  Serial.print(Kp_balance, 6);
  Serial.print(F(","));
  Serial.print(Ki_balance, 6);
  Serial.print(F(","));
  Serial.print(Kd_balance, 6);
  Serial.print(F(","));
  Serial.println(Kp_motor, 6);
  Serial.println(F("[BALANCE_COLUMNS],time_s,setpoint_deg,pendulum_deg,stepper_steps,base_deg,control_delta_steps"));
}

void endBalanceLog(BalanceLogReason reason) {
  if (!balanceLogActive && !balanceLogPendingStart) {
    return;
  }
  if (balanceLogActive) {
    unsigned long now = millis();
    Serial.print(F("[BALANCE_LOG_END],"));
    Serial.print(balanceLogSessionCounter);
    Serial.print(F(","));
    Serial.print(now);
    Serial.print(F(","));
    printBalanceLogEndReason(reason);
  }
  balanceLogActive = false;
  balanceLogPendingStart = false;
}

// Control parameters (defaults tuned for 24V stepper build)
// Kp_balance   - pendulum angle proportional gain  (deg)
// Kd_balance   - pendulum angular velocity gain    (deg/s)
// Ki_balance   - integral gain on pendulum angle   (deg * s)
// Kp_motor     - base centering proportional gain  (steps)
// Start with small integral to combat drift at high rate; tune live via serial
float Kp_balance = 6.0f;
float Kd_balance = 1.0f;
float Ki_balance = 0.0f;
float Kp_motor   = 0.20f;

float balanceThreshold = 25.0; // Switch to balance mode (degrees)

// Constants
const float DEG2RAD = PI / 180.0;
const float BALANCE_SWITCH_ANGLE = 12.0f;
const float BALANCE_SWITCH_RATE = 1.5f; // rad/s
const float BALANCE_SWITCH_BASE_DEG = 5.0f;
const long BALANCE_SWITCH_BASE_STEPS = 400;
const long BALANCE_OUTPUT_CLAMP = 900;
const float MOTOR_CENTER_WINDOW_DEG = 60.0f;
// EWMA filter on angle (applied only during BALANCE) at 1 kHz control
const float EWMA_ALPHA_BAL = 0.55f;  // 0.3–0.6 typical; higher = more responsive
// Per-tick slew clamp expressed in degrees at 1 kHz
const float MAX_DEG_PER_TICK_BAL = 180.0f; // matches ~150k steps/s with current virtual scaling

static inline void applyMotionProfile(float maxSpeed, float accel) {
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(accel);
}

static inline void applyHighPerformanceProfile() {
  applyMotionProfile(MOTOR_SPEED, MOTOR_ACCEL);
}

static inline void applySwingProfile() {
  applyMotionProfile(SWING_MAX_SPEED, SWING_ACCEL);
}

static inline void applyBalanceProfile() {
  applyMotionProfile(BALANCE_MAX_SPEED, BALANCE_ACCEL);
}

// Control state
float previousPendulumAngle = 0.0;
float integralError = 0.0;

// High-rate control scheduling
unsigned long lastControlMicros = 0;
#define CONTROL_LOOP_MS 1 // ms helper for places using ms-based math
#define CONTROL_LOOP_US 1000 // 1ms = 1kHz control loop

// Incremental BALANCE state
long balanceTargetSteps = 0;
float balanceTargetStepsF = 0.0f; // floating accumulator to preserve sub-step deltas
float ewmaAngle = 0.0f;
bool ewmaInitialized = false;

// Live tuning removed – commands are single-key menu entries only

// Forward declarations
void checkSerialInput();
void parseSerialCommand(String cmd);
void printMenu();

// ============================================================
//  SENSOR I2C FUNCTIONS
// ============================================================

// Helper for median filter
void insertionSort(uint16_t arr[], int n) {
  int i, j;
  uint16_t key;
  for (i = 1; i < n; i++) {
    key = arr[i];
    j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j = j - 1;
    }
    arr[j + 1] = key;
  }
}

// --- MOTOR SENSOR READ FUNCTIONS (Software I2C) ---
uint16_t readMotorRawAngleSingle() {
  motorWire.beginTransmission(AS5600_ADDRESS);
  motorWire.write(AS5600_RAW_ANGLE_REG);
  if (motorWire.endTransmission() != 0) return 0xFFFF;
  if (motorWire.requestFrom(AS5600_ADDRESS, (uint8_t)2) != 2) return 0xFFFF;
  if (!motorWire.available()) return 0xFFFF;
  uint16_t msb = motorWire.read();
  if (!motorWire.available()) return 0xFFFF;
  uint16_t lsb = motorWire.read();
  return (msb << 8) | lsb;
}

uint16_t readMotorRawAngleMedian() {
  uint16_t samples[3];
  for (int i = 0; i < 3; ++i) {
    samples[i] = readMotorRawAngleSingle();
    if (samples[i] == 0xFFFF) return 0xFFFF; // Fail fast
    delayMicroseconds(10);
  }
  insertionSort(samples, 3);
  return samples[1];
}

float rawToDegrees(uint16_t raw) {
  return (raw * 360.0f) / 4096.0f;
}
// --------------------------------------------------

// Read sensor with a 3-sample median filter for EMI rejection
uint16_t readAngleMedian(AS5600 &sensor) {
  uint16_t readings[3];
  readings[0] = sensor.readAngle();
  delayMicroseconds(10);
  readings[1] = sensor.readAngle();
  delayMicroseconds(10);
  readings[2] = sensor.readAngle();
  insertionSort(readings, 3);
  return readings[1]; // Return median value
}



void updateSensors() {
  // Read pendulum sensor (Hardware I2C)
  pendulumRaw = readAngleMedian(as5600_pendulum);
  float pendulum_deg = (pendulumRaw * 360.0) / 4096.0;
  pendulumAngle = normalizeAngle(pendulum_deg - pendulumZeroAngle);

  // Read motor sensor (Software I2C)
  motorRaw = readMotorRawAngleMedian();
  if (motorRaw != 0xFFFF) {
    float absDeg = rawToDegrees(motorRaw);
    motorAngle = normalizeAngle(absDeg - motorZeroAngle);
  }
  // If read fails, we keep the last known motorAngle (safety)
}

// ============================================================
//  MOTOR DRIVER HELPERS
// ============================================================

void setMotorEnabled(bool enable) {
  digitalWrite(EN_PIN, enable ? LOW : HIGH);
  motorDriverEnabled = enable;
}

void applyIdleHoldIfNeeded() {
  if (currentState != STATE_IDLE && currentState != STATE_STARTUP) {
    return;
  }

  if (idleHoldEnabled && zeroPositionSet) {
    setMotorEnabled(true);
  } else {
    setMotorEnabled(false);
  }
}

// ============================================================
//  STARTUP DIAGNOSTICS
// ============================================================

bool checkSystemHealth() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║      SYSTEM DIAGNOSTICS                    ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
  
  bool allGood = true;
  bool motorStateBeforeTest = motorDriverEnabled;
  bool holdPreference = idleHoldEnabled;
  
  // Check motor driver
  Serial.print(F("Motor Driver... "));
  setMotorEnabled(true);
  delay(100);
  Serial.println(F("✓ Enabled"));
  
  // Check pendulum sensor
  Serial.print(F("Pendulum Sensor (Hardware I2C 0x36)... "));
  if (!as5600_pendulum.detectMagnet()) {
    Serial.println(F("✗ FAILED - No magnet detected!"));
    allGood = false;
  } else {
    uint16_t raw = readAngleMedian(as5600_pendulum);
    Serial.print(F("✓ OK (raw: "));
    Serial.print(raw);
    Serial.println(F(")"));
  }

  // Check motor sensor
  Serial.print(F("Motor Sensor (Software I2C 0x36)... "));
  uint16_t mRaw = readMotorRawAngleMedian();
  if (mRaw == 0xFFFF) {
    Serial.println(F("✗ FAILED - Read error!"));
    allGood = false;
  } else {
    Serial.print(F("✓ OK (raw: "));
    Serial.print(mRaw);
    Serial.println(F(")"));
  }
  
  // Test motor movement
  Serial.print(F("Motor Movement Test... "));
  long startPos = stepper.currentPosition();
  stepper.setMaxSpeed(1000); // Use a known speed
  stepper.moveTo(startPos + 200); // Move 200 steps
  while(stepper.distanceToGo() != 0) {
    stepper.run();
  }
  stepper.moveTo(startPos); // Move back
  while(stepper.distanceToGo() != 0) {
    stepper.run();
  }
  stepper.setCurrentPosition(startPos); // Reset
  applyHighPerformanceProfile();
  Serial.println(F("✓ Motor responds"));
  
  // Restore previous motor state / hold preference
  if (!motorStateBeforeTest) {
    setMotorEnabled(false);
  }
  idleHoldEnabled = holdPreference;
  applyIdleHoldIfNeeded();
  
  Serial.println();
  if (allGood) {
    Serial.println(F("✅ ALL SYSTEMS OPERATIONAL"));
  } else {
    Serial.println(F("❌ SYSTEM CHECK FAILED - Fix errors before proceeding"));
  }
  Serial.println(F("════════════════════════════════════════════\n"));
  
  return allGood;
}

// ============================================================
//  MENU ACTIONS (NOW NON-BLOCKING)
// ============================================================

void performZeroing() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║      ZERO POSITION SETUP                   ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
  Serial.println(F("1. Position the motor arm at CENTER"));
  Serial.println(F("2. Hold the pendulum STRAIGHT UP (vertical)"));
  Serial.println(F("3. Press 'Z' when ready to set zero position\n"));
  
  applyMotionProfile(JOG_SPEED, MOTOR_ACCEL);
  currentState = STATE_ZEROING;
  // Don't block - return and let main loop handle input
}

void liveMonitorSensors() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║      LIVE SENSOR MONITORING                ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
  Serial.println(F("Streaming pendulum sensor + virtual base estimate. Press 'X' to stop.\n"));
  Serial.println(F("────────────────────────────────────────────────────────────"));
  Serial.println(F("Pendulum(°) | Raw Deg | Raw Cnt | Base(° est) | Steps"));
  Serial.println(F("────────────────────────────────────────────────────────────"));
  
  applyMotionProfile(JOG_SPEED, MOTOR_ACCEL);
  currentState = STATE_MONITORING; // Set new state
  // REMOVED BLOCKING while(true) LOOP
  // The main loop() will now handle the printing
}

void calibrateLimits() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║      LIMIT CALIBRATION                     ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
  Serial.println(F("Use 'A'/'D' keys to jog, 'L'/'R' to set limits"));
  Serial.println(F("Press 'Q' to quit calibration\n"));
  
  currentState = STATE_CALIBRATION;
  leftLimitSet = false;   // Reset flags
  rightLimitSet = false;
  calibratedDegPerStep = DEFAULT_DEG_PER_STEP;
  calibrationTelemetryPending = false;
  refreshCalibrationStatus();
  setMotorEnabled(true); // Enable motor for jogging
  applyMotionProfile(JOG_SPEED, MOTOR_ACCEL);
  // REMOVED BLOCKING while(true) LOOP
  // The main loop() will now handle stepper.run() and checkSerialInput()
}

void returnToCenter() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║      RETURNING TO CENTER                   ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
  
  setMotorEnabled(true); // Enable motor
  applyHighPerformanceProfile();
  stepper.moveTo(stepsAtZero);

  currentState = STATE_MOVING; // Set state to MOVING
  // REMOVED BLOCKING while(true) LOOP
  // The main loop() will run the motor
}

// =================================================
//  CONTROL SYSTEM - BANG-BANG SWING-UP
// ============================================================

void runSwingUp() {
  updateSensors();
  
  // Estimate velocity (rad/s)
  float angleDelta = normalizeAngleDelta(pendulumAngle - previousPendulumAngle);
  float alpha_dot = (angleDelta * DEG2RAD) / (CONTROL_LOOP_MS / 1000.0);
  
  previousPendulumAngle = pendulumAngle;
  
  // Check if ready to switch to BALANCE (near upright AND slow)
  float baseErrorSteps = fabs(getBaseErrorSteps());
  float baseErrorDeg = fabs(getBaseErrorDegrees());
  bool baseCentered = (baseErrorDeg <= BALANCE_SWITCH_BASE_DEG) || (baseErrorSteps <= BALANCE_SWITCH_BASE_STEPS);

  if (fabs(pendulumAngle) < BALANCE_SWITCH_ANGLE && fabs(alpha_dot) < BALANCE_SWITCH_RATE && baseCentered) {
    Serial.println(F("\n✓ Pendulum near upright & slow - Switching to BALANCE mode"));
    swingTargetDir = 0;
    currentState = STATE_BALANCE;
    integralError = 0.0;
    // Initialize incremental balance state
    balanceTargetSteps = stepper.currentPosition();
    balanceTargetStepsF = (float)balanceTargetSteps;
    ewmaAngle = pendulumAngle;
    ewmaInitialized = true;
    lastControlMicros = micros();
    requestBalanceLogStart("swing_up");
    // previousPendulumAngle is already set
    return;
  }
  
  // True Bang-Bang Logic with latched targets
  float alpha_rad = pendulumAngle * DEG2RAD;
  bool atTarget = (labs(stepper.distanceToGo()) < 5);
  if (swingTargetDir == 0 || atTarget) {
    int desiredDir = (alpha_dot * cos(alpha_rad) > 0) ? +1 : -1;
    if (desiredDir == 0) desiredDir = (alpha_dot >= 0) ? +1 : -1;
    swingTargetDir = desiredDir;
    swingTargetSteps = (desiredDir > 0) ? stepsAtRightLimit : stepsAtLeftLimit;
    stepper.moveTo(swingTargetSteps);
  } else {
    // keep chasing current target
    stepper.moveTo(swingTargetSteps);
  }
}

// ============================================================
//  CONTROL SYSTEM - PID BALANCE
// ============================================================

void runBalance() {
  updateSensors();
  startBalanceLogIfNeeded();
  
  // Use fixed dt for consistent control (1 kHz)
  const float dt = 0.001f;  // 1 ms
  
  // Check if pendulum fell
  if (abs(pendulumAngle) > 60.0) {
    Serial.println(F("\n⚠ Pendulum fell - Returning to IDLE"));
    endBalanceLog(BALANCE_LOG_REASON_FALL);
    stepper.stop();
    currentState = STATE_IDLE;
    setMotorEnabled(false);
    resyncBaseFromSensor();
    applyIdleHoldIfNeeded();
    return;
  }
  
  // PID control on pendulum angle (treat angle and angular velocity as separate states)
  // EWMA filter on angle for robust derivative
  if (!ewmaInitialized) {
    ewmaAngle = pendulumAngle;
    ewmaInitialized = true;
  } else {
    float dAng = normalizeAngleDelta(pendulumAngle - ewmaAngle);
    ewmaAngle += EWMA_ALPHA_BAL * dAng;
  }

  float error = ewmaAngle; // Track filtered angle to zero
  integralError += error * dt;

  if (fabs(pendulumAngle) > 25.0f) {
    integralError = 0.0f;
  }
  
  // Anti-windup
  if (integralError > 100.0) integralError = 100.0;
  if (integralError < -100.0) integralError = -100.0;
  
  float angleDelta = normalizeAngleDelta(ewmaAngle - previousPendulumAngle);
  float pendulumVelocity = angleDelta / dt;      // deg/s from EWMA
  
  previousPendulumAngle = ewmaAngle;

  // Core control law: angle + angular velocity + integral
  float pidOutput =
      Kp_balance * error +
      Ki_balance * integralError +
      Kd_balance * pendulumVelocity;
  
  // Convert PID output (degrees) into steps before blending with motor centering
  float degPerStep = calibratedDegPerStep;
  if (degPerStep <= 1e-6f) {
    degPerStep = DEFAULT_DEG_PER_STEP;
  }
  float stepsPerDeg = 1.0f / degPerStep;
  pidOutput *= stepsPerDeg;

  // Reintroduce gentle motor centering in BALANCE to prevent drift
  // Use SENSOR angle for centering (converted to equivalent steps)
  float motorAngleSteps = motorAngle / degPerStep;
  float motorCorrection = 0.0f;
  if (motorCenteringEnabled && Kp_motor > 1e-6f) {
    if (fabs(ewmaAngle) < MOTOR_CENTER_WINDOW_DEG) {
      float blend = 1.0f - (fabs(ewmaAngle) / MOTOR_CENTER_WINDOW_DEG);
      motorCorrection = -Kp_motor * motorAngleSteps * blend;
    }
  }
  float totalControl = pidOutput + motorCorrection;
  if (totalControl > BALANCE_OUTPUT_CLAMP) totalControl = BALANCE_OUTPUT_CLAMP;
  if (totalControl < -BALANCE_OUTPUT_CLAMP) totalControl = -BALANCE_OUTPUT_CLAMP;

  // Compute per-tick slew clamp (steps)
  float maxStepsPerTickByAngle = stepsPerDeg * MAX_DEG_PER_TICK_BAL;
  float maxStepsPerTickBySpeed = BALANCE_MAX_SPEED / 1000.0f; // steps per ms at 1 kHz
  float maxSlew = min(maxStepsPerTickByAngle, maxStepsPerTickBySpeed);

  // Incremental target with per-tick clamp
  float desiredTarget = (float)stepsAtZero + totalControl;
  float deltaSteps = desiredTarget - balanceTargetStepsF;
  if (deltaSteps >  maxSlew) deltaSteps =  maxSlew;
  if (deltaSteps < -maxSlew) deltaSteps = -maxSlew;

  // Accumulate target around current position (float preserves sub-step corrections)
  balanceTargetStepsF += deltaSteps;
  if (balanceTargetStepsF < (float)stepsAtLeftLimit)  balanceTargetStepsF = (float)stepsAtLeftLimit;
  if (balanceTargetStepsF > (float)stepsAtRightLimit) balanceTargetStepsF = (float)stepsAtRightLimit;
  long targetSteps = lround(balanceTargetStepsF);

  // Clamp to left/right limits:
  if (targetSteps < stepsAtLeftLimit)  targetSteps = stepsAtLeftLimit;
  if (targetSteps > stepsAtRightLimit) targetSteps = stepsAtRightLimit;

  balanceTargetSteps = targetSteps;
  stepper.moveTo(targetSteps);
  // stepper.run() is called in main loop

  // Telemetry (throttled to 100 Hz to avoid saturating serial at 1 kHz)
  static unsigned long lastTelemetryMicros = 0;
  unsigned long nowUs = micros();
  if (nowUs - lastTelemetryMicros >= 10000) { // 10 ms
    lastTelemetryMicros = nowUs;
    const float telemetryTime = millis() / 1000.0f;
    Serial.print(telemetryTime, 3);              // Column 1: time (s)
    Serial.print(",");
    Serial.print(0.0f, 3);                       // Column 2: pendulum setpoint (deg)
    Serial.print(",");
    Serial.print(pendulumAngle, 3);              // Column 3: pendulum angle (deg)
    Serial.print(",");
    Serial.print(stepper.currentPosition());     // Column 4: stepper position (microsteps)
    Serial.print(",");
    Serial.print(motorAngle, 3);                 // Column 5: virtual motor angle (deg)
    Serial.print(",");
    Serial.println(deltaSteps, 3);               // Column 6: control delta (steps/tick)
  }
}

// ============================================================
//  MAIN MENU & CONTROL TICK
// ============================================================

void printTuningGains() {
  Serial.println(F("\n--- Current Gains (24V Enabled) ---"));
  Serial.print(F("Kp = ")); Serial.println(Kp_balance, 4);
  Serial.print(F("Kd = ")); Serial.println(Kd_balance, 4);
  Serial.print(F("Ki = ")); Serial.println(Ki_balance, 4);
  Serial.print(F("Km = ")); Serial.println(Kp_motor, 4);
  Serial.println(F("------------------------------------"));
}

void printMenu() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║  ROTARY INVERTED PENDULUM CONTROL (v4.8)  ║"));
  Serial.println(F("╚════════════════════════════════════════════╝"));
  Serial.print(F("\nState: "));
  switch(currentState) {
    case STATE_STARTUP: Serial.println(F("STARTUP")); break;
    case STATE_ZEROING: Serial.println(F("ZEROING")); break;
    case STATE_CALIBRATION: Serial.println(F("CALIBRATION")); break;
    case STATE_IDLE: Serial.println(F("IDLE")); break;
    case STATE_SWING_UP: Serial.println(F("SWING-UP ACTIVE")); break;
    case STATE_BALANCE: Serial.println(F("BALANCING")); break;
    case STATE_EMERGENCY_STOP: Serial.println(F("EMERGENCY STOP")); break;
    case STATE_MONITORING: Serial.println(F("MONITORING")); break;
    case STATE_MOVING: Serial.println(F("MOVING")); break;
    default: Serial.println(F("UNKNOWN"));
  }
  Serial.print(F("Calibrated: "));
  Serial.println(systemCalibrated ? F("YES") : F("NO"));
  Serial.print(F("Motor zero: "));
  Serial.println(zeroPositionSet ? F("SET") : F("NOT SET"));
  Serial.print(F("Motor limits: "));
  Serial.println((leftLimitSet && rightLimitSet) ? F("SET") : F("NOT SET"));
  Serial.print(F("Motor hold: "));
  if (!zeroPositionSet) {
    Serial.println(F("OFF (set zero to enable)"));
  } else if (idleHoldEnabled) {
    Serial.println(motorDriverEnabled ? F("ON (base clamped)") : F("ON (will clamp in IDLE)"));
  } else {
    Serial.println(F("OFF"));
  }
  Serial.print(F("Motor centering: "));
  Serial.println(motorCenteringEnabled ? F("ON") : F("OFF"));
  Serial.println(F("\n--- SETUP SEQUENCE ---"));
  Serial.println(F("1 - System Diagnostics"));
  Serial.println(F("2 - Set Zero Position"));
  Serial.println(F("3 - Live Sensor Monitoring"));
  Serial.println(F("4 - Calibrate Left/Right Limits"));
  Serial.println(F("5 - Return to Center"));
  Serial.println(F("\n--- CONTROL ---"));
  Serial.println(F("S - Start Swing-Up & Balance"));
  Serial.println(F("B - Balance Test (manual start from upright)"));
  Serial.println(F("H - Toggle motor hold (clamp/release base)"));
  Serial.println(F("C - Toggle motor centering term (disabled in BALANCE)"));
  Serial.println(F("P<x> - Set Kp  (e.g. P1.8)"));
  Serial.println(F("I<x> - Set Ki  (e.g. I0.05)"));
  Serial.println(F("D<x> - Set Kd  (e.g. D0.3)"));
  Serial.println(F("M<x> - Set Kp_motor (e.g. M0.05)"));
  Serial.println(F("X - Stop Control"));
  Serial.println(F("E - Emergency Stop"));
  Serial.println(F("T - Show Current Tuning Gains"));
  Serial.println(F("════════════════════════════════════════════"));
  Serial.print(F("Choice: "));
}

// New function to run control logic at a fixed rate
void controlTick() {
  // This function is called every 1ms by the timer in loop()
  if (currentState == STATE_SWING_UP) {
    runSwingUp();
  } else if (currentState == STATE_BALANCE) {
    runBalance();
  }
}

// ============================================================
//  NEW: NON-BLOCKING COMMAND PARSER
// ============================================================

void parseSerialCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.length() > 1) {
    char prefix = cmd.charAt(0);
    char prefixUpper = toupper((unsigned char)prefix);
    if (prefixUpper == 'P' || prefixUpper == 'I' || prefixUpper == 'D' || prefixUpper == 'M') {
      String valueStr = cmd.substring(1);
      valueStr.trim();
      if (valueStr.length() == 0) {
        switch (prefixUpper) {
          case 'P':
            Serial.println(F("\nUsage: P<value>  e.g. P1.8"));
            break;
          case 'I':
            Serial.println(F("\nUsage: I<value>  e.g. I0.05"));
            break;
          case 'D':
            Serial.println(F("\nUsage: D<value>  e.g. D0.3"));
            break;
          case 'M':
            Serial.println(F("\nUsage: M<value>  e.g. M0.05"));
            break;
        }
      } else {
        float newValue = valueStr.toFloat();
        switch (prefixUpper) {
          case 'P':
            Kp_balance = newValue;
            Serial.print(F("\n✓ Kp updated to "));
            Serial.println(Kp_balance, 4);
            break;
          case 'I':
            Ki_balance = newValue;
            integralError = 0.0f;  // reset accumulated error when Ki changes
            Serial.print(F("\n✓ Ki updated to "));
            Serial.println(Ki_balance, 4);
            break;
          case 'D':
            Kd_balance = newValue;
            Serial.print(F("\n✓ Kd updated to "));
            Serial.println(Kd_balance, 4);
            break;
          case 'M':
            Kp_motor = newValue;
            Serial.print(F("\n✓ Kp_motor updated to "));
            Serial.println(Kp_motor, 4);
            break;
        }
      }
      printTuningGains();
      return;
    }
  }

  char commandType = cmd.charAt(0);

  // These commands work in ANY state (Stop/Status)
  switch (commandType) {
    case 'T': case 't':
      printTuningGains();
      return;
    case 'H': case 'h':
      if (currentState == STATE_SWING_UP || currentState == STATE_BALANCE || currentState == STATE_MOVING) {
        Serial.println(F("\n⚠ Stop control ('X') before toggling motor hold."));
        return;
      }
      if (!zeroPositionSet) {
        Serial.println(F("\n⚠ Set zero (option 2) before enabling motor hold."));
        return;
      }
      idleHoldEnabled = !idleHoldEnabled;
      if (idleHoldEnabled) {
        Serial.println(F("\n✓ Motor hold ENABLED - base clamped"));
      } else {
        Serial.println(F("\n✓ Motor hold DISABLED - base free"));
      }
      applyIdleHoldIfNeeded();
      printMenu();
      return;
    case 'C': case 'c':
      motorCenteringEnabled = !motorCenteringEnabled;
      if (motorCenteringEnabled) {
        Serial.println(F("\n✓ Motor centering ENABLED"));
        resyncBaseFromSensor();
      } else {
        Serial.println(F("\n✓ Motor centering DISABLED"));
      }
      printMenu();
      return;
    case 'X': case 'x':
      stepper.stop();
      setMotorEnabled(false);
      if (currentState == STATE_BALANCE) {
        endBalanceLog(BALANCE_LOG_REASON_USER_STOP);
      } else if (currentState == STATE_SWING_UP) {
        cancelBalanceLogStart();
      }
      if (currentState == STATE_MONITORING) {
        Serial.println(F("\n✓ Monitoring stopped"));
      } else if (currentState == STATE_SWING_UP || currentState == STATE_BALANCE) {
        Serial.println(F("\n✓ Control stopped"));
        swingTargetDir = 0;
        resyncBaseFromSensor();
      } else if (currentState == STATE_CALIBRATION) {
        calibrationTelemetryPending = false;
        Serial.println(F("\n✓ Calibration stopped"));
      }
      currentState = STATE_IDLE;
      applyIdleHoldIfNeeded();
      printMenu();
      return;
    case 'E': case 'e':
      stepper.stop();
      idleHoldEnabled = false;
      setMotorEnabled(false);
      endBalanceLog(BALANCE_LOG_REASON_EMERGENCY);
      currentState = STATE_EMERGENCY_STOP;
      swingTargetDir = 0;
      resyncBaseFromSensor();
      Serial.println(F("\n!!! EMERGENCY STOP !!!"));
      Serial.println(F("Power cycle or reset to continue.\n"));
      return;
  }

  // --- Menu commands below only work if NOT in a control state ---
  if (currentState == STATE_SWING_UP || currentState == STATE_BALANCE || currentState == STATE_MOVING) {
    Serial.println(F("Must press 'X' to stop control before giving menu commands."));
    return;
  }
  
  // Special handling for calibration mode commands
  if (currentState == STATE_CALIBRATION) {
    switch (commandType) {
      case 'a': case 'A':
        stepper.move(-50);
        calibrationTelemetryPending = true;
        Serial.println(F("← Jogging Left 50 steps..."));
        return;
      case 'd': case 'D':
        stepper.move(50);
        calibrationTelemetryPending = true;
        Serial.println(F("→ Jogging Right 50 steps..."));
        return;
      case 'l': case 'L':
        stepsAtLeftLimit = stepper.currentPosition();
        // Record current absolute motor angle
        motorRaw = readMotorRawAngleMedian();
        if (motorRaw != 0xFFFF) {
           sensorLeftAbsDeg = rawToDegrees(motorRaw);
        }
        leftLimitSet = true;
        Serial.print(F("\n✓ LEFT LIMIT SET: "));
        Serial.print(stepsAtLeftLimit);
        Serial.print(F(" steps | Sensor Abs: "));
        Serial.print(sensorLeftAbsDeg, 2);
        Serial.println(F("°\n"));
        return;
      case 'r': case 'R':
        stepsAtRightLimit = stepper.currentPosition();
        // Record current absolute motor angle
        motorRaw = readMotorRawAngleMedian();
        if (motorRaw != 0xFFFF) {
           sensorRightAbsDeg = rawToDegrees(motorRaw);
        }
        rightLimitSet = true;
        Serial.print(F("\n✓ RIGHT LIMIT SET: "));
        Serial.print(stepsAtRightLimit);
        Serial.print(F(" steps | Sensor Abs: "));
        Serial.print(sensorRightAbsDeg, 2);
        Serial.println(F("°\n"));
        return;
      case 'q': case 'Q':
        if (leftLimitSet && rightLimitSet) {
          if (stepsAtLeftLimit > stepsAtRightLimit) {
            Serial.println(F("   (i) Note: Limits were entered backward and have been swapped."));
            long tmp = stepsAtLeftLimit;
            stepsAtLeftLimit = stepsAtRightLimit;
            stepsAtRightLimit = tmp;
            // Swap sensor limits too
            float tmpDeg = sensorLeftAbsDeg;
            sensorLeftAbsDeg = sensorRightAbsDeg;
            sensorRightAbsDeg = tmpDeg;
          }
          updateCalibratedScale();
          refreshCalibrationStatus();
          currentState = STATE_IDLE;
          calibrationTelemetryPending = false;
          applyIdleHoldIfNeeded();
          stepper.setMaxSpeed(MOTOR_SPEED);
          stepper.setAcceleration(MOTOR_ACCEL);
          Serial.println(F("\n✅ CALIBRATION COMPLETE!"));
          long spanSteps = stepsAtRightLimit - stepsAtLeftLimit;
          Serial.print(F("   Step Range: ")); Serial.print(spanSteps); Serial.println(F(" steps"));
          Serial.print(F("   Virtual base span (assumed): +/-"));
          Serial.print(ASSUMED_BASE_LIMIT_DEG, 1);
          Serial.println(F("°"));
          Serial.print(F("   Calibrated deg/step: "));
          Serial.println(calibratedDegPerStep, 4);
          Serial.println(F("════════════════════════════════════════════\n"));
          printMenu();
        } else {
          Serial.println(F("\n⚠ Must set both LEFT and RIGHT limits before quitting!\n"));
        }
        return;
    }
  }

  // Special handling for zeroing mode
  if (currentState == STATE_ZEROING) {
    if (commandType == 'z' || commandType == 'Z') {
      stepper.setCurrentPosition(0);
      stepsAtZero = stepper.currentPosition();
      pendulumZeroAngle = UPRIGHT_PENDULUM_DEG;
      
      // Record motor zero
      motorRaw = readMotorRawAngleMedian();
      if (motorRaw != 0xFFFF) {
        motorZeroAngle = rawToDegrees(motorRaw);
      }
      
      Serial.println(F("\n✅ ZERO POSITION SET!"));
      Serial.print(F("   Pendulum zero: ")); Serial.print(pendulumZeroAngle, 2); Serial.println(F("°"));
      Serial.print(F("   Motor zero abs: ")); Serial.print(motorZeroAngle, 2); Serial.println(F("°"));
      
      motorAngle = 0.0f;
      Serial.println(F("════════════════════════════════════════════\n"));
      updateSensors();
      previousPendulumAngle = pendulumAngle;
      zeroPositionSet = true;
      refreshCalibrationStatus();
      if (!idleHoldEnabled) {
        idleHoldEnabled = true;
        Serial.println(F("   Motor hold enabled to preserve zero (toggle with 'H')."));
      }
      currentState = STATE_IDLE;
      applyIdleHoldIfNeeded();
      printMenu();
    }
    return;
  }

  // If user entered tuning command without a value, provide usage help
  if (commandType == 'P' || commandType == 'p') {
    Serial.println(F("\nUsage: P<value>  e.g. P1.8"));
    return;
  }
  if (commandType == 'I' || commandType == 'i') {
    Serial.println(F("\nUsage: I<value>  e.g. I0.05"));
    return;
  }
  if (commandType == 'D' || commandType == 'd') {
    Serial.println(F("\nUsage: D<value>  e.g. D0.3"));
    return;
  }
  if (commandType == 'M' || commandType == 'm') {
    Serial.println(F("\nUsage: M<value>  e.g. M0.05"));
    return;
  }

  // --- Standard Menu Commands (must be in IDLE or STARTUP) ---
  if (currentState == STATE_IDLE || currentState == STATE_STARTUP) {
    switch (commandType) {
      case '1':
        checkSystemHealth();
        printMenu();
        break;
      case '2':
        performZeroing();
        // Menu will be shown after 'Z' is pressed
        break;
      case '3':
        liveMonitorSensors();
        // printMenu() is NOT called, 'X' will call it
        break;
      case '4':
        if (!zeroPositionSet) {
          Serial.println(F("\nMust set zero position first (option 2).\n"));
          printMenu();
        } else {
          calibrateLimits();
          // printMenu() is called by parseSerialCommand on 'Q'
        }
        break;
      case '5':
        if (!systemCalibrated) {
          Serial.println(F("\nMust calibrate limits first (option 4).\n"));
          printMenu();
        } else {
          returnToCenter();
          // printMenu() will be called from main loop when move finishes
        }
        break;
      case 's': case 'S':
        if (!systemCalibrated) {
          Serial.println(F("\n⚠ Cannot start - System not calibrated!"));
          Serial.println(F("Complete setup sequence (options 1-5) first.\n"));
          printMenu();
        } else {
          Serial.println(F("\n╔════════════════════════════════════════════╗"));
          Serial.println(F("║  STARTING SWING-UP CONTROL                 ║"));
          Serial.println(F("╚════════════════════════════════════════════╝\n"));
          Serial.println(F("Pull pendulum down, then release..."));
          Serial.println(F("Press 'X' to stop.\n"));
          resyncBaseFromSensor();
          setMotorEnabled(true);
          applySwingProfile();
          updateSensors();
          previousPendulumAngle = pendulumAngle;
          swingTargetDir = 0;
          delay(500); // Give user time to release
          currentState = STATE_SWING_UP;
          lastControlMicros = micros();
        }
        break;
      case 'b': case 'B':
        if (!systemCalibrated) {
          Serial.println(F("\n⚠ Cannot start - System not calibrated!"));
          Serial.println(F("Complete setup sequence (options 1-5) first.\n"));
          printMenu();
        } else {
          Serial.println(F("\n╔════════════════════════════════════════════╗"));
          Serial.println(F("║  BALANCE TEST MODE                         ║"));
          Serial.println(F("╚════════════════════════════════════════════╝\n"));
          Serial.println(F("Hold pendulum near upright, then release..."));
          Serial.println(F("Press 'X' to stop.\n"));
          resyncBaseFromSensor();
          setMotorEnabled(true);
          applyBalanceProfile();
          updateSensors();
          previousPendulumAngle = pendulumAngle;
          integralError = 0.0;
          swingTargetDir = 0;
          // Initialize incremental balance state
          balanceTargetSteps = stepper.currentPosition();
          balanceTargetStepsF = (float)balanceTargetSteps;
          ewmaAngle = pendulumAngle;
          ewmaInitialized = true;
          delay(500); // Give user time to release
          currentState = STATE_BALANCE;
          lastControlMicros = micros();
          requestBalanceLogStart("manual");
        }
        break;
      
      default:
        Serial.println(F("\nInvalid choice."));
        printMenu();
        break;
    }
  }
}

// New non-blocking function to check for serial commands
void checkSerialInput() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\r') {
      // Ignore carriage return, wait for newline
      continue;
    }

    if (inChar == '\n') {
      Serial.println();
      if (serialCommandBuffer.length() > 0) {
        parseSerialCommand(serialCommandBuffer);
        serialCommandBuffer = "";
      }
      continue;
    }

    if (inChar == '\b' || inChar == 0x7F) {
      if (serialCommandBuffer.length() > 0) {
        serialCommandBuffer.remove(serialCommandBuffer.length() - 1);
        Serial.print("\b \b");
      }
      continue;
    }

    if (inChar < ' ') {
      continue;
    }

    Serial.print(inChar);
    serialCommandBuffer += inChar;
  }
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  // Initialize pins
  pinMode(EN_PIN, OUTPUT);
  setMotorEnabled(false); // Disabled initially
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  
  // Initialize serial
  Serial.begin(115200);
  delay(2000);
  
  Serial.println(F("\n\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║  ROTARY INVERTED PENDULUM - FULL SYSTEM   ║"));
  Serial.println(F("║  Step-Based Calibration & Control (v4.8)  ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
  
  // Configure stepper
  stepper.setMaxSpeed(MOTOR_SPEED);
  stepper.setAcceleration(MOTOR_ACCEL);
  stepper.setCurrentPosition(0);
  
  // Initialize Hardware I2C (pendulum sensor)
  Wire.begin();
  Wire.setClock(400000); // 400kHz for faster sensor reads (median at 1kHz control)
  delay(100);
  as5600_pendulum.begin();

  // Initialize Software I2C (motor sensor)
  motorWire.begin();
  Serial.println(F("Software I2C sensor bus initialized (SDA=22, SCL=24)."));
  
  Serial.println(F("System initialized. Run diagnostics (option 1) first.\n"));
  Serial.flush(); // Ensure output is sent
  
  currentState = STATE_STARTUP;
  printMenu();
  Serial.flush(); // Ensure menu is sent
}

// ============================================================
//  MAIN LOOP (NEW ARCHITECTURE)
// ============================================================

void loop() {
  // Main loop must run as fast as possible
  
  // 1. Run the non-blocking serial command checker
  checkSerialInput();

  // 2. Handle control logic at a fixed rate (1 kHz)
  unsigned long now = millis();
  unsigned long nowUs = micros();
  if ((nowUs - lastControlMicros) >= CONTROL_LOOP_US) {
    lastControlMicros = nowUs;
    if (currentState == STATE_SWING_UP || currentState == STATE_BALANCE) {
      controlTick();
    }
  }

  // 3. Handle sensor monitoring display (if active)
  static unsigned long lastMonitorPrint = 0;
  if (currentState == STATE_MONITORING && (now - lastMonitorPrint >= 200)) {
    lastMonitorPrint = now;
    updateSensors();
    float pendulumRawDeg = (pendulumRaw * 360.0f) / 4096.0f;
    Serial.print(F("  "));
    Serial.print(pendulumAngle, 2);
    Serial.print(F("°   \t"));
    Serial.print(pendulumRawDeg, 2);
    Serial.print(F("°   \t"));
    Serial.print(pendulumRaw);
    Serial.print(F("  \t"));
    Serial.print(motorAngle, 2);
    Serial.print(F("°   \t"));
    Serial.println(stepper.currentPosition());
  }

  // 4. Run the stepper motor
  // This must be run for ALL states that can move the motor
  if (currentState == STATE_SWING_UP || 
      currentState == STATE_BALANCE || 
      currentState == STATE_CALIBRATION || 
      currentState == STATE_MOVING) {
    stepper.run();
  }

  // 5. Check for completion of non-blocking moves
  if (currentState == STATE_MOVING && stepper.distanceToGo() == 0) {
    // The "returnToCenter" move has finished
    currentState = STATE_IDLE;
    setMotorEnabled(false);
    resyncBaseFromSensor();
    applyIdleHoldIfNeeded();
    Serial.print(F("✓ At center | Steps: "));
    Serial.println(stepper.currentPosition());
    Serial.println(F("════════════════════════════════════════════\n"));
    printMenu();
  }

  if (currentState == STATE_CALIBRATION && calibrationTelemetryPending && stepper.distanceToGo() == 0) {
    calibrationTelemetryPending = false;
    updateSensors();
    Serial.print(F("   Steps: "));
    Serial.print(stepper.currentPosition());
    Serial.print(F(" | Base est: "));
    Serial.print(motorAngle, 2);
    Serial.print(F("° | Pendulum: "));
    Serial.print(pendulumAngle, 2);
    Serial.print(F("° (raw "));
    Serial.print(pendulumRaw);
    Serial.println(F(")"));
  }
  
  // The old blocking serial menu is GONE.
  // All commands are handled by checkSerialInput() -> parseSerialCommand()
}
