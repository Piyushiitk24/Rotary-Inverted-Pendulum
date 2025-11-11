#include <Arduino.h>
#include <Wire.h>
#include <SoftwareWire.h>
#include <AS5600.h>
#include <AccelStepper.h>
#include <math.h>

// ============================================================
//  ROTARY INVERTED PENDULUM - STEPPER-BASED CONTROL
// ============================================================
//
// V4.8 - 24V POWER ENABLED
// - 24V Power Supply
// - TMC2209 Vref = 2.11V (for 1.5A RMS)
// - Pendulum Sensor: AS5600 (Hardware I2C)
// - Motor Position: Stepper Count (Virtual Encoder)
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
#define MOTOR_SDA_PIN 22
#define MOTOR_SCL_PIN 24
#define AS5600_ADDRESS 0x36
#define AS5600_RAW_ANGLE_REG 0x0C
#define AS5600_STATUS_REG 0x0B
SoftwareWire motorWire(MOTOR_SDA_PIN, MOTOR_SCL_PIN);

// Motor configuration
// --- 24V HIGH PERFORMANCE VALUES ---
// With 24V and 1.5A, the motor can now handle high speeds.
#define MOTOR_SPEED 12000     // (Was 4000)
#define MOTOR_ACCEL 18000     // (Was 8000)
#define SWING_MAX_SPEED 20000
#define SWING_ACCEL 80000
#define BALANCE_MAX_SPEED 15000
#define BALANCE_ACCEL 40000
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
long stepsAtZero = 0;
long stepsAtLeftLimit = 0;
long stepsAtRightLimit = 0;
bool systemCalibrated = false;
bool leftLimitSet = false;   // Track if left limit was set
bool rightLimitSet = false;  // Track if right limit was set
bool zeroPositionSet = false;

// Motor sensor calibration
float motorZeroAngle = 0.0f;
float motorLeftLimitAngle = 0.0f;   // relative after calibration
float motorRightLimitAngle = 0.0f;
float motorLeftLimitAbs = 0.0f;
float motorRightLimitAbs = 0.0f;
float motorStepsPerDeg = 1.0f;
bool motorScaleReady = false;

// Motor driver state tracking
bool motorDriverEnabled = false;
bool idleHoldEnabled = false;

void refreshCalibrationStatus() {
  systemCalibrated = zeroPositionSet && leftLimitSet && rightLimitSet && motorScaleReady;
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

float getBaseErrorSteps() {
  return (float)stepper.currentPosition() - (float)stepsAtZero;
}

float getBaseErrorDegrees() {
  if (motorScaleReady && fabs(motorStepsPerDeg) > 1e-3f) {
    return motorAngle;
  }
  float stepError = getBaseErrorSteps();
  float spanSteps = (float)(stepsAtRightLimit - stepsAtLeftLimit);
  if (fabs(spanSteps) < 1e-3f) return stepError;
  // approximate conversion using measured step span vs pendulum (assuming ~30 deg span)
  float approxDegPerStep = 30.0f / spanSteps;
  return stepError * approxDegPerStep;
}

void resyncBaseFromSensor() {
  if (!motorScaleReady || fabs(motorStepsPerDeg) < 1e-3f) {
    return;
  }
  updateSensors();
  long sensorSteps = (long)lroundf(motorAngle * motorStepsPerDeg);
  stepper.setCurrentPosition(stepsAtZero + sensorSteps);
}

// Control parameters (static defaults tuned for lightweight PLA build)
const float Kp_balance = 4.0f;
const float Kd_balance = 0.8f;
const float Ki_balance = 0.05f;
const float Kp_motor = 0.03f;   // proportional gain on step-count error to keep arm centered

float balanceThreshold = 25.0; // Switch to balance mode (degrees)

// Constants
const float DEG2RAD = PI / 180.0;
const float BALANCE_SWITCH_ANGLE = 12.0f;
const float BALANCE_SWITCH_RATE = 1.5f; // rad/s
const float BALANCE_SWITCH_BASE_DEG = 5.0f;
const long BALANCE_SWITCH_BASE_STEPS = 400;
const long BALANCE_OUTPUT_CLAMP = 800;

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
unsigned long lastControlTime = 0;
#define CONTROL_LOOP_MS 10 // 10ms = 100Hz control loop

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

// Read sensor with a 3-sample median filter for EMI rejection
uint16_t readAngleMedian(AS5600 &sensor) {
  uint16_t readings[3];
  readings[0] = sensor.readAngle();
  delayMicroseconds(100);
  readings[1] = sensor.readAngle();
  delayMicroseconds(100);
  readings[2] = sensor.readAngle();
  insertionSort(readings, 3);
  return readings[1]; // Return median value
}

uint16_t readMotorRawAngleSingle() {
  motorWire.beginTransmission(AS5600_ADDRESS);
  motorWire.write(AS5600_RAW_ANGLE_REG);
  if (motorWire.endTransmission() != 0) {
    return 0xFFFF;
  }
  if (motorWire.requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)2) != 2) {
    return 0xFFFF;
  }
  if (!motorWire.available()) {
    return 0xFFFF;
  }
  uint16_t msb = motorWire.read();
  if (!motorWire.available()) {
    return 0xFFFF;
  }
  uint16_t lsb = motorWire.read();
  return (msb << 8) | lsb;
}

uint16_t readMotorRawAngleMedian() {
  uint16_t samples[3];
  for (int i = 0; i < 3; ++i) {
    samples[i] = readMotorRawAngleSingle();
    if (samples[i] == 0xFFFF) {
      return 0xFFFF;
    }
    delayMicroseconds(100);
  }
  insertionSort(samples, 3);
  return samples[1];
}

bool detectAS5600Magnet(SoftwareWire &wire) {
  wire.beginTransmission(AS5600_ADDRESS);
  wire.write(AS5600_STATUS_REG);
  if (wire.endTransmission() != 0) {
    return false;
  }
  if (wire.requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)1) != 1) {
    return false;
  }
  if (!wire.available()) {
    return false;
  }
  uint8_t status = wire.read();
  return (status & 0x20) != 0; // MD bit
}

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

void updateSensors() {
  // Read pendulum sensor (Hardware I2C)
  pendulumRaw = readAngleMedian(as5600_pendulum);
  float pendulum_deg = (pendulumRaw * 360.0) / 4096.0;
  pendulumAngle = normalizeAngle(pendulum_deg - pendulumZeroAngle);

  // Read motor sensor (Software I2C)
  uint16_t raw = readMotorRawAngleMedian();
  if (raw != 0xFFFF) {
    motorRaw = raw;
    float motor_deg = (motorRaw * 360.0f) / 4096.0f;
    motorAngle = normalizeAngle(motor_deg - motorZeroAngle);
  }
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
  if (!detectAS5600Magnet(motorWire)) {
    Serial.println(F("✗ FAILED - No magnet detected!"));
    allGood = false;
  } else {
    uint16_t raw = readMotorRawAngleMedian();
    if (raw == 0xFFFF) {
      Serial.println(F("✗ FAILED - I2C read error"));
      allGood = false;
    } else {
      Serial.print(F("✓ OK (raw: "));
      Serial.print(raw);
      Serial.println(F(")"));
    }
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
  
  currentState = STATE_ZEROING;
  // Don't block - return and let main loop handle input
}

void liveMonitorSensors() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║      LIVE SENSOR MONITORING                ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
  Serial.println(F("Streaming pendulum sensor. Press 'X' to stop.\n"));
  Serial.println(F("────────────────────────────────────────────────"));
  Serial.println(F("Pendulum(°) | Pend Raw | Motor(°) | Motor Raw | Steps"));
  Serial.println(F("────────────────────────────────────────────────"));
  
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
  motorScaleReady = false;
  refreshCalibrationStatus();
  setMotorEnabled(true); // Enable motor for jogging
  stepper.setMaxSpeed(JOG_SPEED);
  stepper.setAcceleration(MOTOR_ACCEL);
  // REMOVED BLOCKING while(true) LOOP
  // The main loop() will now handle stepper.run() and checkSerialInput()
}

void returnToCenter() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║      RETURNING TO CENTER                   ║"));
  Serial.println(F("╚════════════════════════════════════════════╝\n"));
  
  setMotorEnabled(true); // Enable motor
  applyHighPerformanceProfile();
  long targetSteps = stepsAtZero;
  if (motorScaleReady && fabs(motorStepsPerDeg) > 1e-3f) {
    updateSensors();
    float correctionSteps = motorAngle * motorStepsPerDeg;
    targetSteps = stepper.currentPosition() - (long)lroundf(correctionSteps);
  }
  stepper.moveTo(targetSteps);

  currentState = STATE_MOVING; // Set state to MOVING
  // REMOVED BLOCKING while(true) LOOP
  // The main loop() will run the motor
}

// ============================================================
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
    lastControlTime = millis();
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
  
  // Use fixed dt for consistent control
  const float dt = CONTROL_LOOP_MS / 1000.0;  // 0.01s
  
  // Check if pendulum fell
  if (abs(pendulumAngle) > 60.0) {
    Serial.println(F("\n⚠ Pendulum fell - Returning to IDLE"));
    stepper.stop();
    currentState = STATE_IDLE;
    setMotorEnabled(false);
    resyncBaseFromSensor();
    applyIdleHoldIfNeeded();
    return;
  }
  
  // PID control on pendulum angle
  float error = pendulumAngle; // Want it at 0°
  integralError += error * dt;
  
  // Anti-windup
  if (integralError > 100.0) integralError = 100.0;
  if (integralError < -100.0) integralError = -100.0;
  
  float angleDelta = normalizeAngleDelta(pendulumAngle - previousPendulumAngle);
  float derivative = angleDelta / dt;
  
  previousPendulumAngle = pendulumAngle;
  
  float pidOutput = Kp_balance * error + Ki_balance * integralError + Kd_balance * derivative;
  
  // Also include motor position feedback (keep centered)
  float motorStepError = getBaseErrorSteps();
  if (motorScaleReady) {
    motorStepError = motorAngle * motorStepsPerDeg;
  }
  float motorCorrection = Kp_motor * motorStepError;
  
  float totalControl = pidOutput + motorCorrection;

  if (totalControl > BALANCE_OUTPUT_CLAMP) totalControl = BALANCE_OUTPUT_CLAMP;
  if (totalControl < -BALANCE_OUTPUT_CLAMP) totalControl = -BALANCE_OUTPUT_CLAMP;
  
  // CRITICAL LOGIC FIX (v4.3)
  long targetSteps = stepsAtZero + (long)(totalControl);
  
  // Enforce limits
  if (targetSteps < stepsAtLeftLimit) targetSteps = stepsAtLeftLimit;
  if (targetSteps > stepsAtRightLimit) targetSteps = stepsAtRightLimit;
  
  stepper.moveTo(targetSteps);
  // stepper.run() is called in main loop
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
  Serial.println(F("X - Stop Control"));
  Serial.println(F("E - Emergency Stop"));
  Serial.println(F("T - Show Current Tuning Gains"));
  Serial.println(F("════════════════════════════════════════════"));
  Serial.print(F("Choice: "));
}

// New function to run control logic at a fixed rate
void controlTick() {
  // This function is called every 10ms by the timer in loop()
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
  if (cmd.length() == 0) return;
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
    case 'X': case 'x':
      stepper.stop();
      setMotorEnabled(false);
      if (currentState == STATE_MONITORING) {
        Serial.println(F("\n✓ Monitoring stopped"));
      } else if (currentState == STATE_SWING_UP || currentState == STATE_BALANCE) {
        Serial.println(F("\n✓ Control stopped"));
        swingTargetDir = 0;
        resyncBaseFromSensor();
      } else if (currentState == STATE_CALIBRATION) {
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
        Serial.println(F("← Jogging Left 50 steps..."));
        return;
      case 'd': case 'D':
        stepper.move(50);
        Serial.println(F("→ Jogging Right 50 steps..."));
        return;
      case 'l': case 'L':
        stepsAtLeftLimit = stepper.currentPosition();
        {
          uint16_t raw = readMotorRawAngleMedian();
          if (raw != 0xFFFF) {
            float absDeg = (raw * 360.0f) / 4096.0f;
            motorLeftLimitAbs = absDeg;
            motorLeftLimitAngle = normalizeAngle(absDeg - motorZeroAngle);
          }
        }
        leftLimitSet = true;
        Serial.print(F("\n✓ LEFT LIMIT SET: ")); Serial.print(stepsAtLeftLimit); Serial.print(F(" steps | Motor rel: "));
        Serial.print(motorLeftLimitAngle, 2); Serial.println(F("°\n"));
        return;
      case 'r': case 'R':
        stepsAtRightLimit = stepper.currentPosition();
        {
          uint16_t raw = readMotorRawAngleMedian();
          if (raw != 0xFFFF) {
            float absDeg = (raw * 360.0f) / 4096.0f;
            motorRightLimitAbs = absDeg;
            motorRightLimitAngle = normalizeAngle(absDeg - motorZeroAngle);
          }
        }
        rightLimitSet = true;
        Serial.print(F("\n✓ RIGHT LIMIT SET: ")); Serial.print(stepsAtRightLimit); Serial.print(F(" steps | Motor rel: "));
        Serial.print(motorRightLimitAngle, 2); Serial.println(F("°\n"));
        return;
      case 'q': case 'Q':
        if (leftLimitSet && rightLimitSet) {
          if (stepsAtLeftLimit > stepsAtRightLimit) {
            Serial.println(F("   (i) Note: Limits were entered backward and have been swapped."));
            long tmp = stepsAtLeftLimit;
            stepsAtLeftLimit = stepsAtRightLimit;
            stepsAtRightLimit = tmp;
          }
          float spanDeg = signedAngleDelta(motorLeftLimitAbs, motorRightLimitAbs);
          if (fabs(spanDeg) > 1e-3f) {
            motorStepsPerDeg = (float)(stepsAtRightLimit - stepsAtLeftLimit) / spanDeg;
            motorScaleReady = true;
            float midpointAbs = motorLeftLimitAbs + spanDeg * 0.5f;
            while (midpointAbs > 360.0f) midpointAbs -= 360.0f;
            while (midpointAbs < 0.0f) midpointAbs += 360.0f;
            motorZeroAngle = midpointAbs;
            // Update stored relative values based on new zero
            motorLeftLimitAngle = normalizeAngle(motorLeftLimitAbs - motorZeroAngle);
            motorRightLimitAngle = normalizeAngle(motorRightLimitAbs - motorZeroAngle);
            updateSensors();
          }
          refreshCalibrationStatus();
          currentState = STATE_IDLE;
          applyIdleHoldIfNeeded();
          stepper.setMaxSpeed(MOTOR_SPEED);
          stepper.setAcceleration(MOTOR_ACCEL);
          Serial.println(F("\n✅ CALIBRATION COMPLETE!"));
          Serial.print(F("   Step Range: ")); Serial.print(stepsAtRightLimit - stepsAtLeftLimit); Serial.println(F(" steps")); 
          Serial.print(F("   Motor range: "));
          Serial.print(spanDeg, 2);
          Serial.println(F("°"));
          Serial.print(F("   Motor deg/step: "));
          Serial.println(motorStepsPerDeg, 4);
          Serial.print(F("   Left/Right rel: "));
          Serial.print(motorLeftLimitAngle, 2);
          Serial.print(F("° / "));
          Serial.print(motorRightLimitAngle, 2);
          Serial.println(F("°"));
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
      stepsAtZero = 0;
      uint16_t pendulum_raw = readAngleMedian(as5600_pendulum);
      pendulumZeroAngle = (pendulum_raw * 360.0) / 4096.0;
      uint16_t motor_raw = readMotorRawAngleMedian();
      float motorAbsDeg = NAN;
      if (motor_raw != 0xFFFF) {
        motorAbsDeg = (motor_raw * 360.0f) / 4096.0f;
        motorZeroAngle = motorAbsDeg;
        motorAngle = 0.0f;
      } else {
        Serial.println(F("⚠ Motor sensor read failed while zeroing."));
      }
      Serial.println(F("\n✅ ZERO POSITION SET!"));
      Serial.print(F("   Pendulum zero: ")); Serial.print(pendulumZeroAngle, 2); Serial.println(F("°"));
      if (!isnan(motorAbsDeg)) {
        Serial.print(F("   Motor zero abs: ")); Serial.print(motorAbsDeg, 2); Serial.println(F("° (reference)"));
      } else {
        Serial.println(F("   Motor zero abs: (read failed)"));
      }
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
          delay(1000); // Give user time to release
          currentState = STATE_SWING_UP;
          lastControlTime = millis();
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
          delay(1000); // Give user time to release
          currentState = STATE_BALANCE;
          lastControlTime = millis();
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
    if (inChar == '\r' || inChar == '\n') {
      continue;
    }
    if (inChar < ' ') {
      continue;
    }
    Serial.println(inChar);
    parseSerialCommand(String(inChar));
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
  Wire.setClock(100000); // 100kHz for noise robustness
  delay(100);
  as5600_pendulum.begin();

  // Initialize Software I2C for motor sensor
  motorWire.begin();
  Serial.println(F("Motor sensor bus ready on SDA=22, SCL=24."));
  
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

  // 2. Handle control logic at a fixed rate (100Hz)
  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_LOOP_MS) {
    lastControlTime = now;
    if (currentState == STATE_SWING_UP || currentState == STATE_BALANCE) {
      controlTick();
    }
  }

  // 3. Handle sensor monitoring display (if active)
  static unsigned long lastMonitorPrint = 0;
  if (currentState == STATE_MONITORING && (now - lastMonitorPrint >= 200)) {
    lastMonitorPrint = now;
    updateSensors();
    Serial.print(F("  "));
    Serial.print(pendulumAngle, 2);
    Serial.print(F("°   \t"));
    Serial.print(pendulumRaw);
    Serial.print(F("  \t"));
    Serial.print(motorAngle, 2);
    Serial.print(F("°   \t"));
    Serial.print(motorRaw);
    Serial.print(F("  \t"));
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
  
  // The old blocking serial menu is GONE.
  // All commands are handled by checkSerialInput() -> parseSerialCommand()
}
