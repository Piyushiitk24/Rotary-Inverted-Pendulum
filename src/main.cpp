#include <Arduino.h>
#include <Wire.h>
#include <SoftwareWire.h>
#include <AS5600.h>
#include <AccelStepper.h>
#include <math.h>

// Stepper Motor
#define STEP_PIN 5
#define DIR_PIN  6
#define EN_PIN   7
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Pendulum Sensor on Hardware I2C
AS5600 as5600_pendulum(&Wire);

// Motor position tracking via step counting (no motor sensor needed)
#define STEPS_PER_REVOLUTION 200
#define DEGREES_PER_STEP (360.0 / STEPS_PER_REVOLUTION)  // 1.8 degrees per step

// Motor settings
#define MOTOR_SPEED 8000  // Maximum speed cap for AccelStepper (CRITICAL: must be >= swingSpeed)
#define MOTOR_ACCEL 20000  // High acceleration for responsive control

// CALIBRATION DATA - STEP-BASED (much more reliable!)
long maxStepsLeft = -200;   // HARDCODED: 200 steps to the left
long maxStepsRight = 200;   // HARDCODED: 200 steps to the right
float pendulumZeroAngle = 0.0;
bool isCalibrated = true;   // Auto-calibrated with hardcoded limits

// Current readings
float pendulumAngle = 0.0;
float motorAngle = 0.0;  // Calculated from step count
long motorSteps = 0;     // Authoritative motor position

// Previous readings for change-rate limiting
float lastPendulumRaw = 0.0;
bool firstReading = true;

constexpr float MAX_SENSOR_DELTA_DEG = 120.0f;       // Max believable change (catches 180° bit flips)
constexpr float SAMPLE_CONSISTENCY_DEG = 3.0f;       // Required agreement between samples
constexpr uint8_t MAX_STABLE_READ_RETRIES = 5;       // Attempts to obtain consistent data
constexpr uint16_t AS5600_MIN_VALID = 20;            // Reject near-zero glitch values
constexpr uint16_t AS5600_MAX_VALID = 4075;          // Reject near-4095 glitch values
constexpr uint8_t LARGE_JUMP_CONFIRM_COUNT = 2;      // Consistent large readings needed to accept jump

bool pendulumMagnetHealthy = true;
float pendulumJumpCandidate = 0.0f;
uint8_t pendulumJumpCount = 0;

// ==================== CONTROL SYSTEM ====================

// Control modes
enum ControlMode {
  MODE_IDLE,
  MODE_SWING_UP,
  MODE_BALANCE
};
ControlMode controlMode = MODE_IDLE;

// PD Controller for Balance (Outer Loop - Pendulum Angle)
float Kp_alpha = 25.0;     // Proportional gain for pendulum angle
float Kd_alpha = 1.0;      // Derivative gain for pendulum velocity
float Ki_theta = 0.2;      // Integral gain to keep arm centered

// Inner Loop - Position Control (Motor Steps)
float Kp_theta = 200.0;    // Proportional gain for step position (MAXIMUM for fast response)

// Swing-up parameters
float swingAmplitude = 50.0;   // Steps to swing (will use calibrated limits)
float swingSpeed = 5000.0;     // Steps/sec during swing-up (MAXIMUM aggressive pumping)
float balanceThreshold = 25.0; // Degrees - switch to balance when |alpha| < this

// State variables
float lastPendulumAngle = 0.0;
float pendulumVelocity = 0.0;
float thetaIntegral = 0.0;      // Integral term for centering
unsigned long lastControlTime = 0;
float dt = 0.01;  // Control loop time step (10ms = 100Hz)

// Desired position from outer loop
long thetaDesiredSteps = 0;

// Low-pass filter for velocity
float alpha_filter = 0.7;  // Filter coefficient (0-1, higher = less filtering)

// Forward declarations for sensor reading functions
uint16_t readPendulumAngleMedian();
float normalizeAngleDelta(float delta);
bool isRawAngleValid(uint16_t raw);
bool readStablePendulumDegrees(float &deg);
void updateSensorReadings();
void updatePendulumVelocity();

// Motor position from step count
float getMotorAngleFromSteps() {
  return motorSteps * DEGREES_PER_STEP;
}

bool checkPendulumMagnet() {
  uint8_t status = as5600_pendulum.readStatus();
  // Bit 5 = MD (magnet detected), Bit 4 = ML (too weak), Bit 3 = MH (too strong)
  bool detected = (status & 0x20) != 0;  // MD bit
  bool tooWeak = (status & 0x10) != 0;   // ML bit  
  bool tooStrong = (status & 0x08) != 0; // MH bit
  return detected && !tooWeak && !tooStrong;
}

// Motor position tracking (step-based, no sensor needed)
void updateMotorPosition() {
  motorSteps = stepper.currentPosition();
  motorAngle = getMotorAngleFromSteps();
}

// ==================== CONTROL FUNCTIONS ====================

// Compute pendulum velocity (low-pass filtered derivative)
void updatePendulumVelocity() {
  // Handle wraparound: if angle jumps from +179° to -179°, that's only 2° not 358°
  float angleDelta = normalizeAngleDelta(pendulumAngle - lastPendulumAngle);
  float rawVelocity = angleDelta / dt;
  
  // Limit velocity to reasonable range (catch sensor glitches)
  if (abs(rawVelocity) > 1000.0) {
    rawVelocity = 0.0;  // Reject obviously wrong velocities
  }
  
  pendulumVelocity = alpha_filter * pendulumVelocity + (1.0 - alpha_filter) * rawVelocity;
  lastPendulumAngle = pendulumAngle;
}

// Constrain desired steps to calibrated limits
long constrainSteps(long steps) {
  if (steps < maxStepsLeft) return maxStepsLeft;
  if (steps > maxStepsRight) return maxStepsRight;
  return steps;
}

// Inner loop: Position control (makes motor go to desired step position)
void innerLoopControl() {
  // ⚠️ CRITICAL SAFETY: Enforce limits at lowest level
  thetaDesiredSteps = constrainSteps(thetaDesiredSteps);
  
  // ⚠️ SAFETY CHECK: If already at limit, don't try to go further
  if (motorSteps <= maxStepsLeft && thetaDesiredSteps < motorSteps) {
    // Already at left limit, can't go further left
    stepper.setSpeed(0.0);
    thetaDesiredSteps = motorSteps;  // Freeze at current position
    return;
  }
  if (motorSteps >= maxStepsRight && thetaDesiredSteps > motorSteps) {
    // Already at right limit, can't go further right
    stepper.setSpeed(0.0);
    thetaDesiredSteps = motorSteps;  // Freeze at current position
    return;
  }
  
  long stepError = thetaDesiredSteps - motorSteps;
  float desiredVelocity = Kp_theta * stepError;
  
  // Limit maximum speed for safety (much higher limit)
  if (desiredVelocity > 6000.0) desiredVelocity = 6000.0;
  if (desiredVelocity < -6000.0) desiredVelocity = -6000.0;
  
  // Set stepper speed (runSpeed() called in main loop)
  stepper.setSpeed(desiredVelocity);
}

// Outer loop: Balance control (PD on pendulum angle)
void balanceControl() {
  // PD control on pendulum angle (alpha)
  // Goal: keep pendulum at alpha = 0 (upright)
  float alphaError = 0.0 - pendulumAngle;  // Want zero angle
  
  // PD control output (desired motor position change)
  float thetaCorrection = Kp_alpha * alphaError + Kd_alpha * pendulumVelocity;
  
  // Integral term to slowly center the arm
  thetaIntegral += Ki_theta * motorSteps * dt;
  thetaCorrection -= thetaIntegral;
  
  // Convert correction to desired step position
  thetaDesiredSteps = motorSteps + (long)thetaCorrection;
  
  // Enforce limits
  thetaDesiredSteps = constrainSteps(thetaDesiredSteps);
  
  // Calculate and set velocity (inner loop)
  innerLoopControl();
}

// Swing-up control: Oscillate left-right
void swingUpControl() {
  static bool swingingRight = true;
  
  // ⚠️ CRITICAL SAFETY: Emergency stop if beyond limits
  if (motorSteps < maxStepsLeft || motorSteps > maxStepsRight) {
    Serial.println("\n[EMERGENCY] Position beyond limits! STOPPING!");
    controlMode = MODE_IDLE;
    stepper.setSpeed(0.0);
    thetaDesiredSteps = motorSteps;  // Freeze position
    digitalWrite(EN_PIN, HIGH);
    return;
  }
  
  // Check if we should switch to balance mode
  if (abs(pendulumAngle) < balanceThreshold) {
    Serial.println("\n[BALANCE] Pendulum near vertical - switching to balance mode!");
    controlMode = MODE_BALANCE;
    thetaDesiredSteps = motorSteps;  // Start from current position
    thetaIntegral = 0.0;
    return;
  }
  
  // Oscillate between calibrated limits (NEVER exceed these)
  if (swingingRight) {
    thetaDesiredSteps = maxStepsRight;  // Target: RIGHT limit
    // Only reverse when we ACTUALLY reach the limit (within 2 steps)
    if (motorSteps >= maxStepsRight - 2) {
      swingingRight = false;
    }
  } else {
    thetaDesiredSteps = maxStepsLeft;  // Target: LEFT limit
    // Only reverse when we ACTUALLY reach the limit (within 2 steps)
    if (motorSteps <= maxStepsLeft + 2) {
      swingingRight = true;
    }
  }
  
  // ⚠️ SAFETY: Constrain target before applying
  thetaDesiredSteps = constrainSteps(thetaDesiredSteps);
  
  // Use MAXIMUM speed for aggressive swing-up - NO SLOWING DOWN
  long stepError = thetaDesiredSteps - motorSteps;
  float desiredVelocity = swingSpeed * (stepError > 0 ? 1.0 : -1.0);
  
  // Set speed (runSpeed() called in main loop)
  stepper.setSpeed(desiredVelocity);
}

// Main control tick (called from loop at ~100Hz)
void controlTick() {
  unsigned long now = millis();
  dt = (now - lastControlTime) / 1000.0;  // Convert to seconds
  if (dt < 0.001) dt = 0.01;  // Prevent division by zero
  lastControlTime = now;
  
  // Update sensor readings
  updateSensorReadings();
  updatePendulumVelocity();
  
  // ⚠️ CRITICAL SAFETY: Global limit enforcement
  // If motor position is beyond limits, EMERGENCY STOP
  if (motorSteps < maxStepsLeft - 2 || motorSteps > maxStepsRight + 2) {
    Serial.println("\n[EMERGENCY STOP] Motor beyond safe limits!");
    Serial.print("  Current: ");
    Serial.print(motorSteps);
    Serial.print(" steps  Limits: [");
    Serial.print(maxStepsLeft);
    Serial.print(", ");
    Serial.print(maxStepsRight);
    Serial.println("]");
    controlMode = MODE_IDLE;
    stepper.setSpeed(0.0);
    thetaDesiredSteps = motorSteps;  // Freeze position
    digitalWrite(EN_PIN, HIGH);
    return;
  }
  
  // Execute control based on mode
  switch (controlMode) {
    case MODE_IDLE:
      // Do nothing
      break;
      
    case MODE_SWING_UP:
      swingUpControl();
      break;
      
    case MODE_BALANCE:
      balanceControl();
      
      // Check if pendulum fell - return to swing-up
      if (abs(pendulumAngle) > 60.0) {
        Serial.println("\n[SWING-UP] Pendulum fell - restarting swing-up!");
        controlMode = MODE_SWING_UP;
      }
      break;
  }
}

// Start control system
void startControl() {
  // Limits are hardcoded to ±200 steps, always ready
  Serial.println("\n[INFO] Using hardcoded limits: ±200 steps");
  
  // ⚠️ SAFETY: Check current position is within limits
  if (motorSteps < maxStepsLeft || motorSteps > maxStepsRight) {
    Serial.println("\n[ERROR] Current position outside limits!");
    Serial.print("  Position: ");
    Serial.print(motorSteps);
    Serial.print(" steps  Limits: [");
    Serial.print(maxStepsLeft);
    Serial.print(", ");
    Serial.print(maxStepsRight);
    Serial.println("]");
    Serial.println("  Run option 9 to return to zero first.");
    return;
  }
  
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║    STARTING SWING-UP & BALANCE CONTROL     ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println();
  Serial.println("Hardcoded Limits (±200 steps):");
  Serial.print("  Left limit:  ");
  Serial.print(maxStepsLeft);
  Serial.println(" steps");
  Serial.print("  Right limit: ");
  Serial.print(maxStepsRight);
  Serial.println(" steps");
  Serial.print("  Range:       ");
  Serial.print(maxStepsRight - maxStepsLeft);
  Serial.println(" steps\n");
  
  Serial.println("Control Parameters:");
  Serial.print("  Kp_alpha = ");
  Serial.println(Kp_alpha);
  Serial.print("  Kd_alpha = ");
  Serial.println(Kd_alpha);
  Serial.print("  Kp_theta = ");
  Serial.println(Kp_theta);
  Serial.print("  Balance threshold = ");
  Serial.print(balanceThreshold);
  Serial.println("°\n");
  
  // Enable motor
  digitalWrite(EN_PIN, LOW);
  
  // Initialize control state
  controlMode = MODE_SWING_UP;
  lastControlTime = millis();
  lastPendulumAngle = pendulumAngle;
  pendulumVelocity = 0.0;
  thetaIntegral = 0.0;
  thetaDesiredSteps = motorSteps;
  
  Serial.println("[SWING-UP] Starting oscillation...");
  Serial.println("Press 'S' to stop\n");
  
  // Control loop - CRITICAL: Tight loop for step generation, timed control updates
  unsigned long lastControlMicros = micros();
  unsigned long lastPrintMillis = millis();
  const unsigned long CONTROL_PERIOD_US = 10000;  // 10ms = 100Hz control rate
  
  while (true) {
    // Exit loop if control disabled
    if (controlMode == MODE_IDLE) {
      stepper.setSpeed(0.0);
      digitalWrite(EN_PIN, HIGH);
      Serial.println("\n[STOPPED] Control system disabled\n");
      delay(1000);
      return;
    }
    
    // CRITICAL: Run step generation as fast as possible
    stepper.runSpeed();
    
    // Update control calculations at fixed rate (10ms = 100Hz)
    unsigned long nowMicros = micros();
    if (nowMicros - lastControlMicros >= CONTROL_PERIOD_US) {
      controlTick();  // Updates thetaDesiredSteps and calls setSpeed()
      lastControlMicros = nowMicros;
    }
    
    // Print status every 200ms
    unsigned long nowMillis = millis();
    if (nowMillis - lastPrintMillis >= 200) {
      lastPrintMillis = nowMillis;
      
      Serial.print(controlMode == MODE_SWING_UP ? "[SWING] " : "[BALANCE] ");
      Serial.print("α=");
      Serial.print(pendulumAngle, 1);
      Serial.print("° (ω=");
      Serial.print(pendulumVelocity, 1);
      Serial.print("°/s)  θ=");
      Serial.print(motorSteps);
      Serial.print(" steps (target=");
      Serial.print(thetaDesiredSteps);
      Serial.println(")");
    }
    
    // Check for stop command
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'S' || c == 's') {
        controlMode = MODE_IDLE;  // Loop will exit on next iteration
      }
    }
  }
}

uint16_t readPendulumAngleMedian() {
  uint16_t r1 = as5600_pendulum.readAngle();
  delayMicroseconds(50);  // Reduced delay for faster control
  uint16_t r2 = as5600_pendulum.readAngle();
  delayMicroseconds(50);
  uint16_t r3 = as5600_pendulum.readAngle();
  
  // Sort and return middle value
  if (r1 > r2) { uint16_t t = r1; r1 = r2; r2 = t; }
  if (r2 > r3) { uint16_t t = r2; r2 = r3; r3 = t; }
  if (r1 > r2) { uint16_t t = r1; r1 = r2; r2 = t; }
  return r2;
}

float normalizeAngleDelta(float delta) {
  while (delta > 180.0f) delta -= 360.0f;
  while (delta < -180.0f) delta += 360.0f;
  return delta;
}

bool isRawAngleValid(uint16_t raw) {
  return raw >= AS5600_MIN_VALID && raw <= AS5600_MAX_VALID;
}

bool readStablePendulumDegrees(float &deg) {
  float lastDeg = 0.0f;
  bool hasLast = false;
  for (uint8_t attempt = 0; attempt < MAX_STABLE_READ_RETRIES; attempt++) {
    uint16_t raw = readPendulumAngleMedian();
    if (!isRawAngleValid(raw)) continue;
    float currentDeg = (raw * 360.0f) / 4096.0f;
    if (hasLast) {
      float delta = normalizeAngleDelta(currentDeg - lastDeg);
      float deltaAbs = delta >= 0 ? delta : -delta;
      if (deltaAbs <= SAMPLE_CONSISTENCY_DEG) {
        deg = currentDeg;
        return true;
      }
    }
    lastDeg = currentDeg;
    hasLast = true;
    delayMicroseconds(150);
  }
  if (hasLast) {
    deg = lastDeg;
    return true;
  }
  return false;
}

// Motor position from step count (no sensor needed)

void updateSensorReadings() {
  // Check pendulum magnet health
  bool pendulumStatus = checkPendulumMagnet();
  if (pendulumStatus != pendulumMagnetHealthy) {
    if (!pendulumStatus) {
      Serial.println("[WARN] Pendulum magnet out of range - holding last value");
      pendulumJumpCount = 0;
    } else {
      Serial.println("[INFO] Pendulum magnet restored");
    }
    pendulumMagnetHealthy = pendulumStatus;
  }
  
  // Read pendulum angle with filtering
  float pendulum_deg = lastPendulumRaw;
  bool pendulumUpdated = false;
  if (pendulumMagnetHealthy) {
    float candidate;
    if (readStablePendulumDegrees(candidate)) {
      if (!firstReading) {
        float delta = normalizeAngleDelta(candidate - lastPendulumRaw);
        float deltaAbs = fabsf(delta);
        if (deltaAbs > MAX_SENSOR_DELTA_DEG) {
          Serial.print("[WARN] Pendulum jump rejected: ");
          Serial.print(delta, 1);
          Serial.println("°");
          if (pendulumJumpCount > 0 && fabsf(candidate - pendulumJumpCandidate) <= SAMPLE_CONSISTENCY_DEG) {
            pendulumJumpCount++;
          } else {
            pendulumJumpCandidate = candidate;
            pendulumJumpCount = 1;
          }
          if (pendulumJumpCount >= LARGE_JUMP_CONFIRM_COUNT) {
            Serial.println("[WARN] Pendulum jump accepted after stability hold");
            pendulumUpdated = true;
            pendulum_deg = candidate;
            pendulumJumpCount = 0;
          }
        } else {
          pendulumUpdated = true;
          pendulum_deg = candidate;
          pendulumJumpCount = 0;
        }
      } else {
        pendulumUpdated = true;
        pendulum_deg = candidate;
        pendulumJumpCount = 0;
      }
    }
  }
  if (pendulumUpdated) {
    lastPendulumRaw = pendulum_deg;
  } else {
    pendulum_deg = lastPendulumRaw;
  }
  
  firstReading = false;
  
  // Calculate pendulum angle relative to zero
  pendulumAngle = pendulum_deg - pendulumZeroAngle;
  
  // Normalize pendulum angle to -180 to +180
  while (pendulumAngle > 180.0) pendulumAngle -= 360.0;
  while (pendulumAngle < -180.0) pendulumAngle += 360.0;
  
  // Update motor position from step counter (100% accurate)
  updateMotorPosition();
}

void printSensorReadings() {
  updateSensorReadings();
  Serial.print("Pendulum: ");
  Serial.print(pendulumAngle, 2);
  Serial.print("° \t Motor Sensor: ");
  Serial.print(motorAngle, 2);
  Serial.print("° \t Steps: ");
  Serial.println(motorSteps);
}

void printCalibrationData() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║         SYSTEM STATUS                      ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.print("Pendulum Zero: ");
  Serial.print(pendulumZeroAngle, 2);
  Serial.println("°");
  Serial.println("\n--- MOTOR LIMITS (HARDCODED) ---");
  Serial.println("LEFT Limit:  -200 steps");
  Serial.println("RIGHT Limit: +200 steps");
  Serial.println("Total Range: 400 steps (144°)");
  Serial.println("\n✓ System ready! Just set zero position.");
  Serial.println("════════════════════════════════════════════\n");
}

void printMainMenu() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║   ROTARY INVERTED PENDULUM CONTROL         ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\n[Hardcoded Limits: ±200 steps]");
  Serial.println("\n--- SETUP ---");
  Serial.println("1 - Set Zero (center arm, pendulum up, press Z)");
  Serial.println("2 - Live Sensor Test");
  Serial.println("5 - Show System Info");
  Serial.println("\n--- CONTROL ---");
  Serial.println("S - START Swing-Up & Balance Control");
  Serial.println("\n--- MANUAL TESTING ---");
  Serial.println("6 - Test Full Range (move between limits)");
  Serial.println("7 - Jog Left (CW, -1 step)");
  Serial.println("8 - Jog Right (CCW, +1 step)");
  Serial.println("9 - Return to Zero");
  Serial.println("\n--- QUICK LIMITS ---");
  Serial.println("L - Set limits to ±10 steps");
  Serial.println("W - Set limits to ±15 steps");
  Serial.println("M - Set limits to ±20 steps");
  Serial.println("\n--- DIAGNOSTICS ---");
  Serial.println("D - EMI Test");
  Serial.println("N - Motor Power Noise Test");
  Serial.println("R - RAW Diagnostic Mode");
  Serial.println("0 - I2C Bus Scan");
  Serial.println("C - Clear calibration");
  Serial.println("════════════════════════════════════════════");
  Serial.print("Enter choice: ");
}

void setZeroPosition() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║        STEP 1: SET ZERO POSITION           ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\nINSTRUCTIONS:");
  Serial.println("1. Position the motor arm at CENTER");
  Serial.println("2. Position the pendulum STRAIGHT UP (0°)");
  Serial.println("3. Press 'Z' to set zero");
  Serial.println("4. Press 'X' to cancel\n");
  
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'z' || c == 'Z') {
        stepper.setCurrentPosition(0);
        uint16_t pendulum_raw = as5600_pendulum.readAngle();
        pendulumZeroAngle = (pendulum_raw * 360.0) / 4096.0;
        
        Serial.println("\n✓ ZERO POSITION SET!");
        Serial.print("  Pendulum zero: ");
        Serial.print(pendulumZeroAngle, 2);
        Serial.println("°");
        Serial.println("  Stepper position: 0 steps");
        delay(2000);
        return;
      } else if (c == 'x' || c == 'X') {
        Serial.println("\nCancelled.");
        delay(1000);
        return;
      }
    }
    delay(50);
  }
}

void rawDiagnosticMode() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║       RAW SENSOR DIAGNOSTIC MODE           ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\nShows RAW 12-bit values and filtered values.");
  Serial.println("Use this to detect bit-flip patterns.\n");
  Serial.println("Press 'S' to stop.\n");
  
  digitalWrite(EN_PIN, HIGH);  // Disable motor
  delay(100);
  
  Serial.println("───────────────────────────────────────────────────────────────");
  Serial.println("Pend_RAW  Pend_Filt  Motor_RAW  Motor_Filt  Pend(°)  Motor(°)");
  Serial.println("───────────────────────────────────────────────────────────────");
  
  unsigned long lastPrint = millis();
  
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's' || c == 'S' || c == 'x' || c == 'X') {
        Serial.println("───────────────────────────────────────────────────────────────");
        Serial.println("\nStopped.\n");
        digitalWrite(EN_PIN, LOW);
        delay(1000);
        return;
      }
    }
    
    if (millis() - lastPrint >= 200) {
      lastPrint = millis();
      
      // Single reads (corrupted by EMI)
      uint16_t pend_single = as5600_pendulum.readAngle();
      
      // Median-filtered reads (robust)
      uint16_t pend_median = readPendulumAngleMedian();
      
      // Convert to degrees
      float pend_deg = (pend_median * 360.0) / 4096.0;
      
      // Highlight differences
      bool pend_diff = abs((int)pend_single - (int)pend_median) > 100;
      
      if (pend_diff) Serial.print("⚠ ");
      else Serial.print("  ");
      
      Serial.print("Pendulum Raw: ");
      Serial.print(pend_single);
      Serial.print(" → ");
      Serial.print(pend_median);
      Serial.print(" (");
      Serial.print(pend_deg, 1);
      Serial.print("°)  |  Motor: ");
      Serial.print(motorSteps);
      Serial.print(" steps (");
      Serial.print(motorAngle, 1);
      Serial.println("°)");
    }
  }
}

void liveSensorTest() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║         LIVE SENSOR READINGS               ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\n⚠ Testing with MOTOR DRIVER DISABLED");
  Serial.println("This tests if motor driver causes interference.\n");
  Serial.println("Rotate pendulum and move motor arm.");
  Serial.println("Press 'S' to stop.\n");
  
  // DISABLE motor driver to test for EMI
  digitalWrite(EN_PIN, HIGH);
  delay(100);
  
  Serial.println("────────────────────────────────────────────");
  Serial.println("Pendulum(°) \tMotor(°) \tΔPend \tΔMotor");
  Serial.println("────────────────────────────────────────────");
  
  unsigned long lastPrint = millis();
  float lastPendulum = 0;
  float lastMotor = 0;
  int stableCount = 0;
  int jumpCount = 0;
  
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's' || c == 'S' || c == 'x' || c == 'X') {
        Serial.println("────────────────────────────────────────────");
        Serial.print("Stable readings: ");
        Serial.println(stableCount);
        Serial.print("Large jumps (>10°): ");
        Serial.println(jumpCount);
        Serial.println("\nStopped.\n");
        
        // Re-enable motor driver
        digitalWrite(EN_PIN, LOW);
        delay(1000);
        return;
      }
    }
    
    if (millis() - lastPrint >= 100) {
      lastPrint = millis();
      
      updateSensorReadings();
      
      // Calculate deltas
      float deltaPend = abs(pendulumAngle - lastPendulum);
      float deltaMotor = abs(motorAngle - lastMotor);
      
      // Track stability
      if (deltaPend < 2.0 && deltaMotor < 2.0) {
        stableCount++;
      }
      if (deltaPend > 10.0 || deltaMotor > 10.0) {
        jumpCount++;
        Serial.print("⚠ ");
      } else {
        Serial.print("  ");
      }
      
      Serial.print(pendulumAngle, 2);
      Serial.print("° \t");
      Serial.print(motorAngle, 2);
      Serial.print("° \t");
      Serial.print(deltaPend, 1);
      Serial.print("° \t");
      Serial.print(deltaMotor, 1);
      Serial.println("°");
      
      lastPendulum = pendulumAngle;
      lastMotor = motorAngle;
    }
  }
}

void motorPowerNoiseTest() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║    MOTOR POWER NOISE TEST (Capacitor Test) ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\n✓ Testing with MOTOR DRIVER ENABLED & MOTOR POWER ON");
  Serial.println("This tests capacitor effectiveness against power supply noise.\n");
  Serial.println("⚠ Motor will NOT move - just powered and ready.");
  Serial.println("Keep pendulum and motor arm STATIONARY.");
  Serial.println("Press 'S' to stop.\n");
  
  // ENABLE motor driver (motor power ON but not moving)
  digitalWrite(EN_PIN, LOW);
  delay(100);
  
  Serial.println("────────────────────────────────────────────");
  Serial.println("Pendulum(°) \tMotor(°) \tΔPend \tΔMotor");
  Serial.println("────────────────────────────────────────────");
  
  unsigned long lastPrint = millis();
  float lastPendulum = 0;
  float lastMotor = 0;
  int stableCount = 0;
  int jumpCount = 0;
  
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's' || c == 'S' || c == 'x' || c == 'X') {
        Serial.println("────────────────────────────────────────────");
        Serial.print("Stable readings: ");
        Serial.println(stableCount);
        Serial.print("Large jumps (>10°): ");
        Serial.println(jumpCount);
        
        float successRate = (stableCount * 100.0) / (stableCount + jumpCount);
        Serial.print("Success rate: ");
        Serial.print(successRate, 1);
        Serial.println("%");
        
        if (successRate > 90) {
          Serial.println("\n✓✓✓ EXCELLENT! Capacitors working well!");
        } else if (successRate > 70) {
          Serial.println("\n✓ GOOD. Minor noise remains.");
        } else {
          Serial.println("\n⚠ POOR. Need more filtering or check capacitor connections.");
        }
        
        Serial.println("\nStopped.\n");
        return;
      }
    }
    
    if (millis() - lastPrint >= 100) {
      lastPrint = millis();
      
      updateSensorReadings();
      
      // Calculate deltas
      float deltaPend = abs(pendulumAngle - lastPendulum);
      float deltaMotor = abs(motorAngle - lastMotor);
      
      // Track stability
      if (deltaPend < 2.0 && deltaMotor < 2.0) {
        stableCount++;
      }
      if (deltaPend > 10.0 || deltaMotor > 10.0) {
        jumpCount++;
        Serial.print("⚠ ");
      } else {
        Serial.print("  ");
      }
      
      Serial.print(pendulumAngle, 2);
      Serial.print("° \t");
      Serial.print(motorAngle, 2);
      Serial.print("° \t");
      Serial.print(deltaPend, 1);
      Serial.print("° \t");
      Serial.print(deltaMotor, 1);
      Serial.println("°");
      
      lastPendulum = pendulumAngle;
      lastMotor = motorAngle;
    }
  }
}

void manualStep(int direction) {
  // ⚠️ CRITICAL SAFETY: Check limits before allowing movement
  long nextPosition = stepper.currentPosition() + direction;
  
  if (isCalibrated) {
    if (nextPosition < maxStepsLeft) {
      Serial.println("\n[BLOCKED] Would exceed LEFT limit!");
      Serial.print("  Limit: ");
      Serial.print(maxStepsLeft);
      Serial.print(" steps, Attempted: ");
      Serial.println(nextPosition);
      return;
    }
    if (nextPosition > maxStepsRight) {
      Serial.println("\n[BLOCKED] Would exceed RIGHT limit!");
      Serial.print("  Limit: ");
      Serial.print(maxStepsRight);
      Serial.print(" steps, Attempted: ");
      Serial.println(nextPosition);
      return;
    }
  }
  
  // Force enable the motor
  digitalWrite(EN_PIN, LOW);
  delay(1);
  
  // Set direction (LOW = CW/left, HIGH = CCW/right)
  digitalWrite(DIR_PIN, direction > 0 ? HIGH : LOW);
  delayMicroseconds(5);
  
  // Generate ONE step pulse
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(2);  // 2us pulse width
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(2);
  
  // Update AccelStepper's internal position to match
  stepper.setCurrentPosition(nextPosition);

  // Allow driver current spike to settle before next sensor read
  delayMicroseconds(500);
}

void setLeftLimit() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║       SET LEFT LIMIT (STEP-BASED)          ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\nINSTRUCTIONS:");
  Serial.println("Use keys to jog motor to LEFT edge:");
  Serial.println("  'A' = Move left 1 step");
  Serial.println("  'D' = Move right 1 step");
  Serial.println("  'L' = Set current position as LEFT limit");
  Serial.println("  'X' = Cancel\n");
  
  // Check enable pin state
  Serial.print("Enable pin: ");
  Serial.println(digitalRead(EN_PIN) == LOW ? "ENABLED" : "DISABLED!");
  
  updateSensorReadings();
  Serial.print("Starting position: ");
  Serial.print(motorSteps);
  Serial.println(" steps\n");
  
  while (true) {
    // Check serial immediately (no delay)
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'a' || c == 'A') {
        Serial.println("Stepping LEFT (CW)...");
        manualStep(-1);  // Move left
        delay(50);  // Wait for motor to settle
        
        updateSensorReadings();
        Serial.print("← Left | Step: ");
        Serial.print(motorSteps);
        Serial.print(" | Pend: ");
        Serial.print(pendulumAngle, 1);
        Serial.print("° | Motor: ");
        Serial.print(motorAngle, 1);
        Serial.println("°");
      } else if (c == 'd' || c == 'D') {
        Serial.println("Stepping RIGHT (CCW)...");
        manualStep(+1);  // Move right
        delay(50);  // Wait for motor to settle
        
        updateSensorReadings();
        Serial.print("→ Right | Step: ");
        Serial.print(motorSteps);
        Serial.print(" | Pend: ");
        Serial.print(pendulumAngle, 1);
        Serial.print("° | Motor: ");
        Serial.print(motorAngle, 1);
        Serial.println("°");
      } else if (c == 'l' || c == 'L') {
        maxStepsLeft = stepper.currentPosition();
        Serial.println("\n✓ LEFT LIMIT SET!");
        Serial.print("  Position: ");
        Serial.print(maxStepsLeft);
        Serial.println(" steps");
        delay(2000);
        return;
      } else if (c == 'x' || c == 'X') {
        Serial.println("\nCancelled.");
        delay(1000);
        return;
      }
    }
    delay(10); // Small delay to prevent CPU spinning
  }
}

void setRightLimit() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║      SET RIGHT LIMIT (STEP-BASED)          ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\nINSTRUCTIONS:");
  Serial.println("From LEFT edge, jog motor to RIGHT edge:");
  Serial.println("  'A' = Move left 1 step");
  Serial.println("  'D' = Move right 1 step");
  Serial.println("  'R' = Set current position as RIGHT limit");
  Serial.println("  'X' = Cancel\n");
  Serial.println("NOTE: Start from left edge position, jog right");
  
  // Check enable pin state
  Serial.print("Enable pin: ");
  Serial.println(digitalRead(EN_PIN) == LOW ? "ENABLED" : "DISABLED!");
  
  updateSensorReadings();
  Serial.print("Starting position: ");
  Serial.print(motorSteps);
  Serial.println(" steps\n");
  
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'a' || c == 'A') {
        Serial.println("Stepping LEFT (CW)...");
        manualStep(-1);  // Move left
        delay(50);
        
        updateSensorReadings();
        Serial.print("← Left | Step: ");
        Serial.print(motorSteps);
        Serial.print(" | Pend: ");
        Serial.print(pendulumAngle, 1);
        Serial.print("° | Motor: ");
        Serial.print(motorAngle, 1);
        Serial.println("°");
      } else if (c == 'd' || c == 'D') {
        Serial.println("Stepping RIGHT (CCW)...");
        manualStep(+1);  // Move right
        delay(50);
        
        updateSensorReadings();
        Serial.print("→ Right | Step: ");
        Serial.print(motorSteps);
        Serial.print(" | Pend: ");
        Serial.print(pendulumAngle, 1);
        Serial.print("° | Motor: ");
        Serial.print(motorAngle, 1);
        Serial.println("°");
      } else if (c == 'r' || c == 'R') {
        maxStepsRight = stepper.currentPosition();
        isCalibrated = true;
        Serial.println("\n✓ RIGHT LIMIT SET!");
        Serial.print("  Position: ");
        Serial.print(maxStepsRight);
        Serial.println(" steps");
        Serial.println("\n✓ CALIBRATION COMPLETE!");
        printCalibrationData();
        delay(3000);
        return;
      } else if (c == 'x' || c == 'X') {
        Serial.println("\nCancelled.");
        delay(1000);
        return;
      }
    }
    delay(10);
  }
}

void testFullRange() {
  if (!isCalibrated) {
    Serial.println("\nERROR: Please calibrate first!");
    delay(2000);
    return;
  }
  
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║         TEST FULL RANGE                    ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\n⚠ Motor will move between calibrated limits!");
  Serial.println("Press 'S' at any time to STOP.\n");
  delay(2000);
  
  Serial.println("Moving to LEFT limit...");
  stepper.moveTo(maxStepsLeft);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's' || c == 'S') {
        stepper.stop();
        digitalWrite(EN_PIN, HIGH); // Disable motor
        Serial.println("\n⚠ EMERGENCY STOP!");
        delay(2000);
        digitalWrite(EN_PIN, LOW); // Re-enable
        return;
      }
    }
  }
  Serial.print("At LEFT: ");
  Serial.print(stepper.currentPosition());
  Serial.println(" steps");
  delay(1000);
  
  Serial.println("Moving to RIGHT limit...");
  stepper.moveTo(maxStepsRight);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's' || c == 'S') {
        stepper.stop();
        digitalWrite(EN_PIN, HIGH); // Disable motor
        Serial.println("\n⚠ EMERGENCY STOP!");
        delay(2000);
        digitalWrite(EN_PIN, LOW); // Re-enable
        return;
      }
    }
  }
  Serial.print("At RIGHT: ");
  Serial.print(stepper.currentPosition());
  Serial.println(" steps");
  delay(1000);
  
  Serial.println("Returning to center...");
  stepper.moveTo(0);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's' || c == 'S') {
        stepper.stop();
        digitalWrite(EN_PIN, HIGH); // Disable motor
        Serial.println("\n⚠ EMERGENCY STOP!");
        delay(2000);
        digitalWrite(EN_PIN, LOW); // Re-enable
        return;
      }
    }
  }
  Serial.println("✓ Test complete!\n");
  delay(1000);
}

void jogLeft() {
  stepper.move(-1);
  stepper.runToPosition();
  delayMicroseconds(500);
  updateSensorReadings();
  Serial.print("← Step ");
  Serial.print(motorSteps);
  Serial.print(" | Pend: ");
  Serial.print(pendulumAngle, 1);
  Serial.print("° | Motor: ");
  Serial.print(motorAngle, 1);
  Serial.println("°");
  delay(300);
}

void jogRight() {
  stepper.move(1);
  stepper.runToPosition();
  delayMicroseconds(500);
  updateSensorReadings();
  Serial.print("→ Step ");
  Serial.print(motorSteps);
  Serial.print(" | Pend: ");
  Serial.print(pendulumAngle, 1);
  Serial.print("° | Motor: ");
  Serial.print(motorAngle, 1);
  Serial.println("°");
  delay(300);
}

void returnToZero() {
  Serial.println("Returning to zero...");
  stepper.moveTo(0);
  stepper.runToPosition();
  delayMicroseconds(500);
  Serial.println("✓ At center");
  printSensorReadings();
  delay(500);
}

void setQuickLimits(long steps) {
  maxStepsLeft = -steps;
  maxStepsRight = steps;
  isCalibrated = true;
  Serial.print("\n✓ Limits set to ±");
  Serial.print(steps);
  Serial.println(" steps");
  printCalibrationData();
  delay(2000);
}

void runI2CScan() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║           I2C BUS SCANNER                  ║");
  Serial.println("╚════════════════════════════════════════════╝");
  
  Serial.println("\n--- Hardware I2C (pins 20/21) ---");
  byte hw_count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      if (addr == 0x36) Serial.print(" (AS5600)");
      Serial.println();
      hw_count++;
    }
  }
  if (hw_count == 0) Serial.println("  None");
  
  Serial.println("\n[INFO] Motor position tracking via step counter (no sensor)");
  Serial.println();
  delay(2000);
}

void clearCalibration() {
  Serial.println("\nClearing calibration...");
  pendulumZeroAngle = 0.0;
  maxStepsLeft = -10;
  maxStepsRight = 10;
  isCalibrated = false;
  Serial.println("✓ Cleared\n");
  delay(1000);
}

void setup() {
  // EMERGENCY: Disable motor IMMEDIATELY before anything else!
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);  // HIGH = DISABLED
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);  // Stop any step signals
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);   // Set direction low
  delay(100);  // Let signals stabilize
  
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n╔════════════════════════════════════════════╗");
  Serial.println("║   ROTARY INVERTED PENDULUM - V2.0          ║");
  Serial.println("║   Step-Based Calibration (Reliable!)       ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
  
  Serial.println("⚠⚠⚠ SAFETY WARNING ⚠⚠⚠");
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║  1. CHECK MOTOR ARM IS FREE TO MOVE        ║");
  Serial.println("║  2. ENSURE WIRES HAVE SLACK                ║");
  Serial.println("║  3. BE READY TO DISCONNECT POWER           ║");
  Serial.println("║  4. LIMITS: ±280 steps = ±504° max         ║");
  Serial.println("║  5. Motor driver DISABLED during setup     ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\n⚠ IMPORTANT: If motor moves during power-on,");
  Serial.println("  immediately disconnect power! This indicates");
  Serial.println("  a hardware issue with the driver board.");
  Serial.println("\nPress ANY KEY to continue...\n");
  
  while (!Serial.available()) {
    delay(100);
  }
  while (Serial.available()) Serial.read(); // Clear buffer
  
  Serial.println("\nInitializing...\n");
  
  // Motor already disabled at start of setup()
  // Configure stepper settings (motor still disabled)
  stepper.setMaxSpeed(MOTOR_SPEED);
  stepper.setAcceleration(MOTOR_ACCEL);
  stepper.setCurrentPosition(0);
  Serial.println("[OK] Motor Driver (disabled for safety)");
  
  // Initialize Hardware I2C with longer timeout for capacitors
  Serial.print("Initializing Hardware I2C... ");
  Wire.begin();
  Wire.setClock(100000);  // REDUCED to 100kHz for noise immunity
  Wire.setWireTimeout(5000, true);  // 5ms timeout, reset on timeout
  delay(200);  // Give capacitors time to stabilize
  Serial.println("done (100kHz for EMI immunity)");
  
  // Test I2C bus first with raw transmission
  Serial.print("Testing I2C bus for device 0x36... ");
  Wire.beginTransmission(0x36);
  uint8_t error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("found!");
  } else {
    Serial.print("error ");
    Serial.println(error);
    Serial.println("  → Device not responding on Hardware I2C");
    Serial.println("  → Check pins 20 (SDA), 21 (SCL)");
    Serial.println("  → Check 5V, GND, and pull-up resistors");
  }
  
  // Test pendulum sensor with AS5600 library
  Serial.print("Initializing AS5600 library... ");
  as5600_pendulum.begin();
  
  // Check magnet field strength
  Serial.print("Checking pendulum magnet field... ");
  if (checkPendulumMagnet()) {
    Serial.println("OK");
  } else {
    Serial.println("WARNING: Magnet too weak/strong or not detected!");
    Serial.println("  → Adjust magnet distance (should be 2-3mm)");
  }
  Serial.println("done");
  
  Serial.print("Testing magnet detection... ");
  // Try reading angle directly instead of detectMagnet (which might hang)
  uint16_t test_angle = as5600_pendulum.readAngle();
  if (test_angle > 0 && test_angle < 4096) {
    Serial.print("[OK] angle=");
    Serial.println(test_angle);
  } else {
    Serial.println("[ERROR]");
    Serial.println("  → Check magnet distance (2-3mm from chip)");
    Serial.println("  → Magnet may be too far or missing");
  }
  
  // Motor position tracking via step counter
  Serial.println("[INFO] Motor sensor: Using step counter for position");
  Serial.println("  → 100% accurate position tracking");
  Serial.println("  → No sensor needed for motor shaft");
  Serial.print("  → Resolution: ");
  Serial.print(DEGREES_PER_STEP, 2);
  Serial.println("° per step");
  
  Serial.println("\n════════════════════════════════════════════");
  delay(1000);
  
  printMainMenu();
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') return;
    
    switch (c) {
      case '1': setZeroPosition(); printMainMenu(); break;
      case '2': liveSensorTest(); printMainMenu(); break;
      case '3': setLeftLimit(); printMainMenu(); break;
      case '4': setRightLimit(); printMainMenu(); break;
      case '5': printCalibrationData(); delay(2000); printMainMenu(); break;
      case 's': case 'S': startControl(); printMainMenu(); break;  // NEW: Start control
      case '6': testFullRange(); printMainMenu(); break;
      case '7': jogLeft(); break;
      case '8': jogRight(); break;
      case '9': returnToZero(); printMainMenu(); break;
      case 'l': case 'L': setQuickLimits(10); printMainMenu(); break;
      case 'w': case 'W': setQuickLimits(15); printMainMenu(); break;
      case 'm': case 'M': setQuickLimits(20); printMainMenu(); break;
      case 'd': case 'D': liveSensorTest(); printMainMenu(); break;
      case 'n': case 'N': motorPowerNoiseTest(); printMainMenu(); break;
      case 'r': case 'R': rawDiagnosticMode(); printMainMenu(); break;
      case '0': runI2CScan(); printMainMenu(); break;
      case 'c': case 'C': clearCalibration(); printMainMenu(); break;
      default: 
        Serial.println("Invalid."); 
        delay(300); 
        printMainMenu(); 
        break;
    }
  }
}
