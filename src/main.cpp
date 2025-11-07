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

// Motor Sensor on Software I2C
#define MOTOR_SDA_PIN 22
#define MOTOR_SCL_PIN 24
#define AS5600_ADDRESS 0x36
#define AS5600_RAW_ANGLE_REG 0x0C
#define AS5600_STATUS_REG 0x0B
SoftwareWire motorWire(MOTOR_SDA_PIN, MOTOR_SCL_PIN);

// Motor settings
#define MOTOR_SPEED 1000
#define MOTOR_ACCEL 2000

// CALIBRATION DATA - STEP-BASED (much more reliable!)
long maxStepsLeft = -10;   // Default: 10 steps to the left
long maxStepsRight = 10;   // Default: 10 steps to the right
float pendulumZeroAngle = 0.0;
bool isCalibrated = false;

// Current readings
float pendulumAngle = 0.0;
float motorAngle = 0.0;
long motorSteps = 0;

// Previous readings for change-rate limiting
float lastPendulumRaw = 0.0;
float lastMotorRaw = 0.0;
bool firstReading = true;

constexpr float MAX_SENSOR_DELTA_DEG = 120.0f;       // Max believable change (catches 180° bit flips)
constexpr float SAMPLE_CONSISTENCY_DEG = 3.0f;       // Required agreement between samples
constexpr uint8_t MAX_STABLE_READ_RETRIES = 5;       // Attempts to obtain consistent data
constexpr uint16_t AS5600_MIN_VALID = 20;            // Reject near-zero glitch values
constexpr uint16_t AS5600_MAX_VALID = 4075;          // Reject near-4095 glitch values
constexpr uint8_t LARGE_JUMP_CONFIRM_COUNT = 2;      // Consistent large readings needed to accept jump

bool pendulumMagnetHealthy = true;
bool motorMagnetHealthy = true;
float pendulumJumpCandidate = 0.0f;
uint8_t pendulumJumpCount = 0;
float motorJumpCandidate = 0.0f;
uint8_t motorJumpCount = 0;

// Sensor I2C helpers
uint16_t readAS5600Angle(SoftwareWire &wire) {
  wire.beginTransmission(AS5600_ADDRESS);
  wire.write(AS5600_RAW_ANGLE_REG);
  if (wire.endTransmission() != 0) return 0;
  wire.requestFrom(AS5600_ADDRESS, 2);
  if (wire.available() >= 2) {
    uint8_t high = wire.read();
    uint8_t low = wire.read();
    return (high << 8) | low;
  }
  return 0;
}

bool detectAS5600Magnet(SoftwareWire &wire) {
  wire.beginTransmission(AS5600_ADDRESS);
  wire.write(AS5600_STATUS_REG);
  if (wire.endTransmission() != 0) return false;
  wire.requestFrom(AS5600_ADDRESS, 1);
  if (wire.available()) {
    uint8_t status = wire.read();
    return (status & 0x20) != 0;
  }
  return false;
}

bool checkPendulumMagnet() {
  uint8_t status = as5600_pendulum.readStatus();
  // Bit 5 = MD (magnet detected), Bit 4 = ML (too weak), Bit 3 = MH (too strong)
  bool detected = (status & 0x20) != 0;  // MD bit
  bool tooWeak = (status & 0x10) != 0;   // ML bit  
  bool tooStrong = (status & 0x08) != 0; // MH bit
  return detected && !tooWeak && !tooStrong;
}

// Median of 3 readings to reject single-bit flips
uint16_t readAS5600AngleMedian(SoftwareWire &wire) {
  uint16_t r1 = readAS5600Angle(wire);
  delayMicroseconds(100);
  uint16_t r2 = readAS5600Angle(wire);
  delayMicroseconds(100);
  uint16_t r3 = readAS5600Angle(wire);
  
  // Sort and return middle value
  if (r1 > r2) { uint16_t t = r1; r1 = r2; r2 = t; }
  if (r2 > r3) { uint16_t t = r2; r2 = r3; r3 = t; }
  if (r1 > r2) { uint16_t t = r1; r1 = r2; r2 = t; }
  return r2;
}

uint16_t readPendulumAngleMedian() {
  uint16_t r1 = as5600_pendulum.readAngle();
  delayMicroseconds(100);
  uint16_t r2 = as5600_pendulum.readAngle();
  delayMicroseconds(100);
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

bool readStableMotorDegrees(float &deg) {
  float lastDeg = 0.0f;
  bool hasLast = false;
  for (uint8_t attempt = 0; attempt < MAX_STABLE_READ_RETRIES; attempt++) {
    uint16_t raw = readAS5600AngleMedian(motorWire);
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

void updateSensorReadings() {
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
  
  bool motorStatus = detectAS5600Magnet(motorWire);
  if (motorStatus != motorMagnetHealthy) {
    if (!motorStatus) {
      Serial.println("[WARN] Motor magnet not detected - holding last value");
      motorJumpCount = 0;
    } else {
      Serial.println("[INFO] Motor magnet restored");
    }
    motorMagnetHealthy = motorStatus;
  }
  
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
  
  float motor_deg = lastMotorRaw;
  bool motorUpdated = false;
  if (motorMagnetHealthy) {
    float candidate;
    if (readStableMotorDegrees(candidate)) {
      if (!firstReading) {
        float delta = normalizeAngleDelta(candidate - lastMotorRaw);
        float deltaAbs = fabsf(delta);
        if (deltaAbs > MAX_SENSOR_DELTA_DEG) {
          Serial.print("[WARN] Motor jump rejected: ");
          Serial.print(delta, 1);
          Serial.println("°");
          if (motorJumpCount > 0 && fabsf(candidate - motorJumpCandidate) <= SAMPLE_CONSISTENCY_DEG) {
            motorJumpCount++;
          } else {
            motorJumpCandidate = candidate;
            motorJumpCount = 1;
          }
          if (motorJumpCount >= LARGE_JUMP_CONFIRM_COUNT) {
            Serial.println("[WARN] Motor jump accepted after stability hold");
            motorUpdated = true;
            motor_deg = candidate;
            motorJumpCount = 0;
          }
        } else {
          motorUpdated = true;
          motor_deg = candidate;
          motorJumpCount = 0;
        }
      } else {
        motorUpdated = true;
        motor_deg = candidate;
        motorJumpCount = 0;
      }
    }
  }
  if (motorUpdated) {
    lastMotorRaw = motor_deg;
  } else {
    motor_deg = lastMotorRaw;
  }
  motorAngle = motor_deg;
  
  firstReading = false;
  
  // Calculate pendulum angle relative to zero
  pendulumAngle = pendulum_deg - pendulumZeroAngle;
  
  // Normalize pendulum angle to -180 to +180
  while (pendulumAngle > 180.0) pendulumAngle -= 360.0;
  while (pendulumAngle < -180.0) pendulumAngle += 360.0;
  
  // Get stepper position
  motorSteps = stepper.currentPosition();
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
  Serial.println("║         CALIBRATION DATA                   ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.print("Pendulum Zero: ");
  Serial.print(pendulumZeroAngle, 2);
  Serial.println("°");
  Serial.println("\n--- MOTOR STEP LIMITS ---");
  Serial.print("LEFT Limit:  ");
  Serial.print(maxStepsLeft);
  Serial.println(" steps");
  Serial.print("RIGHT Limit: ");
  Serial.print(maxStepsRight);
  Serial.println(" steps");
  Serial.print("Total Range: ");
  Serial.print(maxStepsRight - maxStepsLeft);
  Serial.println(" steps");
  
  if (isCalibrated) {
    Serial.println("\n✓ System is CALIBRATED and ready!");
  } else {
    Serial.println("\n⚠ Please complete calibration steps 1-3");
  }
  Serial.println("════════════════════════════════════════════\n");
}

void printMainMenu() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║  STEP-BASED CALIBRATION (RELIABLE!)       ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\n--- STEP 1: SET ZERO POSITION ---");
  Serial.println("1 - Set Zero (center arm, pendulum up, press Z)");
  Serial.println("\n--- STEP 2: VERIFY SENSORS ---");
  Serial.println("2 - Live Sensor Test");
  Serial.println("\n--- STEP 3: SET STEP LIMITS ---");
  Serial.println("3 - Set LEFT Limit (manually jog to left edge)");
  Serial.println("4 - Set RIGHT Limit (manually jog to right edge)");
  Serial.println("5 - Show Calibration Data");
  Serial.println("\n--- STEP 4: TEST & MOVE ---");
  Serial.println("6 - Test Full Range (move between limits)");
  Serial.println("7 - Jog Left (CW, -1 step)");
  Serial.println("8 - Jog Right (CCW, +1 step)");
  Serial.println("9 - Return to Zero");
  Serial.println("\n--- QUICK LIMITS (if you know them) ---");
  Serial.println("L - Set limits to ±10 steps (default)");
  Serial.println("W - Set limits to ±15 steps");
  Serial.println("M - Set limits to ±20 steps");
  Serial.println("--- DIAGNOSTICS ---");
  Serial.println("D - EMI Test (disable motor, check sensor stability)");
  Serial.println("N - Motor Power Noise Test (capacitor effectiveness)");
  Serial.println("R - RAW Diagnostic Mode (see bit-flip patterns)");
  Serial.println("\n--- UTILITIES ---");
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
      uint16_t motor_single = readAS5600Angle(motorWire);
      
      // Median-filtered reads (robust)
      uint16_t pend_median = readPendulumAngleMedian();
      uint16_t motor_median = readAS5600AngleMedian(motorWire);
      
      // Convert to degrees
      float pend_deg = (pend_median * 360.0) / 4096.0;
      float motor_deg = (motor_median * 360.0) / 4096.0;
      
      // Highlight differences
      bool pend_diff = abs((int)pend_single - (int)pend_median) > 100;
      bool motor_diff = abs((int)motor_single - (int)motor_median) > 100;
      
      if (pend_diff || motor_diff) Serial.print("⚠ ");
      else Serial.print("  ");
      
      Serial.print(pend_single);
      Serial.print("\\t");
      Serial.print(pend_median);
      Serial.print("\\t");
      Serial.print(motor_single);
      Serial.print("\\t");
      Serial.print(motor_median);
      Serial.print("\\t");
      Serial.print(pend_deg, 1);
      Serial.print("°\\t");
      Serial.print(motor_deg, 1);
      Serial.println("°");
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
  stepper.setCurrentPosition(stepper.currentPosition() + direction);

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
  
  Serial.println("\n--- Software I2C (pins 22/24) ---");
  byte sw_count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    motorWire.beginTransmission(addr);
    if (motorWire.endTransmission() == 0) {
      Serial.print("  0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      if (addr == 0x36) Serial.print(" (AS5600)");
      Serial.println();
      sw_count++;
    }
  }
  if (sw_count == 0) Serial.println("  None");
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
  bool pendulum_ok = false;
  // Try reading angle directly instead of detectMagnet (which might hang)
  uint16_t test_angle = as5600_pendulum.readAngle();
  if (test_angle > 0 && test_angle < 4096) {
    pendulum_ok = true;
    Serial.print("[OK] angle=");
    Serial.println(test_angle);
  } else {
    Serial.println("[ERROR]");
    Serial.println("  → Check magnet distance (2-3mm from chip)");
    Serial.println("  → Magnet may be too far or missing");
  }
  
  // Initialize Software I2C
  Serial.print("Initializing Software I2C... ");
  motorWire.begin();
  motorWire.setClock(100000);  // 100kHz
  delay(200);  // Give capacitors time to stabilize
  Serial.println("done");
  
  // Test motor sensor with raw I2C first
  Serial.print("Testing I2C bus for device 0x36... ");
  motorWire.beginTransmission(0x36);
  uint8_t motor_error = motorWire.endTransmission();
  
  if (motor_error == 0) {
    Serial.println("found!");
  } else {
    Serial.print("error ");
    Serial.println(motor_error);
    Serial.println("  → Device not responding on Software I2C");
    Serial.println("  → Check pins 22 (SDA), 24 (SCL)");
    Serial.println("  → Check 5V, GND, and pull-up resistors");
  }
  
  // Test motor sensor by reading angle
  Serial.print("Reading motor sensor angle... ");
  uint16_t motor_test = readAS5600Angle(motorWire);
  if (motor_test > 0 && motor_test < 4096) {
    Serial.print("[OK] angle=");
    Serial.println(motor_test);
  } else {
    Serial.println("[ERROR]");
    Serial.println("  → Check magnet distance (2-3mm from chip)");
  }
  
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
      case '6': testFullRange(); printMainMenu(); break;
      case '7': jogLeft(); break;
      case '8': jogRight(); break;
      case '9': returnToZero(); printMainMenu(); break;
      case '0': runI2CScan(); printMainMenu(); break;
      case 'l': case 'L': setQuickLimits(10); printMainMenu(); break;
      case 'w': case 'W': setQuickLimits(15); printMainMenu(); break;
      case 'm': case 'M': setQuickLimits(20); printMainMenu(); break;
      case 'd': case 'D': liveSensorTest(); printMainMenu(); break;
      case 'n': case 'N': motorPowerNoiseTest(); printMainMenu(); break;
      case 'r': case 'R': rawDiagnosticMode(); printMainMenu(); break;
      case 'c': case 'C': clearCalibration(); printMainMenu(); break;
      default: 
        Serial.println("Invalid."); 
        delay(300); 
        printMainMenu(); 
        break;
    }
  }
}
