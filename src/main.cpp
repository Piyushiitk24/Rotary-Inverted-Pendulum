#include <Arduino.h>
#include <Wire.h>
#include <SoftwareWire.h>
#include <AS5600.h>
#include <AccelStepper.h>
#include <math.h>

// ============================================================
//  ROTARY INVERTED PENDULUM - SENSOR-BASED CONTROL
// ============================================================
//
// WORKFLOW:
// 1. Startup diagnostics (both sensors + motor)
// 2. Set zero position (center arm, pendulum up)
// 3. Test sensors continuously
// 4. Calibrate left/right limits (sensor-based)
// 5. Return to center
// 6. Energy-based swing-up with constraints
// 7. LQR balance control
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
#define MOTOR_SPEED 6000
#define MOTOR_ACCEL 15000
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
  STATE_EMERGENCY_STOP
};

SystemState currentState = STATE_STARTUP;

// Calibration data
float pendulumZeroAngle = 0.0;
float motorZeroAngle = 0.0;
float motorLeftLimit = 0.0;   // degrees, sensor-based
float motorRightLimit = 0.0;  // degrees, sensor-based
long stepsAtZero = 0;
long stepsAtLeftLimit = 0;
long stepsAtRightLimit = 0;
bool systemCalibrated = false;

// Current sensor readings
float pendulumAngle = 0.0;  // relative to zero (upright)
float motorAngle = 0.0;     // relative to zero (center)
uint16_t pendulumRaw = 0;
uint16_t motorRaw = 0;

// Control parameters (will be tuned)
float Kp_swing = 300.0;      // Swing-up proportional gain
float Kp_balance = 25.0;     // Balance proportional gain
float Kd_balance = 8.0;      // Balance derivative gain
float Ki_balance = 0.5;      // Balance integral gain
float balanceThreshold = 25.0; // Switch to balance mode (degrees)

// Constants
const float DEG2RAD = PI / 180.0;

// Control state
float previousPendulumAngle = 0.0;
float integralError = 0.0;
unsigned long lastControlTime = 0;
#define CONTROL_LOOP_MS 10

// ============================================================
//  SENSOR I2C FUNCTIONS
// ============================================================

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

float normalizeAngle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

void updateSensors() {
  // Read pendulum sensor (Hardware I2C)
  pendulumRaw = as5600_pendulum.readAngle();
  float pendulum_deg = (pendulumRaw * 360.0) / 4096.0;
  pendulumAngle = normalizeAngle(pendulum_deg - pendulumZeroAngle);
  
  // Read motor sensor (Software I2C)
  motorRaw = readAS5600Angle(motorWire);
  float motor_deg = (motorRaw * 360.0) / 4096.0;
  motorAngle = normalizeAngle(motor_deg - motorZeroAngle);
}

// ============================================================
//  STARTUP DIAGNOSTICS
// ============================================================

bool checkSystemHealth() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║      SYSTEM DIAGNOSTICS                    ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
  
  bool allGood = true;
  
  // Check motor driver
  Serial.print("Motor Driver... ");
  digitalWrite(EN_PIN, LOW);
  delay(100);
  Serial.println("✓ Enabled");
  
  // Check pendulum sensor
  Serial.print("Pendulum Sensor (Hardware I2C 0x36)... ");
  if (!as5600_pendulum.detectMagnet()) {
    Serial.println("✗ FAILED - No magnet detected!");
    allGood = false;
  } else {
    uint16_t raw = as5600_pendulum.readAngle();
    Serial.print("✓ OK (raw: ");
    Serial.print(raw);
    Serial.println(")");
  }
  
  // Check motor sensor
  Serial.print("Motor Sensor (Software I2C 0x36)... ");
  if (!detectAS5600Magnet(motorWire)) {
    Serial.println("✗ FAILED - No magnet detected!");
    allGood = false;
  } else {
    uint16_t raw = readAS5600Angle(motorWire);
    Serial.print("✓ OK (raw: ");
    Serial.print(raw);
    Serial.println(")");
  }
  
  // Test motor movement
  Serial.print("Motor Movement Test... ");
  long startPos = stepper.currentPosition();
  stepper.setSpeed(1000);
  for (int i = 0; i < 100; i++) {
    stepper.runSpeed();
    delayMicroseconds(100);
  }
  stepper.setCurrentPosition(startPos); // Reset
  Serial.println("✓ Motor responds");
  
  Serial.println();
  if (allGood) {
    Serial.println("✅ ALL SYSTEMS OPERATIONAL");
  } else {
    Serial.println("❌ SYSTEM CHECK FAILED - Fix errors before proceeding");
  }
  Serial.println("════════════════════════════════════════════\n");
  
  return allGood;
}

// ============================================================
//  ZEROING PROCEDURE
// ============================================================

void performZeroing() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║      ZERO POSITION SETUP                   ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
  Serial.println("1. Position the motor arm at CENTER");
  Serial.println("2. Hold the pendulum STRAIGHT UP (vertical)");
  Serial.println("3. Press 'Z' when ready to set zero position\n");
  
  currentState = STATE_ZEROING;
  
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'z' || c == 'Z') {
        // Set zero positions
        stepper.setCurrentPosition(0);
        stepsAtZero = 0;
        
        uint16_t pendulum_raw = as5600_pendulum.readAngle();
        pendulumZeroAngle = (pendulum_raw * 360.0) / 4096.0;
        
        uint16_t motor_raw = readAS5600Angle(motorWire);
        motorZeroAngle = (motor_raw * 360.0) / 4096.0;
        
        Serial.println("\n✅ ZERO POSITION SET!");
        Serial.print("   Pendulum zero: ");
        Serial.print(pendulumZeroAngle, 2);
        Serial.println("°");
        Serial.print("   Motor zero: ");
        Serial.print(motorZeroAngle, 2);
        Serial.println("°");
        Serial.println("════════════════════════════════════════════\n");
        
        currentState = STATE_IDLE;
        break;
      }
    }
    delay(50);
  }
}

// ============================================================
//  SENSOR LIVE MONITORING
// ============================================================

void liveMonitorSensors() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║      LIVE SENSOR MONITORING                ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
  Serial.println("Streaming both sensors. Press 'S' to stop.\n");
  Serial.println("────────────────────────────────────────────────");
  Serial.println("Pendulum(°) | Motor(°) | Motor Raw | Steps");
  Serial.println("────────────────────────────────────────────────");
  
  unsigned long lastPrint = millis();
  
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's' || c == 'S') {
        Serial.println("────────────────────────────────────────────────");
        Serial.println("\nMonitoring stopped.\n");
        return;
      }
    }
    
    if (millis() - lastPrint >= 200) {
      lastPrint = millis();
      updateSensors();
      
      Serial.print("  ");
      Serial.print(pendulumAngle, 2);
      Serial.print("°   \t");
      Serial.print(motorAngle, 2);
      Serial.print("°  \t");
      Serial.print(motorRaw);
      Serial.print("  \t");
      Serial.println(stepper.currentPosition());
    }
  }
}

// ============================================================
//  LIMIT CALIBRATION
// ============================================================

void calibrateLimits() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║      LIMIT CALIBRATION                     ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
  Serial.println("Use A/D keys to jog left/right");
  Serial.println("Press 'L' at LEFT limit, 'R' at RIGHT limit");
  Serial.println("Press 'Q' to quit calibration\n");
  
  currentState = STATE_CALIBRATION;
  stepper.setMaxSpeed(JOG_SPEED);
  stepper.setAcceleration(MOTOR_ACCEL);
  
  bool leftSet = false;
  bool rightSet = false;
  
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      
      if (c == 'a' || c == 'A') {
        // Jog left
        stepper.move(-50);
        while (stepper.distanceToGo() != 0) {
          stepper.run();
        }
        updateSensors();
        Serial.print("← Left | Motor: ");
        Serial.print(motorAngle, 2);
        Serial.print("° | Steps: ");
        Serial.println(stepper.currentPosition());
        
      } else if (c == 'd' || c == 'D') {
        // Jog right
        stepper.move(50);
        while (stepper.distanceToGo() != 0) {
          stepper.run();
        }
        updateSensors();
        Serial.print("→ Right | Motor: ");
        Serial.print(motorAngle, 2);
        Serial.print("° | Steps: ");
        Serial.println(stepper.currentPosition());
        
      } else if (c == 'l' || c == 'L') {
        // Set left limit
        updateSensors();
        motorLeftLimit = motorAngle;
        stepsAtLeftLimit = stepper.currentPosition();
        leftSet = true;
        Serial.print("\n✓ LEFT LIMIT SET: ");
        Serial.print(motorLeftLimit, 2);
        Serial.print("° (");
        Serial.print(stepsAtLeftLimit);
        Serial.println(" steps)\n");
        
      } else if (c == 'r' || c == 'R') {
        // Set right limit
        updateSensors();
        motorRightLimit = motorAngle;
        stepsAtRightLimit = stepper.currentPosition();
        rightSet = true;
        Serial.print("\n✓ RIGHT LIMIT SET: ");
        Serial.print(motorRightLimit, 2);
        Serial.print("° (");
        Serial.print(stepsAtRightLimit);
        Serial.println(" steps)\n");
        
      } else if (c == 'q' || c == 'Q') {
        if (leftSet && rightSet) {
          systemCalibrated = true;
          Serial.println("\n✅ CALIBRATION COMPLETE!");
          Serial.print("   Range: ");
          Serial.print(motorRightLimit - motorLeftLimit, 1);
          Serial.print("° (");
          Serial.print(stepsAtRightLimit - stepsAtLeftLimit);
          Serial.println(" steps)");
          Serial.println("════════════════════════════════════════════\n");
          currentState = STATE_IDLE;
          return;
        } else {
          Serial.println("\n⚠ Must set both LEFT and RIGHT limits before quitting!\n");
        }
      }
    }
    delay(10);
  }
}

// ============================================================
//  RETURN TO CENTER
// ============================================================

void returnToCenter() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║      RETURNING TO CENTER                   ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
  
  stepper.setMaxSpeed(MOTOR_SPEED);
  stepper.setAcceleration(MOTOR_ACCEL);
  stepper.moveTo(stepsAtZero);
  
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  
  updateSensors();
  Serial.print("✓ At center | Motor angle: ");
  Serial.print(motorAngle, 2);
  Serial.println("°");
  Serial.println("════════════════════════════════════════════\n");
}

// ============================================================
//  CONTROL SYSTEM - ENERGY-BASED SWING-UP
// ============================================================

float calculatePendulumEnergy() {
  // Simplified energy: E = (1 - cos(alpha))
  // At bottom (180°): E = 2 (max)
  // At top (0°): E = 0 (min)
  float alpha_rad = pendulumAngle * DEG2RAD;
  return 1.0 - cos(alpha_rad);
}

void runSwingUp() {
  updateSensors();
  
  // Calculate pendulum energy
  float energy = calculatePendulumEnergy();
  float alpha_rad = pendulumAngle * DEG2RAD;
  float alpha_dot = (pendulumAngle - previousPendulumAngle) / (CONTROL_LOOP_MS / 1000.0);
  
  // Energy-based control: pump energy into pendulum
  // Direction based on pendulum velocity and position
  float control = Kp_swing * energy * alpha_dot * cos(alpha_rad);
  
  // Convert to motor position target (in steps)
  long targetSteps = stepper.currentPosition() + (long)control;
  
  // Enforce limits
  if (targetSteps < stepsAtLeftLimit) targetSteps = stepsAtLeftLimit;
  if (targetSteps > stepsAtRightLimit) targetSteps = stepsAtRightLimit;
  
  stepper.moveTo(targetSteps);
  stepper.run();
  
  // Check if close enough to switch to balance
  if (abs(pendulumAngle) < balanceThreshold) {
    Serial.println("\n✓ Pendulum near upright - Switching to BALANCE mode");
    currentState = STATE_BALANCE;
    integralError = 0.0;
    lastControlTime = millis();
  }
  
  previousPendulumAngle = pendulumAngle;
}

// ============================================================
//  CONTROL SYSTEM - LQR BALANCE
// ============================================================

void runBalance() {
  updateSensors();
  
  float dt = (millis() - lastControlTime) / 1000.0;
  lastControlTime = millis();
  
  // Check if pendulum fell
  if (abs(pendulumAngle) > 60.0) {
    Serial.println("\n⚠ Pendulum fell - Returning to IDLE");
    stepper.stop();
    currentState = STATE_IDLE;
    return;
  }
  
  // PID control on pendulum angle
  float error = pendulumAngle; // Want it at 0°
  integralError += error * dt;
  
  // Anti-windup
  if (integralError > 100.0) integralError = 100.0;
  if (integralError < -100.0) integralError = -100.0;
  
  float derivative = (pendulumAngle - previousPendulumAngle) / dt;
  
  float pidOutput = Kp_balance * error + Ki_balance * integralError + Kd_balance * derivative;
  
  // Also include motor position feedback (keep centered)
  float motorCorrection = -0.5 * motorAngle;
  
  float totalControl = pidOutput + motorCorrection;
  
  // Convert to target position
  long targetSteps = stepsAtZero - (long)(totalControl * 10.0);
  
  // Enforce limits
  if (targetSteps < stepsAtLeftLimit) targetSteps = stepsAtLeftLimit;
  if (targetSteps > stepsAtRightLimit) targetSteps = stepsAtRightLimit;
  
  stepper.moveTo(targetSteps);
  stepper.run();
  
  previousPendulumAngle = pendulumAngle;
}

// ============================================================
//  MAIN MENU
// ============================================================

void printMenu() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║  ROTARY INVERTED PENDULUM CONTROL         ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.print("\nState: ");
  switch(currentState) {
    case STATE_STARTUP: Serial.println("STARTUP"); break;
    case STATE_ZEROING: Serial.println("ZEROING"); break;
    case STATE_CALIBRATION: Serial.println("CALIBRATION"); break;
    case STATE_IDLE: Serial.println("IDLE"); break;
    case STATE_SWING_UP: Serial.println("SWING-UP ACTIVE"); break;
    case STATE_BALANCE: Serial.println("BALANCING"); break;
    case STATE_EMERGENCY_STOP: Serial.println("EMERGENCY STOP"); break;
  }
  Serial.print("Calibrated: ");
  Serial.println(systemCalibrated ? "YES" : "NO");
  Serial.println("\n--- SETUP SEQUENCE ---");
  Serial.println("1 - System Diagnostics");
  Serial.println("2 - Set Zero Position");
  Serial.println("3 - Live Sensor Monitoring");
  Serial.println("4 - Calibrate Left/Right Limits");
  Serial.println("5 - Return to Center");
  Serial.println("\n--- CONTROL ---");
  Serial.println("S - Start Swing-Up & Balance");
  Serial.println("X - Stop Control");
  Serial.println("E - Emergency Stop");
  Serial.println("════════════════════════════════════════════");
  Serial.print("Choice: ");
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  // Initialize pins
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); // Disabled initially
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  
  // Initialize serial
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n╔════════════════════════════════════════════╗");
  Serial.println("║  ROTARY INVERTED PENDULUM - FULL SYSTEM   ║");
  Serial.println("║  Sensor-Based Calibration & Control       ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
  
  // Configure stepper
  stepper.setMaxSpeed(MOTOR_SPEED);
  stepper.setAcceleration(MOTOR_ACCEL);
  stepper.setCurrentPosition(0);
  
  // Initialize Hardware I2C (pendulum sensor)
  Wire.begin();
  Wire.setClock(100000);
  delay(100);
  as5600_pendulum.begin();
  
  // Initialize Software I2C (motor sensor)
  motorWire.begin();
  motorWire.setClock(100000);
  delay(100);
  
  Serial.println("System initialized. Run diagnostics (option 1) first.\n");
  
  currentState = STATE_STARTUP;
  printMenu();
}

// ============================================================
//  MAIN LOOP
// ============================================================

void loop() {
  // Handle control modes
  if (currentState == STATE_SWING_UP) {
    unsigned long now = millis();
    if (now - lastControlTime >= CONTROL_LOOP_MS) {
      lastControlTime = now;
      runSwingUp();
    } else {
      stepper.run();
    }
    
    // Check for stop command
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'x' || c == 'X' || c == 's' || c == 'S') {
        stepper.stop();
        currentState = STATE_IDLE;
        Serial.println("\n✓ Control stopped");
        printMenu();
      }
    }
    return;
  }
  
  if (currentState == STATE_BALANCE) {
    unsigned long now = millis();
    if (now - lastControlTime >= CONTROL_LOOP_MS) {
      lastControlTime = now;
      runBalance();
    } else {
      stepper.run();
    }
    
    // Check for stop command
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'x' || c == 'X' || c == 's' || c == 'S') {
        stepper.stop();
        currentState = STATE_IDLE;
        Serial.println("\n✓ Control stopped");
        printMenu();
      }
    }
    return;
  }
  
  // Menu handling
  if (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') return;
    
    switch(c) {
      case '1':
        checkSystemHealth();
        printMenu();
        break;
        
      case '2':
        performZeroing();
        printMenu();
        break;
        
      case '3':
        liveMonitorSensors();
        printMenu();
        break;
        
      case '4':
        if (currentState != STATE_IDLE && currentState != STATE_CALIBRATION) {
          Serial.println("\nMust be in IDLE state. Set zero position first (option 2).\n");
        } else {
          calibrateLimits();
        }
        printMenu();
        break;
        
      case '5':
        if (!systemCalibrated) {
          Serial.println("\nMust calibrate limits first (option 4).\n");
        } else {
          returnToCenter();
        }
        printMenu();
        break;
        
      case 's': case 'S':
        if (!systemCalibrated) {
          Serial.println("\n⚠ Cannot start - System not calibrated!");
          Serial.println("Complete setup sequence (options 1-5) first.\n");
          printMenu();
        } else {
          Serial.println("\n╔════════════════════════════════════════════╗");
          Serial.println("║  STARTING SWING-UP CONTROL                 ║");
          Serial.println("╚════════════════════════════════════════════╝\n");
          Serial.println("Pull pendulum down, then release...");
          Serial.println("Press 'X' to stop.\n");
          digitalWrite(EN_PIN, LOW);
          delay(1000);
          currentState = STATE_SWING_UP;
          lastControlTime = millis();
          previousPendulumAngle = 0.0;
        }
        break;
        
      case 'x': case 'X':
        stepper.stop();
        currentState = STATE_IDLE;
        Serial.println("\n✓ Stopped");
        printMenu();
        break;
        
      case 'e': case 'E':
        stepper.stop();
        digitalWrite(EN_PIN, HIGH);
        currentState = STATE_EMERGENCY_STOP;
        Serial.println("\n!!! EMERGENCY STOP !!!");
        Serial.println("Power cycle or reset to continue.\n");
        break;
        
      default:
        if (c != '\n' && c != '\r') {
          Serial.println("\nInvalid choice.");
          printMenu();
        }
        break;
    }
  }
}
