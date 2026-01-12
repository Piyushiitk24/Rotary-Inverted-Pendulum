/*
 * Staircase Position Test
 * 
 * Purpose: Compare stepper motor position tracking vs AS5600 sensor readings
 * Test Profile: Step 50 steps (~11.25 deg), wait 150ms for settling, log, repeat
 * Range: +/- 250 steps (~56 deg)
 * Logging: 1 line per step after settling
 * 
 * Hardware:
 * - Arduino Mega 2560
 * - TMC2209 Driver (STEP=11, DIR=6, EN=7, Active LOW)
 * - AS5600 via TCA9548A I2C Mux (Channel 1)
 * - Mechanics: 1.8 deg/step, 8 microsteps = 0.225 deg/step
 * 
 * Command Sequence (prevents startup jumps):
 *   E - Enable motor (hold torque)
 *   Z - Zero sensor and stepper counters relative to hold position
 *   S - Start staircase test
 *   X - Stop test
 * 
 * Serial: 500000 baud
 * CSV Output: timestamp_ms,target_deg,sensor_deg,stepper_deg,error_deg
 */

#include <Arduino.h>
#include <Wire.h>
#include <FastAccelStepper.h>
#include <AS5600.h>

// ========== Hardware Configuration ==========
#define STEP_PIN        11
#define DIR_PIN         6
#define EN_PIN          7     // Active LOW

// I2C Multiplexer
#define TCA_ADDRESS     0x70
#define MOTOR_CHANNEL   1

// Stepper Configuration
#define MICROSTEPS      8
#define STEPS_PER_REV   200
#define DEG_PER_STEP    (360.0 / (STEPS_PER_REV * MICROSTEPS))  // 0.225 deg/step
#define STEPS_PER_DEG   (1.0 / DEG_PER_STEP)                   // ~4.444 steps/deg

// Serial Configuration
#define SERIAL_BAUD     500000

// Staircase Test Parameters
#define STEP_SIZE       50      // steps per move (~11.25 degrees)
#define SETTLE_TIME_MS  150     // settling time after each move
#define LOG_INTERVAL_MS 20      // log every 20ms during settling (for staircase visualization)
#define MAX_STEPS       250     // +/- range limit
#define TOTAL_POSITIONS 21      // -250 to +250 and back: 11 up + 10 down

// ========== Global Objects ==========
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
AS5600 motorSensor;

// ========== State Variables ==========
enum SystemState {
  STATE_IDLE,
  STATE_ENABLED,
  STATE_RUNNING,
  STATE_MOVING,
  STATE_SETTLING
};

SystemState currentState = STATE_IDLE;
unsigned long testStartTime = 0;
unsigned long settleStartTime = 0;
unsigned long lastLogTime = 0;

float sensorZeroAngle = 0.0;
long stepperZeroPosition = 0;

// Test progression
int currentStepIndex = 0;
long targetPositions[TOTAL_POSITIONS];
int testDirection = 1;  // 1 = forward, -1 = backward

// ========== I2C Multiplexer Helper ==========
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ========== Sensor Reading ==========
float readMotorAngle() {
  tcaSelect(MOTOR_CHANNEL);
  
  // Check if magnet is present
  if (motorSensor.detectMagnet() == 0) {
    return NAN;
  }
  
  // Read raw angle (0-4095) and convert to degrees
  float rawAngle = motorSensor.rawAngle() * (360.0 / 4096.0);
  
  // Apply zero offset
  float calibratedAngle = rawAngle - sensorZeroAngle;
  
  // Normalize to -180 to +180
  while (calibratedAngle > 180.0) calibratedAngle -= 360.0;
  while (calibratedAngle < -180.0) calibratedAngle += 360.0;
  
  return calibratedAngle;
}

// ========== Test Initialization ==========
void initializeTestPositions() {
  // Generate positions: -250 to +250 (climbing), then +200 to -250 (descending)
  // Climbing: -250, -200, -150, -100, -50, 0, 50, 100, 150, 200, 250 (11 positions)
  // Descending: 200, 150, 100, 50, 0, -50, -100, -150, -200, -250 (10 positions)
  
  int pos = 0;
  // Climbing phase (11 positions)
  for (int i = -5; i <= 5; i++) {
    targetPositions[pos++] = stepperZeroPosition + i * STEP_SIZE;
  }
  // Descending phase (10 positions, skip +250 since we just did it)
  for (int i = 4; i >= -5; i--) {
    targetPositions[pos++] = stepperZeroPosition + i * STEP_SIZE;
  }
  
  currentStepIndex = 0;
  testDirection = 1;
}

// ========== Command Handlers ==========
void handleEnableMotor() {
  if (currentState == STATE_IDLE) {
    digitalWrite(EN_PIN, LOW);  // Active LOW - enable motor
    delay(50);  // Allow driver to stabilize
    currentState = STATE_ENABLED;
    Serial.println("# Motor ENABLED - torque engaged");
    Serial.println("# Use 'Z' to zero position, then 'S' to start test");
  } else {
    Serial.println("# Motor already enabled");
  }
}

void handleZeroPosition() {
  if (currentState == STATE_ENABLED || currentState == STATE_IDLE) {
    // Read current sensor position
    tcaSelect(MOTOR_CHANNEL);
    if (motorSensor.detectMagnet() == 0) {
      Serial.println("# ERROR: No magnet detected on motor sensor");
      return;
    }
    
    float currentRawAngle = motorSensor.rawAngle() * (360.0 / 4096.0);
    sensorZeroAngle = currentRawAngle;
    
    // Zero stepper position
    stepperZeroPosition = stepper->getCurrentPosition();
    
    Serial.print("# Position ZEROED - Sensor raw: ");
    Serial.print(currentRawAngle, 2);
    Serial.print(" deg, Stepper pos: ");
    Serial.println(stepperZeroPosition);
    Serial.println("# Use 'S' to start staircase test");
    
    if (currentState == STATE_IDLE) {
      currentState = STATE_ENABLED;
    }
  } else {
    Serial.println("# Cannot zero while test is running - use 'X' to stop first");
  }
}

void handleStartTest() {
  if (currentState == STATE_ENABLED) {
    initializeTestPositions();
    currentState = STATE_RUNNING;
    testStartTime = millis();
    
    Serial.println("# ========================================");
    Serial.println("# Staircase Position Test Started");
    Serial.println("# Step Size: 50 steps (~11.25 deg)");
    Serial.println("# Range: +/- 250 steps (~56 deg)");
    Serial.println("# Settling Time: 150 ms");
    Serial.println("# Bidirectional: Climb up, then descend back");
    Serial.println("# ========================================");
    Serial.println("timestamp_ms,target_deg,sensor_deg,stepper_deg,error_deg");
  } else if (currentState == STATE_IDLE) {
    Serial.println("# ERROR: Motor not enabled - use 'E' first");
  } else {
    Serial.println("# Test already running");
  }
}

void handleStopTest() {
  if (currentState == STATE_RUNNING || currentState == STATE_MOVING || currentState == STATE_SETTLING) {
    currentState = STATE_ENABLED;
    
    // Stop stepper smoothly
    stepper->stopMove();
    
    unsigned long duration = millis() - testStartTime;
    Serial.println("# ========================================");
    Serial.println("# Test STOPPED");
    Serial.print("# Duration: ");
    Serial.print(duration / 1000.0, 2);
    Serial.println(" seconds");
    Serial.print("# Completed steps: ");
    Serial.print(currentStepIndex);
    Serial.print(" / ");
    Serial.println(TOTAL_POSITIONS);
    Serial.println("# ========================================");
  } else {
    Serial.println("# No test running");
  }
}

void handleDisableMotor() {
  if (currentState != STATE_IDLE) {
    if (currentState == STATE_RUNNING || currentState == STATE_MOVING || currentState == STATE_SETTLING) {
      stepper->stopMove();
      currentState = STATE_ENABLED;
      Serial.println("# Test stopped");
    }
    
    digitalWrite(EN_PIN, HIGH);  // Active LOW - disable motor
    currentState = STATE_IDLE;
    Serial.println("# Motor DISABLED - torque released");
  } else {
    Serial.println("# Motor already disabled");
  }
}

void handleCommand() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'E':
      case 'e':
        handleEnableMotor();
        break;
        
      case 'Z':
      case 'z':
        handleZeroPosition();
        break;
        
      case 'S':
      case 's':
        handleStartTest();
        break;
        
      case 'X':
      case 'x':
        handleStopTest();
        break;
        
      case 'D':
      case 'd':
        handleDisableMotor();
        break;
        
      case 'C':
      case 'c':
        // Connection check
        Serial.println("# Connection OK");
        Serial.print("# State: ");
        if (currentState == STATE_IDLE) Serial.println("IDLE");
        else if (currentState == STATE_ENABLED) Serial.println("ENABLED");
        else if (currentState == STATE_RUNNING) Serial.println("RUNNING");
        else if (currentState == STATE_MOVING) Serial.println("MOVING");
        else Serial.println("SETTLING");
        break;
        
      default:
        if (cmd > 32) {  // Printable character
          Serial.println("# Unknown command");
          Serial.println("# Commands: E=Enable, Z=Zero, S=Start, X=Stop, D=Disable, C=Check");
        }
        break;
    }
  }
}

// ========== Test Logic ==========
void runTestLogic() {
  if (currentState == STATE_RUNNING) {
    // Check if we've completed all positions
    if (currentStepIndex >= TOTAL_POSITIONS) {
      Serial.println("# ========================================");
      Serial.println("# Test COMPLETED - All positions visited");
      unsigned long duration = millis() - testStartTime;
      Serial.print("# Duration: ");
      Serial.print(duration / 1000.0, 2);
      Serial.println(" seconds");
      Serial.println("# ========================================");
      currentState = STATE_ENABLED;
      return;
    }
    
    // Move to next target position
    long targetSteps = targetPositions[currentStepIndex];
    stepper->moveTo(targetSteps);
    
    // Transition to moving state - wait for motion to complete
    currentState = STATE_MOVING;
    
  } else if (currentState == STATE_MOVING) {
    // Wait for stepper to reach target position
    if (stepper->isRunning()) {
      return;  // Still moving, keep waiting
    }
    
    // Motion complete - start settling timer and log immediately
    currentState = STATE_SETTLING;
    settleStartTime = millis();
    lastLogTime = millis();
    
    // Log first point at start of settling
    long currentSteps = stepper->getCurrentPosition();
    float stepperAngle = (currentSteps - stepperZeroPosition) * DEG_PER_STEP;
    float targetAngle = (targetPositions[currentStepIndex] - stepperZeroPosition) * DEG_PER_STEP;
    float sensorAngle = readMotorAngle();
    float error = sensorAngle - targetAngle;
    
    Serial.print(millis());
    Serial.print(",");
    Serial.print(targetAngle, 4);
    Serial.print(",");
    if (isnan(sensorAngle)) {
      Serial.print("NaN");
    } else {
      Serial.print(sensorAngle, 4);
    }
    Serial.print(",");
    Serial.print(stepperAngle, 4);
    Serial.print(",");
    if (isnan(sensorAngle)) {
      Serial.print("NaN");
    } else {
      Serial.print(error, 4);
    }
    Serial.println();
    
  } else if (currentState == STATE_SETTLING) {
    unsigned long elapsed = millis() - settleStartTime;
    
    // Log periodically during settling (every LOG_INTERVAL_MS)
    if (millis() - lastLogTime >= LOG_INTERVAL_MS && elapsed < SETTLE_TIME_MS) {
      lastLogTime = millis();
      
      long currentSteps = stepper->getCurrentPosition();
      float stepperAngle = (currentSteps - stepperZeroPosition) * DEG_PER_STEP;
      float targetAngle = (targetPositions[currentStepIndex] - stepperZeroPosition) * DEG_PER_STEP;
      float sensorAngle = readMotorAngle();
      float error = sensorAngle - targetAngle;
      
      Serial.print(millis());
      Serial.print(",");
      Serial.print(targetAngle, 4);
      Serial.print(",");
      if (isnan(sensorAngle)) {
        Serial.print("NaN");
      } else {
        Serial.print(sensorAngle, 4);
      }
      Serial.print(",");
      Serial.print(stepperAngle, 4);
      Serial.print(",");
      if (isnan(sensorAngle)) {
        Serial.print("NaN");
      } else {
        Serial.print(error, 4);
      }
      Serial.println();
    }
    
    // Check if settling time is complete
    if (elapsed < SETTLE_TIME_MS) {
      return;
    }
    
    // Settling complete - move to next position
    currentStepIndex++;
    currentState = STATE_RUNNING;
  }
}

// ========== Setup ==========
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  
  Serial.println("# ========================================");
  Serial.println("# Staircase Position Test");
  Serial.println("# Stepper vs AS5600 Tracking Comparison");
  Serial.println("# ========================================");
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);  // 400 kHz
  
  // Initialize motor sensor
  tcaSelect(MOTOR_CHANNEL);
  motorSensor.begin();
  delay(100);
  
  // Check if magnet is detected
  tcaSelect(MOTOR_CHANNEL);
  if (motorSensor.detectMagnet() == 0) {
    Serial.println("# WARNING: No magnet detected on motor sensor (Channel 1)");
  } else {
    Serial.println("# Motor sensor initialized (AS5600 on Channel 1)");
  }
  
  // Initialize stepper motor
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);  // Start disabled (Active LOW)
  
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  
  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(EN_PIN);
    stepper->setAutoEnable(false);  // Manual control of enable pin
    
    // Set speed and acceleration limits
    stepper->setSpeedInHz(2000);      // Max 2000 steps/sec (smoother for staircase)
    stepper->setAcceleration(5000);   // steps/sec^2
    
    Serial.println("# Stepper motor initialized (FastAccelStepper)");
    Serial.print("# Steps per degree: ");
    Serial.println(STEPS_PER_DEG, 4);
    Serial.print("# Step size: ");
    Serial.print(STEP_SIZE);
    Serial.print(" steps = ");
    Serial.print(STEP_SIZE * DEG_PER_STEP, 2);
    Serial.println(" deg");
  } else {
    Serial.println("# ERROR: Failed to initialize stepper");
  }
  
  Serial.println("# ========================================");
  Serial.println("# Ready for commands:");
  Serial.println("#   E - Enable motor (hold torque)");
  Serial.println("#   Z - Zero position");
  Serial.println("#   S - Start staircase test");
  Serial.println("#   X - Stop test");
  Serial.println("#   D - Disable motor");
  Serial.println("#   C - Connection check");
  Serial.println("# ========================================");
}

// ========== Main Loop ==========
void loop() {
  handleCommand();
  
  // FastAccelStepper uses interrupts - no run() call needed
  
  runTestLogic();
}
