/*
 * Sine Wave Position Control Test
 * 
 * Purpose: Compare stepper motor position tracking vs AS5600 sensor readings
 * Test Profile: Sine wave (Amplitude: 20 deg, Frequency: 1 Hz)
 * Control Loop: 200 Hz (5 ms)
 * Logging Rate: 200 Hz (5 ms)
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
 *   S - Start sine wave test
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

// Control Loop Configuration
#define LOOP_INTERVAL_US    5000    // 5 ms = 200 Hz
#define SERIAL_BAUD         500000

// Sine Wave Test Parameters
#define SINE_AMPLITUDE_DEG  20.0    // +/- 20 degrees
#define SINE_FREQ_HZ        10.0     // 1 Hz
// Note: TWO_PI is already defined in Arduino.h

// ========== Global Objects ==========
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
AS5600 motorSensor;

// ========== State Variables ==========
enum SystemState {
  STATE_IDLE,
  STATE_ENABLED,
  STATE_RUNNING
};

SystemState currentState = STATE_IDLE;
unsigned long testStartTime = 0;
unsigned long lastLoopTime = 0;

float sensorZeroAngle = 0.0;
long stepperZeroPosition = 0;

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
    Serial.println("# Use 'S' to start sine wave test");
    
    if (currentState == STATE_IDLE) {
      currentState = STATE_ENABLED;
    }
  } else {
    Serial.println("# Cannot zero while test is running - use 'X' to stop first");
  }
}

void handleStartTest() {
  if (currentState == STATE_ENABLED) {
    currentState = STATE_RUNNING;
    testStartTime = millis();
    lastLoopTime = micros();
    
    Serial.println("# ========================================");
    Serial.println("# Sine Wave Position Control Test Started");
    Serial.println("# Amplitude: 20 deg, Frequency: 1 Hz");
    Serial.println("# Control Rate: 200 Hz (5 ms)");
    Serial.println("# ========================================");
    Serial.println("timestamp_ms,target_deg,sensor_deg,stepper_deg,error_deg");
  } else if (currentState == STATE_IDLE) {
    Serial.println("# ERROR: Motor not enabled - use 'E' first");
  } else {
    Serial.println("# Test already running");
  }
}

void handleStopTest() {
  if (currentState == STATE_RUNNING) {
    currentState = STATE_ENABLED;
    
    // Stop stepper smoothly
    stepper->stopMove();
    
    unsigned long duration = millis() - testStartTime;
    Serial.println("# ========================================");
    Serial.println("# Test STOPPED");
    Serial.print("# Duration: ");
    Serial.print(duration / 1000.0, 2);
    Serial.println(" seconds");
    Serial.println("# ========================================");
  } else {
    Serial.println("# No test running");
  }
}

void handleDisableMotor() {
  if (currentState != STATE_IDLE) {
    if (currentState == STATE_RUNNING) {
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
        else Serial.println("RUNNING");
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

// ========== Control Loop ==========
void runControlLoop() {
  if (currentState != STATE_RUNNING) return;
  
  unsigned long now = micros();
  if (now - lastLoopTime < LOOP_INTERVAL_US) return;
  lastLoopTime = now;
  
  // Calculate elapsed time in seconds
  float elapsedTime = (millis() - testStartTime) / 1000.0;
  
  // Generate sine wave target position
  float targetAngle = SINE_AMPLITUDE_DEG * sin(TWO_PI * SINE_FREQ_HZ * elapsedTime);
  
  // Convert target angle to stepper steps
  long targetSteps = (long)(targetAngle * STEPS_PER_DEG) + stepperZeroPosition;
  
  // Move stepper to target position
  stepper->moveTo(targetSteps);
  
  // Read current sensor angle
  float sensorAngle = readMotorAngle();
  
  // Calculate stepper angle from steps
  long currentSteps = stepper->getCurrentPosition();
  float stepperAngle = (currentSteps - stepperZeroPosition) * DEG_PER_STEP;
  
  // Calculate error (sensor reading vs target)
  float error = sensorAngle - targetAngle;
  
  // Log data: timestamp_ms,target_deg,sensor_deg,stepper_deg,error_deg
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

// ========== Setup ==========
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  
  Serial.println("# ========================================");
  Serial.println("# Sine Wave Position Control Test");
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
    stepper->setSpeedInHz(4000);      // Max 4000 steps/sec
    stepper->setAcceleration(10000);  // steps/sec^2
    
    Serial.println("# Stepper motor initialized (FastAccelStepper)");
    Serial.print("# Steps per degree: ");
    Serial.println(STEPS_PER_DEG, 4);
  } else {
    Serial.println("# ERROR: Failed to initialize stepper");
  }
  
  Serial.println("# ========================================");
  Serial.println("# Ready for commands:");
  Serial.println("#   E - Enable motor (hold torque)");
  Serial.println("#   Z - Zero position");
  Serial.println("#   S - Start sine wave test");
  Serial.println("#   X - Stop test");
  Serial.println("#   D - Disable motor");
  Serial.println("#   C - Connection check");
  Serial.println("# ========================================");
}

// ========== Main Loop ==========
void loop() {
  handleCommand();
  
  // FastAccelStepper uses interrupts - no run() call needed
  
  runControlLoop();
}
