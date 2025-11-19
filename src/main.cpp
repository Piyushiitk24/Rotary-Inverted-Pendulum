#include <Arduino.h>
#include <Wire.h>
#include <SoftwareWire.h>
#include <AS5600.h>
#include <FastAccelStepper.h>
#include <math.h>
#include <ctype.h>

// ============================================================
// ROTARY INVERTED PENDULUM – V5.3 (FINAL)
// ============================================================
// - Architecture: Non-blocking FastAccelStepper + Interleaved Sensing
// - Fixes: Velocity calculation order, Base Damping, Integral Term
// - Hardware: Mega 2560, NEMA-17, TMC2209 (Standalone/Pot mode)
// ============================================================

// --- PINS ---
#define STEP_PIN 5
#define DIR_PIN  6
#define EN_PIN   7

// Motor Sensor (Software I2C)
#define MOTOR_SDA_PIN 22
#define MOTOR_SCL_PIN 24

// --- CONSTANTS ---
const float DEG_PER_STEP = 0.225f;     // 1.8 deg motor / 8 microsteps
const float PEND_LIMIT_DEG = 45.0f;    // Kill switch angle (Pendulum)
const float BASE_LIMIT_DEG = 60.0f;    // Kill switch angle (Base Arm)

// --- GAINS (Starting Values) ---
// Adjust these via Serial Commands (P, I, D, M, V)
float Kp_balance = 80.0f;   // Stiffness (Pendulum)
float Ki_balance = 0.5f;    // Lean correction
float Kd_balance = 8.0f;    // Damping (Pendulum) - Critical for stopping shake

float Kp_center  = 2.0f;    // Centering force (Base)
float Kd_center  = 0.5f;    // Damping force (Base) - Prevents oscillation

// --- TIMING ---
const unsigned long LOOP_INTERVAL_US = 5000; // 5ms (200Hz)
const float DT_FIXED = 0.005f;               // Fixed math delta
unsigned long lastLoopMicros = 0;
uint16_t loopCounter = 0;
bool balanceEnabled = false;

// --- SENSORS ---
AS5600 pendulumSensor(&Wire);
SoftwareWire motorWire(MOTOR_SDA_PIN, MOTOR_SCL_PIN);
const uint8_t AS5600_ADDR = 0x36;

// *** CRITICAL: UPDATE THESE FOR YOUR HARDWARE ***
const float PENDULUM_ZERO = 63.63f; // Reading when pendulum is perfectly vertical
const float MOTOR_ZERO = 1.93f;     // Reading when arm is centered

// --- OBJECTS ---
FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;

// --- STATE VARIABLES ---
float pendAngle = 0.0f;
float pendVelocity = 0.0f;
float lastPendAngle = 0.0f;
float integralError = 0.0f;

float motorAngle = 0.0f;
float motorVelocity = 0.0f;
float lastMotorAngle = 0.0f;

String serialBuffer;

// ============================================================
// HELPER FUNCTIONS
// ============================================================

float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float normalizeDelta(float angleNew, float angleOld) {
  float diff = angleNew - angleOld;
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

uint16_t readMotorRaw() {
  motorWire.beginTransmission(AS5600_ADDR);
  motorWire.write(0x0C);
  if (motorWire.endTransmission() != 0) return 0xFFFF;
  motorWire.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (motorWire.available() >= 2) {
    uint16_t h = motorWire.read();
    uint16_t l = motorWire.read();
    return (h << 8) | l;
  }
  return 0xFFFF;
}

// ============================================================
// SENSOR UPDATE & CONTROL LOGIC
// ============================================================

void updateSensors(bool readMotor) {
  // --- 1. Pendulum (Hardware I2C) ---
  uint16_t pRaw = pendulumSensor.readAngle();
  float pDeg = (pRaw * 360.0f) / 4096.0f - PENDULUM_ZERO;
  float currentPendAngle = normalizeAngle(pDeg);

  // FIXED: Calculate velocity using current vs last, THEN update last
  float rawVel = normalizeDelta(currentPendAngle, lastPendAngle) / DT_FIXED;
  
  // LPF: Weighted slightly towards new data for faster reaction
  pendVelocity = 0.7f * rawVel + 0.3f * pendVelocity;
  
  // Update state
  pendAngle = currentPendAngle;
  lastPendAngle = currentPendAngle;

  // --- 2. Motor (Software I2C - Interleaved) ---
  if (readMotor) {
    uint16_t mRaw = readMotorRaw();
    if (mRaw != 0xFFFF) {
      float mDeg = (mRaw * 360.0f) / 4096.0f - MOTOR_ZERO;
      float currentMotorAngle = normalizeAngle(mDeg);
      
      // Damping velocity for base (dt is 10x larger due to interleaving)
      float mVelRaw = normalizeDelta(currentMotorAngle, lastMotorAngle) / (10.0f * DT_FIXED);
      
      motorVelocity = 0.7f * mVelRaw + 0.3f * motorVelocity;
      
      motorAngle = currentMotorAngle;
      lastMotorAngle = currentMotorAngle;
    }
  }
}

void setMotorVelocity(float targetHz) {
  const float MAX_HZ = 25000.0f;
  if (targetHz > MAX_HZ) targetHz = MAX_HZ;
  else if (targetHz < -MAX_HZ) targetHz = -MAX_HZ;

  // FIXED: Dead zone reduced to 10Hz (~2.25 deg/sec)
  if (fabs(targetHz) < 10.0f) {
    stepper->setSpeedInHz(0);
    stepper->stopMove();
    return;
  }

  stepper->setSpeedInHz(fabs(targetHz));
  if (targetHz > 0) stepper->runForward();
  else stepper->runBackward();
}

void runBalanceControl() {
  // Safety Checks
  if (fabs(motorAngle) > BASE_LIMIT_DEG) {
    balanceEnabled = false;
    stepper->forceStopAndNewPosition(0);
    digitalWrite(EN_PIN, HIGH);
    Serial.println(F("⚠ Base limit hit!"));
    return;
  }

  if (fabs(pendAngle) > PEND_LIMIT_DEG) {
    balanceEnabled = false;
    stepper->forceStopAndNewPosition(0);
    digitalWrite(EN_PIN, HIGH);
    Serial.println(F("⚠ Pendulum fell!"));
    return;
  }

  // Integral Term (accumulate error to fix steady-state lean)
  integralError += pendAngle * DT_FIXED;
  integralError = constrain(integralError, -2.0f, 2.0f); // ±1° max effect

  // PID Calculation
  float term_P = Kp_balance * pendAngle;
  float term_I = Ki_balance * integralError;
  float term_D = Kd_balance * pendVelocity;
  
  float balanceOutput = term_P + term_I + term_D;

  // Centering Calculation (with Damping)
  // We subtract this because if Base is Right (+), we want to push Left (-)
  float centerOutput = (Kp_center * motorAngle) + (Kd_center * motorVelocity);

  float totalDegPerSec = balanceOutput - centerOutput;
  
  // Convert Degrees/Sec to Steps/Sec (Hz)
  float targetHz = totalDegPerSec / DEG_PER_STEP;
  
  setMotorVelocity(targetHz);
}

// ============================================================
// SERIAL INTERFACE
// ============================================================

void handleCommand(const String &cmd) {
  if (cmd.length() == 0) return;
  char c = toupper(cmd.charAt(0));
  float val = cmd.substring(1).toFloat();
  switch (c) {
    case 'S': // Start
      pendVelocity = 0.0f;
      motorVelocity = 0.0f;
      integralError = 0.0f;
      updateSensors(true); // Prime sensors
      lastPendAngle = pendAngle;
      lastMotorAngle = motorAngle;
      balanceEnabled = true;
      digitalWrite(EN_PIN, LOW);
      Serial.println(F("✓ Balance ON"));
      break;
    case 'X': // Stop
      balanceEnabled = false;
      stepper->forceStop();
      digitalWrite(EN_PIN, HIGH);
      Serial.println(F("✗ Balance OFF"));
      break;
    case 'P': Kp_balance = val; Serial.print(F("Kp=")); Serial.println(val); break;
    case 'I': Ki_balance = val; Serial.print(F("Ki=")); Serial.println(val); break;
    case 'D': Kd_balance = val; Serial.print(F("Kd=")); Serial.println(val); break;
    case 'M': Kp_center = val; Serial.print(F("Kp_C=")); Serial.println(val); break; // 'M' for Motor P
    case 'V': Kd_center = val; Serial.print(F("Kd_C=")); Serial.println(val); break; // 'V' for Motor D
    case 'T': // Telemetry
      Serial.print(F("P:")); Serial.print(pendAngle, 2);
      Serial.print(F(" M:")); Serial.print(motorAngle, 2);
      Serial.print(F(" PVel:")); Serial.println(pendVelocity, 2);
      break;
    default:
      Serial.println(F("Cmds: S(tart) X(stop) P# I# D# M#(Kp_Center) V#(Kd_Center) T(elem)"));
  }
}

// ============================================================
// EMERGENCY STOP
// ============================================================

void emergencyStop() {
  balanceEnabled = false;
  stepper->forceStop();
  digitalWrite(EN_PIN, HIGH);
  Serial.println(F("!!! EMERGENCY STOP !!!"));
}

// ============================================================
// MAIN SETUP & LOOP
// ============================================================

void setup() {
  Serial.begin(115200);
  
  // Init I2C
  Wire.begin();
  Wire.setClock(400000);
  motorWire.begin();
  pendulumSensor.begin();

  // Init Stepper
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); // Disable initially

  // Emergency Stop Button (Pin 2)
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), emergencyStop, RISING);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) {
    Serial.println(F("Stepper init failed!"));
    while (1);
  }
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(EN_PIN, true);
  stepper->setAutoEnable(false); // We control Enable manually
  
  // Max acceleration for instant reaction
  stepper->setAcceleration(100000); 
  stepper->setSpeedInHz(0);

  Serial.println(F("\nRotary Inverted Pendulum V5.3 (Ready)"));
  Serial.println(F("Hold upright and send 'S' to start."));
}

void loop() {
  // 1. Handle Serial Commands
  while (Serial.available() > 0) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (serialBuffer.length() > 0) {
        handleCommand(serialBuffer);
        serialBuffer = "";
      }
    } else if (isPrintable(ch)) {
      serialBuffer += ch;
    }
  }

  // 2. Real-Time Control Loop
  unsigned long now = micros();
  if (now - lastLoopMicros >= LOOP_INTERVAL_US) {
    lastLoopMicros = now; 
    
    loopCounter++;
    // Read Motor only every 10th cycle (20Hz) to save CPU for Stepper
    if (loopCounter >= 10) {
      loopCounter = 0;
      updateSensors(true); 
    } else {
      updateSensors(false); 
    }

    if (balanceEnabled) {
      runBalanceControl();
    }
  }
}