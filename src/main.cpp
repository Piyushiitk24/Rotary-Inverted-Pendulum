#include <avr/interrupt.h>
#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <FastAccelStepper.h>
#include <math.h>
#include <ctype.h>

// ============================================================
// ROTARY INVERTED PENDULUM – V6.9 (NOISE-ROBUST)
// ============================================================
// Architecture: Cascade Control (V6.8) + Dual Filtering (V6.9)
// Features:
// - Unfiltered Position (Fast reaction)
// - Heavily Filtered Velocity (Noise rejection)
// - Deadzones to prevent "hunting" and buzzing
// - Friction Compensation (Stiction Kick)
// ============================================================

// --- PINS ---
#define STEP_PIN 11  
#define DIR_PIN  6
#define EN_PIN   7

// --- I2C MULTIPLEXER ---
#define TCA_ADDR 0x70

void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// --- MECHANICAL CONSTANTS ---
const float DEG_PER_STEP = 0.225f;     
const float PEND_LIMIT_DEG = 45.0f;    
const float BASE_LIMIT_DEG = 90.0f;    // Hard Stop
const float SOFT_LIMIT_DEG = 60.0f;    // Soft Brake

// --- FILTERING & DEADZONES (V6.9) ---
// History Weight: 0.95 means "Trust 95% history, 5% new". Very Smooth.
const float ALPHA_VEL = 0.95f;      
// Position Alpha: 1.0 means "Trust 100% new". No lag.
const float ALPHA_POS = 1.0f;       

const float VEL_DEADZONE = 5.0f;    // Ignore velocity noise < 5 deg/s
const float HZ_DEADZONE  = 15.0f;   // Ignore motor commands < 15 Hz

// --- GAINS (SAFE START) ---
float Kp_balance = 8.0f;    
float Ki_balance = 0.002f;  
float Kd_balance = 0.5f;    // Start low due to heavy filtering

float Kp_center  = 0.05f;   // Gentle setpoint modulation

// --- ANTI-WINDUP ---
const float INTEGRAL_MAX_DEG = 15.0f;  
const float AW_GAIN = 1.0f;            

// --- TIMING ---
const unsigned long LOOP_INTERVAL_US = 5000; // 5ms (200Hz)
const float DT_FIXED = 0.005f;               
bool balanceEnabled = false;
volatile uint8_t controlTickCnt = 0; 
const float MAX_MOTOR_HZ = 25000.0f;

// --- SAFETY ---
unsigned long lastMagnetCheckMs = 0;
const unsigned long MAGNET_CHECK_INTERVAL_MS = 100;

// --- SENSORS ---
AS5600 pendulumSensor(&Wire);
AS5600 motorSensor(&Wire);

ISR(TIMER3_COMPA_vect) {
  controlTickCnt++;
}

// *** CALIBRATION (VERIFY WITH 'T') ***
const float PENDULUM_ZERO = 64.16f; 
const float MOTOR_ZERO = 31.38f;    

// --- OBJECTS ---
FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;

// --- STATE ---
float pendAngle = 0.0f;
float pendVelocity = 0.0f;
float lastPendAngle = 0.0f;
float integralError = 0.0f;

float motorAngle = 0.0f;
float motorVelocity = 0.0f;
float lastMotorAngle = 0.0f;

// Friction Comp State
float lastTargetHz = 0.0f;

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

// ============================================================
// SENSOR UPDATE (DUAL FILTERING)
// ============================================================

void updateSensors() {
  bool checkMagnetsThisCycle = (millis() - lastMagnetCheckMs >= MAGNET_CHECK_INTERVAL_MS);
  if (checkMagnetsThisCycle) lastMagnetCheckMs = millis();

  bool sensorsHealthy = true;

  // --- 1. PENDULUM ---
  tcaSelect(0);
  if (checkMagnetsThisCycle && !pendulumSensor.detectMagnet()) {
    Serial.println(F("⚠ Pendulum magnet lost!"));
    sensorsHealthy = false;
  }
  uint16_t pRaw = pendulumSensor.readAngle();
  float pDeg = (pRaw * 360.0f) / 4096.0f - PENDULUM_ZERO;
  float currentPendAngle = normalizeAngle(pDeg);

  // A. Position Filtering (Light/None)
  // Keeps P-Term responsive
  pendAngle = (ALPHA_POS * currentPendAngle) + ((1.0f - ALPHA_POS) * pendAngle);

  // B. Velocity Filtering (Heavy)
  // Reduces D-Term noise
  float pVelRaw = normalizeDelta(currentPendAngle, lastPendAngle) / DT_FIXED;
  
  // Apply Deadzone to raw velocity BEFORE filtering
  if (fabs(pVelRaw) < VEL_DEADZONE) {
    pVelRaw = 0.0f;
  }

  pendVelocity = (ALPHA_VEL * pendVelocity) + ((1.0f - ALPHA_VEL) * pVelRaw);
  lastPendAngle = currentPendAngle;

  // --- 2. MOTOR ---
  tcaSelect(1);
  if (checkMagnetsThisCycle && !motorSensor.detectMagnet()) {
    Serial.println(F("⚠ Motor magnet lost!"));
    sensorsHealthy = false;
  }
  uint16_t mRaw = motorSensor.readAngle();
  float mDeg = (mRaw * 360.0f) / 4096.0f - MOTOR_ZERO;
  float currentMotorAngle = normalizeAngle(mDeg);
  
  // Motor doesn't need as much filtering, but we apply some for consistency
  float mVelRaw = normalizeDelta(currentMotorAngle, lastMotorAngle) / DT_FIXED;
  motorVelocity = (0.8f * motorVelocity) + (0.2f * mVelRaw); // Moderate smoothing
  
  motorAngle = currentMotorAngle;
  lastMotorAngle = currentMotorAngle;

  if (!sensorsHealthy) {
    balanceEnabled = false;
    stepper->forceStopAndNewPosition(0);
    digitalWrite(EN_PIN, HIGH);
  }
}

// ============================================================
// CONTROL FUNCTIONS
// ============================================================

void setMotorVelocity(float targetHz) {
  const float MAX_HZ = MAX_MOTOR_HZ;
  if (targetHz > MAX_HZ) targetHz = MAX_HZ;
  else if (targetHz < -MAX_HZ) targetHz = -MAX_HZ;

  // 1. Deadzone: Prevent micro-jitter around zero
  if (fabs(targetHz) < HZ_DEADZONE) {
    stepper->setSpeedInHz(0);
    stepper->stopMove();
    lastTargetHz = 0;
    return;
  }

  // 2. Friction Compensation (Stiction Kick)
  // If we change direction, add a small boost to break static friction
  if ((targetHz > 0 && lastTargetHz < 0) || (targetHz < 0 && lastTargetHz > 0)) {
      // Add a 50Hz kick in the new direction
      targetHz += (targetHz > 0) ? 50.0f : -50.0f;
  }
  lastTargetHz = targetHz;

  stepper->setSpeedInHz(fabs(targetHz));
  
  // Direction Logic
  if (targetHz > 0) {
    stepper->runBackward(); 
  } else {
    stepper->runForward();
  }
}

void runBalanceControl() {
  // 1. Safety
  if (fabs(pendAngle) > PEND_LIMIT_DEG) {
    balanceEnabled = false;
    stepper->forceStopAndNewPosition(0);
    digitalWrite(EN_PIN, HIGH);
    Serial.println(F("⚠ Pendulum fell!"));
    return;
  }

  // 2. Cascade Setpoint
  float pendulumSetpoint = 0.0f;
  if (fabs(motorAngle) > 10.0f) {
      pendulumSetpoint = -Kp_center * motorAngle;
      pendulumSetpoint = constrain(pendulumSetpoint, -5.0f, 5.0f);
  }

  // 3. PID
  float error = pendAngle - pendulumSetpoint;

  integralError += error * DT_FIXED;
  integralError = constrain(integralError, -INTEGRAL_MAX_DEG, INTEGRAL_MAX_DEG);

  // Derivative is on Velocity (Measurement), not Error (Setpoint)
  float balanceOutput = (Kp_balance * error) + (Ki_balance * integralError) + (Kd_balance * (-pendVelocity));

  float targetHz = balanceOutput / DEG_PER_STEP;

  // 4. Anti-Windup (Back Calc)
  float saturatedHz = constrain(targetHz, -MAX_MOTOR_HZ, MAX_MOTOR_HZ);
  if (fabs(targetHz) > MAX_MOTOR_HZ) {
      integralError -= AW_GAIN * (targetHz - saturatedHz) * DT_FIXED;
  }
  targetHz = saturatedHz;

  // 5. Soft Limits (Exponential)
  if (fabs(motorAngle) > SOFT_LIMIT_DEG) {
      float distanceToWall = BASE_LIMIT_DEG - fabs(motorAngle);
      // Exp braking
      float brakeFactor = exp(-3.0f * (fabs(motorAngle) - SOFT_LIMIT_DEG) / (BASE_LIMIT_DEG - SOFT_LIMIT_DEG));
      brakeFactor = constrain(brakeFactor, 0.05f, 1.0f);
      targetHz *= brakeFactor;
  }

  // 6. Hard Limit
  if (fabs(motorAngle) > BASE_LIMIT_DEG) {
     balanceEnabled = false;
     stepper->forceStopAndNewPosition(0);
     digitalWrite(EN_PIN, HIGH);
     Serial.print(F("⚠ Base limit hit: ")); 
     Serial.println(motorAngle);
     return;
  }

  setMotorVelocity(targetHz);
}

// ============================================================
// SERIAL & SETUP
// ============================================================

void handleCommand(const String &cmd) {
  if (cmd.length() == 0) return;
  char c = toupper(cmd.charAt(0));
  float val = cmd.substring(1).toFloat();
  switch (c) {
    case 'S': 
      pendVelocity = 0.0f; motorVelocity = 0.0f; integralError = 0.0f;
      updateSensors(); 
      lastPendAngle = pendAngle; lastMotorAngle = motorAngle;
      balanceEnabled = true;
      digitalWrite(EN_PIN, LOW);
      Serial.println(F("✓ Balance ON"));
      break;
    case 'X': 
      balanceEnabled = false;
      stepper->forceStop();
      digitalWrite(EN_PIN, HIGH);
      Serial.println(F("✗ Balance OFF"));
      break;
    case 'P': Kp_balance = val; Serial.print(F("Kp=")); Serial.println(val); break;
    case 'I': Ki_balance = val; Serial.print(F("Ki=")); Serial.println(val); break;
    case 'D': Kd_balance = val; Serial.print(F("Kd=")); Serial.println(val); break;
    case 'M': Kp_center = val; Serial.print(F("Kp_C=")); Serial.println(val); break;
    case 'T': 
      {
        float pendulumSetpoint = 0.0f;
        if (fabs(motorAngle) > 10.0f) {
             pendulumSetpoint = constrain(-Kp_center * motorAngle, -5.0f, 5.0f);
        }
        float error = pendAngle - pendulumSetpoint;
        float outputHz = (Kp_balance * error + Ki_balance * integralError + Kd_balance * (-pendVelocity)) / DEG_PER_STEP;
        
        Serial.print(F("P:")); Serial.print(pendAngle, 2);
        Serial.print(F(" M:")); Serial.print(motorAngle, 2);
        Serial.print(F(" Err:")); Serial.print(error, 2);
        Serial.print(F(" Vel:")); Serial.print(pendVelocity, 2); // Debug filtered velocity
        Serial.print(F(" Hz:")); Serial.println(outputHz, 0);
      }
      break;
    default: Serial.println(F("Cmds: S, X, P#, I#, D#, M#, T"));
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  #if defined(WIRE_HAS_ENDTRANSMISSION_TIMEOUT)
    Wire.setWireTimeout(5000, true);
  #endif
  
  Serial.println(F("Init Sensors..."));
  
  tcaSelect(0);
  pendulumSensor.begin();
  delay(10);
  if (!pendulumSensor.detectMagnet()) {
    Serial.println(F("⚠ ERROR: Pendulum Magnet (Ch0) NOT detected!"));
    while(1); 
  }
  
  tcaSelect(1);
  motorSensor.begin();
  delay(10);
  if (!motorSensor.detectMagnet()) {
    Serial.println(F("⚠ ERROR: Motor Magnet (Ch1) NOT detected!"));
    while(1); 
  }
  
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); 

  if (TCCR1A != 0 || TCCR1B != 0 || TIMSK1 != 0) {
    TCCR1A = 0; TCCR1B = 0; TIMSK1 = 0; TCNT1  = 0;
  }

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) { Serial.println(F("Stepper init failed! Check Pin 11.")); while (1); }
  
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(EN_PIN, true);
  stepper->setAutoEnable(false);
  stepper->setAcceleration(100000); 
  stepper->setSpeedInHz(0);

  noInterrupts();
  TCCR3A = 0; TCCR3B = 0; TCNT3 = 0;
  TCCR3B |= (1 << WGM32); 
  TCCR3B |= (1 << CS31) | (1 << CS30); 
  OCR3A = 1249; 
  TIMSK3 |= (1 << OCIE3A);
  interrupts();

  Serial.println(F("\nRotary Inverted Pendulum V6.9 (Noise-Robust)"));
  Serial.println(F("Sensors OK. Hold upright, Check 'T', Send 'S'."));
}

void loop() {
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

  uint8_t ticksToProcess;
  noInterrupts();
  ticksToProcess = controlTickCnt;
  controlTickCnt = 0;
  interrupts();

  if (ticksToProcess > 0) {
    unsigned long loopStart = micros();
    while (ticksToProcess--) {
      updateSensors();
      if (balanceEnabled) {
          runBalanceControl();
      } 
    }
    unsigned long loopDuration = micros() - loopStart;
    if (loopDuration > LOOP_INTERVAL_US) {
      Serial.print(F("⚠ Control loop overrun (us): "));
      Serial.println(loopDuration);
    }
  }
}