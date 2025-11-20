#include <avr/interrupt.h>
#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <FastAccelStepper.h>
#include <math.h>
#include <ctype.h>

// ============================================================
// ROTARY INVERTED PENDULUM – V6.1 (FINAL MULTIPLEXER)
// ============================================================
// Hardware:
// - TCA9548A Multiplexer (0x70)
// - Pendulum Sensor: Channel 0 (SD0/SC0)
// - Motor Sensor:    Channel 1 (SD1/SC1)
// - Stepper:         Pin 11 (Timer 1)
//
// Fixes Applied:
// - I2C Clock set to 100kHz for robustness (use external pull-ups for 400kHz)
// - Magnet Presence Detection on Startup
// - Corrected Base Limits (35 deg)
// - Extended Telemetry
// ============================================================

// --- PINS ---
#define STEP_PIN 11  // Timer 1 (Required for FastAccelStepper on Mega)
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

// Timer3 Compare Match A ISR: increment control tick counter
ISR(TIMER3_COMPA_vect) {
  if (controlTickCnt < 250) controlTickCnt++; // prevent roll-over
}

// --- MECHANICAL CONSTANTS ---
const float DEG_PER_STEP = 0.225f;     // 1.8 deg / 8 microsteps
const float PEND_LIMIT_DEG = 45.0f;    // Fall limit
const float BASE_LIMIT_DEG = 35.0f;    // FIX 3: Reduced to realistic mechanical limit

// --- GAINS ---
float Kp_balance = 80.0f;   
float Ki_balance = 0.5f;    
float Kd_balance = 8.0f;    

float Kp_center  = 2.0f;    
float Kd_center  = 0.5f;    

// --- TIMING ---
const unsigned long LOOP_INTERVAL_US = 5000; // 5ms (200Hz)
const float DT_FIXED = 0.005f;               
unsigned long lastLoopMicros = 0;
bool balanceEnabled = false;
volatile uint8_t controlTickCnt = 0; // incremented by timer ISR
const float MAX_MOTOR_HZ = 25000.0f;
const float ALPHA_VEL = 0.7f; // velocity EMA coefficient
const float ALPHA_VEL = 0.7f; // velocity EMA coefficient

// --- SENSORS ---
// Use distinct AS5600 objects for each physical sensor. Sharing a single
// AS5600 instance across channels can cause state/cached register confusion.
AS5600 pendulumSensor(&Wire);
AS5600 motorSensor(&Wire);

// *** CALIBRATION REQUIRED ***
// Run 'T' command first to find these values!
const float PENDULUM_ZERO = 63.63f; 
const float MOTOR_ZERO = 1.93f;     

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
// SENSOR UPDATE
// ============================================================

void updateSensors() {
  // --- 1. PENDULUM (Channel 0) ---
  tcaSelect(0);
  uint16_t pRaw = pendulumSensor.readAngle();
  float pDeg = (pRaw * 360.0f) / 4096.0f - PENDULUM_ZERO;
  float currentPendAngle = normalizeAngle(pDeg);

  // Velocity
  float pVelRaw = normalizeDelta(currentPendAngle, lastPendAngle) / DT_FIXED;
  pendVelocity = ALPHA_VEL * pVelRaw + (1.0f - ALPHA_VEL) * pendVelocity;
  
  pendAngle = currentPendAngle;
  lastPendAngle = currentPendAngle;

  // --- 2. MOTOR (Channel 1) ---
  tcaSelect(1);
  uint16_t mRaw = motorSensor.readAngle();
  float mDeg = (mRaw * 360.0f) / 4096.0f - MOTOR_ZERO;
  float currentMotorAngle = normalizeAngle(mDeg);

  // Velocity
  float mVelRaw = normalizeDelta(currentMotorAngle, lastMotorAngle) / DT_FIXED;
  motorVelocity = ALPHA_VEL * mVelRaw + (1.0f - ALPHA_VEL) * motorVelocity;

  motorAngle = currentMotorAngle;
  lastMotorAngle = currentMotorAngle;
}

// ============================================================
// CONTROL LOOP
// ============================================================

void setMotorVelocity(float targetHz) {
  const float MAX_HZ = MAX_MOTOR_HZ;
  if (targetHz > MAX_HZ) targetHz = MAX_HZ;
  else if (targetHz < -MAX_HZ) targetHz = -MAX_HZ;

  // Dead zone 10Hz
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
  // Runtime magnet presence check for pendulum & motor
  tcaSelect(0);
  if (!pendulumSensor.detectMagnet()) {
    Serial.println(F("⚠ Pendulum magnet lost! Stopping balance."));
    balanceEnabled = false;
    stepper->forceStopAndNewPosition(0);
    digitalWrite(EN_PIN, HIGH);
    return;
  }
  // Check Base Limits (Reduced to 35 deg)
  if (fabs(motorAngle) > BASE_LIMIT_DEG) {
    balanceEnabled = false;
    stepper->forceStopAndNewPosition(0);
    digitalWrite(EN_PIN, HIGH);
    Serial.print(F("⚠ Base limit hit: ")); 
    Serial.println(motorAngle);
    return;
  }

  // Check Pendulum Limits
  if (fabs(pendAngle) > PEND_LIMIT_DEG) {
    balanceEnabled = false;
    stepper->forceStopAndNewPosition(0);
    digitalWrite(EN_PIN, HIGH);
    Serial.println(F("⚠ Pendulum fell!"));
    return;
  }

  integralError += pendAngle * DT_FIXED;
  // Anti-windup: scale to motor torque capability if Ki non-zero
  float maxIntegral = (Ki_balance != 0.0f) ? ((MAX_MOTOR_HZ * DEG_PER_STEP) / Ki_balance) : 10.0f;
  integralError = constrain(integralError, -maxIntegral, maxIntegral);

  float balanceOutput = (Kp_balance * pendAngle) + (Ki_balance * integralError) + (Kd_balance * pendVelocity);
  
  // Centering: Subtract because Positive Angle -> Needs Negative Force
  float centerOutput = (Kp_center * motorAngle) + (Kd_center * motorVelocity);

  float targetHz = (balanceOutput - centerOutput) / DEG_PER_STEP;
  
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
    case 'V': Kd_center = val; Serial.print(F("Kd_C=")); Serial.println(val); break;
    case 'T': // FIX 5: Enhanced Telemetry
      {
        float outputHz = (Kp_balance*pendAngle + Ki_balance*integralError + Kd_balance*pendVelocity - (Kp_center*motorAngle + Kd_center*motorVelocity)) / DEG_PER_STEP;
        Serial.print(F("P:")); Serial.print(pendAngle, 2);
        Serial.print(F(" M:")); Serial.print(motorAngle, 2);
        Serial.print(F(" PV:")); Serial.print(pendVelocity, 2);
        Serial.print(F(" MV:")); Serial.print(motorVelocity, 2);
        Serial.print(F(" Hz:")); Serial.println(outputHz, 0);
      }
      break;
    default: Serial.println(F("Cmds: S, X, P#, I#, D#, M#, V#, T"));
  }
}

void setup() {
  Serial.begin(115200);
  
  // FIX 1: Set I2C Clock BEFORE sensor init
  Wire.begin();
  // Use 100kHz default for better EMI tolerance; 400kHz requires proper external pull-ups
  Wire.setClock(100000);
  // Protect against bus hangs: set a timeout (in microseconds or 0 meaning default for platform)
  #if defined(WIRE_HAS_ENDTRANSMISSION_TIMEOUT)
    Wire.setWireTimeout(5000, true);
  #endif
  
  // FIX 2 & 4: Select Channel, Check Magnet, Init
  Serial.println(F("Init Sensors..."));
  
  // Channel 0: Pendulum
  tcaSelect(0);
  pendulumSensor.begin();
  delay(10);
  if (!pendulumSensor.detectMagnet()) {
    Serial.println(F("⚠ ERROR: Pendulum Magnet (Ch0) NOT detected!"));
    while(1); // Halt
  }
  
  // Channel 1: Motor
  tcaSelect(1);
  motorSensor.begin();
  delay(10);
  if (!motorSensor.detectMagnet()) {
    Serial.println(F("⚠ ERROR: Motor Magnet (Ch1) NOT detected!"));
    while(1); // Halt
  }
  
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); 

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) { Serial.println(F("Stepper init failed! Check Pin 11.")); while (1); }
  
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(EN_PIN, true);
  stepper->setAutoEnable(false);
  stepper->setAcceleration(100000); 
  stepper->setSpeedInHz(0);

  Serial.println(F("\nRotary Inverted Pendulum V6.1 (Multiplexer)"));
  Serial.println(F("Sensors OK. Hold upright, Check 'T', Send 'S'."));
  // Initialize Timer3 for 5ms ticks (200Hz) — do not call heavy functions in ISR
  noInterrupts();
  TCCR3A = 0;              // CTC mode
  TCCR3B = 0;
  TCNT3 = 0;              // reset counter
  // CTC mode with OCR3A as top
  TCCR3B |= (1 << WGM32);
  // Prescaler 64
  TCCR3B |= (1 << CS31) | (1 << CS30);
  OCR3A = 1249; // 16MHz/64 = 250k ticks/sec -> 250k * 0.005 = 1250 -> OCR = 1249
  TIMSK3 |= (1 << OCIE3A); // enable compare interrupt
  interrupts();
  // Initialize control loop timing
  lastLoopMicros = micros();
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

  // Allow Serial input to be handled rapidly; process control ticks thereafter

  // FIX 6: Timer Overflow Safe Logic
  unsigned long loopStart = micros();

  // Process pending fixed-rate control ticks set by Timer3 ISR
  while (controlTickCnt > 0) {
    noInterrupts();
    if (controlTickCnt > 0) controlTickCnt--; // consume 1 tick
    interrupts();
    updateSensors();
    if (balanceEnabled) runBalanceControl();
  }

  unsigned long loopDuration = micros() - loopStart;
  if (loopDuration > LOOP_INTERVAL_US) {
    Serial.print(F("⚠ Loop overrun (us): "));
    Serial.println(loopDuration);
  }
}