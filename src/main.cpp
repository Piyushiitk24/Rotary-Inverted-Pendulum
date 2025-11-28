#include <avr/interrupt.h>
#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <FastAccelStepper.h>
#include <math.h>
#include <ctype.h>

// ============================================================
// ROTARY INVERTED PENDULUM – V7.3 (DIAGNOSTIC / LATENCY FIX)
// ============================================================
// CHANGES:
// - Added P-only mode for testing (D=0)
// - Added direct manual step test
// - Reduced loop to 2ms (500Hz) with single sensor read
// - Option to skip motor sensor (use stepper position instead)
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
const float DEG_PER_STEP   = 0.225f;
const float PEND_LIMIT_DEG = 45.0f;
const float BASE_LIMIT_DEG = 90.0f;
const float SOFT_LIMIT_DEG = 60.0f;

// --- FILTERING ---
const float ALPHA_VEL = 0.9f;   // Very fast velocity response (0.9 = 90% new)
const float ALPHA_POS = 1.0f;   // No position filtering

// --- GAINS ---
// INCREASED SIGNIFICANTLY - the push test showed we need more authority
float Kp_balance = 100.0f;      // Was 25 - now 6x higher
float Ki_balance = 0.0f;
float Kd_balance = 40.0f;        // Add some damping back
float Kp_center  = 0.0f;

// --- TIMING ---
// 4ms loop (250 Hz) - read only pendulum sensor each tick
const unsigned long LOOP_INTERVAL_US = 4000;
const float DT_FIXED = 0.004f;

volatile uint8_t controlTickCnt = 0;
bool balanceEnabled = false;

const float MAX_MOTOR_HZ = 4000.0f;  // Allow higher speeds

// --- SAFETY ---
unsigned long lastMagnetCheckMs = 0;
const unsigned long MAGNET_CHECK_INTERVAL_MS = 500; // Less frequent
volatile bool magnetCheckPending = false;

// --- TELEMETRY ---
const bool TELEMETRY_ENABLED = false;
unsigned long lastTelemetryMs = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 50;

// --- SENSORS ---
AS5600 pendulumSensor(&Wire);
AS5600 motorSensor(&Wire);

// --- STEPPER ---
FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;

// --- STATE ---
float pendAngle      = 0.0f;
float pendVelocity   = 0.0f;
float lastPendAngle  = 0.0f;
float integralError  = 0.0f;

float motorAngle     = 0.0f;
float motorVelocity  = 0.0f;
float lastMotorAngle = 0.0f;

float lastTargetHz   = 0.0f;
float lastCommandHz  = 0.0f;

bool sensorsPresent  = true;

// NEW: Option to use stepper position instead of motor sensor
bool useStepperPosition = true;  // Skip motor I2C read for speed

String serialBuffer;

// --- CALIBRATION ---
const float PENDULUM_ZERO_DEFAULT = 64.34f;
const float MOTOR_ZERO_DEFAULT    = 274.834f;
float pendulumZeroDeg = PENDULUM_ZERO_DEFAULT;
float motorZeroDeg    = MOTOR_ZERO_DEFAULT;

// ============================================================
// TIMER ISR
// ============================================================
ISR(TIMER3_COMPA_vect) {
  controlTickCnt++;
}

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
// SENSOR UPDATE - OPTIMIZED
// ============================================================
void updateSensors() {
  // --- PENDULUM ONLY (fast path) ---
  tcaSelect(0);
  
  // Skip magnet check most of the time
  bool doMagnetCheck = false;
  noInterrupts();
  doMagnetCheck = magnetCheckPending;
  magnetCheckPending = false;
  interrupts();
  
  if (doMagnetCheck && !pendulumSensor.detectMagnet()) {
    balanceEnabled = false;
    stepper->forceStopAndNewPosition(0);
    digitalWrite(EN_PIN, HIGH);
    Serial.println(F("Magnet lost!"));
    return;
  }

  uint16_t pRaw = pendulumSensor.readAngle();
  float pDegRaw = (pRaw * 360.0f) / 4096.0f;
  float pDeg = pDegRaw - pendulumZeroDeg;
  float currentPendAngle = normalizeAngle(pDeg);

  // No position filtering
  pendAngle = currentPendAngle;

  // Velocity with fast filter
  float pVelRaw = normalizeDelta(currentPendAngle, lastPendAngle) / DT_FIXED;
  pendVelocity = (ALPHA_VEL * pVelRaw) + ((1.0f - ALPHA_VEL) * pendVelocity);
  lastPendAngle = currentPendAngle;

  // --- MOTOR: Use stepper position (no I2C delay) ---
  if (useStepperPosition) {
    // Convert stepper steps to degrees
    long steps = stepper->getCurrentPosition();
    motorAngle = steps * DEG_PER_STEP;
    motorAngle = normalizeAngle(motorAngle);
    
    float mVelRaw = normalizeDelta(motorAngle, lastMotorAngle) / DT_FIXED;
    motorVelocity = (0.3f * mVelRaw) + (0.7f * motorVelocity);
    lastMotorAngle = motorAngle;
  } else {
    // Original motor sensor read (slower)
    tcaSelect(1);
    uint16_t mRaw = motorSensor.readAngle();
    float mDegRaw = (mRaw * 360.0f) / 4096.0f;
    float mDeg = -(mDegRaw - motorZeroDeg);
    float currentMotorAngle = normalizeAngle(mDeg);
    
    float mVelRaw = normalizeDelta(currentMotorAngle, lastMotorAngle) / DT_FIXED;
    motorVelocity = (0.2f * mVelRaw) + (0.8f * motorVelocity);
    motorAngle = currentMotorAngle;
    lastMotorAngle = currentMotorAngle;
  }
}

// ============================================================
// MOTOR COMMAND - SIMPLIFIED
// ============================================================
void setMotorVelocity(float targetHz) {
  // Saturate
  if (targetHz > MAX_MOTOR_HZ) targetHz = MAX_MOTOR_HZ;
  if (targetHz < -MAX_MOTOR_HZ) targetHz = -MAX_MOTOR_HZ;

  lastTargetHz = targetHz;
  lastCommandHz = targetHz;

  // REMOVED deadzone - we need response even at small angles
  if (fabs(targetHz) < 10.0f) {
    stepper->stopMove();
    return;
  }

  stepper->setSpeedInHz((uint32_t)fabs(targetHz));

  if (targetHz > 0) {
    stepper->runBackward();
  } else {
    stepper->runForward();
  }
}

// ============================================================
// BALANCE CONTROL - SIMPLIFIED
// ============================================================
void runBalanceControl() {
  // Safety check
  if (fabs(pendAngle) > PEND_LIMIT_DEG) {
    balanceEnabled = false;
    stepper->forceStopAndNewPosition(0);
    digitalWrite(EN_PIN, HIGH);
    Serial.println(F("Pendulum fell!"));
    return;
  }

  // Simple PD control - no setpoint adjustment for now
  float error = pendAngle;  // Target is 0 (upright)

  // Clamp velocity for D-term
  float velForD = constrain(pendVelocity, -150.0f, 150.0f);

  // PD output
  float targetHz = (Kp_balance * error) - (Kd_balance * velForD);

  // Base limit safety (simplified)
  if (fabs(motorAngle) > BASE_LIMIT_DEG) {
    balanceEnabled = false;
    stepper->forceStopAndNewPosition(0);
    digitalWrite(EN_PIN, HIGH);
    Serial.println(F("Base limit!"));
    return;
  }

  setMotorVelocity(targetHz);
}

// ============================================================
// COMMAND HANDLER
// ============================================================
void handleCommand(const String &cmd) {
  if (cmd.length() == 0) return;
  char c = toupper(cmd.charAt(0));
  float val = cmd.substring(1).toFloat();

  switch (c) {
    case 'S': {
      pendVelocity = 0.0f;
      motorVelocity = 0.0f;
      integralError = 0.0f;
      
      // Reset stepper position to 0 at start
      stepper->forceStopAndNewPosition(0);
      
      updateSensors();
      lastPendAngle = pendAngle;
      lastMotorAngle = 0.0f;

      balanceEnabled = true;
      digitalWrite(EN_PIN, LOW);
      
      Serial.print(F("Balance ON. Kp=")); Serial.print(Kp_balance);
      Serial.print(F(" Kd=")); Serial.println(Kd_balance);
      break;
    }

    case 'X': {
      balanceEnabled = false;
      stepper->forceStop();
      digitalWrite(EN_PIN, HIGH);
      Serial.println(F("Balance OFF"));
      break;
    }

    case 'P':
      Kp_balance = val;
      Serial.print(F("Kp=")); Serial.println(val);
      break;

    case 'D':
      Kd_balance = val;
      Serial.print(F("Kd=")); Serial.println(val);
      break;

    case 'Y': {
      tcaSelect(0);
      uint16_t pRaw = pendulumSensor.readAngle();
      pendulumZeroDeg = (pRaw * 360.0f) / 4096.0f;
      Serial.print(F("Pendulum zero=")); Serial.println(pendulumZeroDeg);
      break;
    }

    case 'Z': {
      tcaSelect(1);
      uint16_t mRaw = motorSensor.readAngle();
      motorZeroDeg = (mRaw * 360.0f) / 4096.0f;
      Serial.print(F("Motor zero=")); Serial.println(motorZeroDeg);
      break;
    }

    case 'T': {
      updateSensors();
      float hz = (Kp_balance * pendAngle) - (Kd_balance * pendVelocity);
      Serial.print(F("P:")); Serial.print(pendAngle, 1);
      Serial.print(F(" V:")); Serial.print(pendVelocity, 0);
      Serial.print(F(" Hz:")); Serial.println(hz, 0);
      break;
    }

    // NEW: Manual motor test
    case 'L': {  // Test motor left
      Serial.println(F("Motor LEFT 1000Hz for 500ms"));
      digitalWrite(EN_PIN, LOW);
      stepper->setSpeedInHz(1000);
      stepper->runForward();
      delay(500);
      stepper->forceStop();
      digitalWrite(EN_PIN, HIGH);
      Serial.println(F("Done"));
      break;
    }

    case 'R': {  // Test motor right
      Serial.println(F("Motor RIGHT 1000Hz for 500ms"));
      digitalWrite(EN_PIN, LOW);
      stepper->setSpeedInHz(1000);
      stepper->runBackward();
      delay(500);
      stepper->forceStop();
      digitalWrite(EN_PIN, HIGH);
      Serial.println(F("Done"));
      break;
    }

    // NEW: Test P-only control (no D-term)
    case 'Q': {
      Kd_balance = 0.0f;
      Serial.println(F("D-term disabled (Kd=0). P-only mode."));
      break;
    }

    // NEW: Push test - verify direction mapping
    case 'W': {
      Serial.println(F("PUSH TEST: Tilt pendulum and watch motor direction"));
      Serial.println(F("Pendulum RIGHT (+angle) -> Motor should go RIGHT (under pendulum)"));
      Serial.println(F("Press X to stop"));
      
      pendVelocity = 0.0f;
      stepper->forceStopAndNewPosition(0);
      balanceEnabled = true;
      digitalWrite(EN_PIN, LOW);
      
      // Temporarily disable D-term for clearer direction test
      float savedKd = Kd_balance;
      Kd_balance = 0.0f;
      
      unsigned long start = millis();
      while (millis() - start < 10000) {  // 10 second test
        updateSensors();
        float hz = Kp_balance * pendAngle;
        setMotorVelocity(hz);
        
        // Print every 200ms
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 200) {
          Serial.print(F("Angle:")); Serial.print(pendAngle, 1);
          Serial.print(F(" -> Hz:")); Serial.println(hz, 0);
          lastPrint = millis();
        }
        
        // Check for X command
        if (Serial.available() && toupper(Serial.read()) == 'X') break;
      }
      
      Kd_balance = savedKd;
      balanceEnabled = false;
      stepper->forceStop();
      digitalWrite(EN_PIN, HIGH);
      Serial.println(F("Push test done"));
      break;
    }

    default:
      Serial.println(F("S:Start X:Stop P#:Kp D#:Kd Y:Zero-pend Z:Zero-mot T:Diag L/R:MotorTest W:PushTest Q:P-only"));
      break;
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(500000);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("V7.3 Diagnostic"));

  // Init pendulum sensor
  tcaSelect(0);
  pendulumSensor.begin();
  delay(10);
  if (!pendulumSensor.detectMagnet()) {
    Serial.println(F("WARNING: Pendulum magnet not detected!"));
  }

  // Init motor sensor (even if we don't use it)
  tcaSelect(1);
  motorSensor.begin();
  delay(10);
  if (!motorSensor.detectMagnet()) {
    Serial.println(F("Note: Motor sensor not detected (using stepper position)"));
  }

  // Stepper
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) {
    Serial.println(F("Stepper init failed!"));
    while(1);
  }

  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(EN_PIN, true);
  stepper->setAutoEnable(false);
  
  // CRITICAL: High but not excessive acceleration
  // Too high = motor stalls with "tick"
  // Too low = delayed response
  stepper->setAcceleration(50000);  // 50k - balanced choice
  stepper->setSpeedInHz(0);

  // Timer3: 4ms (250 Hz)
  noInterrupts();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS31) | (1 << CS30);
  OCR3A = 999;  // 4ms
  TIMSK3 |= (1 << OCIE3A);
  interrupts();

  Serial.println(F("Ready. Commands: S X P# D# Y Z T L R W Q"));
  Serial.println(F("1) Do Y (zero pendulum upright)"));
  Serial.println(F("2) Do W (push test) to verify direction"));
  Serial.println(F("3) Do S to start balance"));
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  // Magnet check (infrequent)
  if (millis() - lastMagnetCheckMs >= MAGNET_CHECK_INTERVAL_MS) {
    lastMagnetCheckMs = millis();
    magnetCheckPending = true;
  }

  // Serial
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

  // Control - run once per timer tick, discard backlog
  noInterrupts();
  uint8_t ticks = controlTickCnt;
  controlTickCnt = 0;
  interrupts();

  if (ticks > 0) {
    updateSensors();
    if (balanceEnabled) {
      runBalanceControl();
    }
  }
}