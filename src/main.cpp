#include <avr/interrupt.h>
#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <FastAccelStepper.h>
#include <math.h>
#include <ctype.h>

// ============================================================
// ROTARY INVERTED PENDULUM – V7.1 (CALIBRATABLE ZEROS)
// ============================================================
// - Pendulum & Motor zeros can be calibrated at runtime:
//     Y -> set current pendulum angle as 0° (upright)
//     Z -> set current motor angle as 0° (base center)
// - Direction convention kept consistent with your current
//   observations (pendulum right -> base moves under it).
// - Angle wrapping and sign paths carefully checked.
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
const float DEG_PER_STEP   = 0.225f;   // assumed; should be close
const float PEND_LIMIT_DEG = 45.0f;    // pendulum safety shutoff
const float BASE_LIMIT_DEG = 90.0f;    // hard base limit (mechanical stop)
const float SOFT_LIMIT_DEG = 60.0f;    // soft brake onset

// --- FILTERING & DEADZONES ---
const float ALPHA_VEL = 0.2f;          // velocity low-pass (reduced lag)
const float ALPHA_POS = 1.0f;          // no smoothing on angle

const float VEL_DEADZONE = 0.0f;       // unused for now
const float HZ_DEADZONE  = 0.0f;       // deadzone on motor Hz (0 for tuning)

// --- GAINS ---
// IMPORTANT: these are *incremental* gains: ΔHz per tick per deg / (deg/s)
// From model: ΔHz_k = Kp_balance*α_deg + Kd_balance*αdot_deg_s
float Kp_balance = 5.74f;              // ΔHz / deg
float Ki_balance = 0.0f;               // integral OFF (not used in control)
float Kd_balance = 0.68f;              // ΔHz / (deg/s)

float Kp_center  = 0.0f;               // base-centering coupling (small)

// --- ANTI-WINDUP ---
// Kept for future use if Ki_balance is used; not active now.
const float INTEGRAL_MAX_DEG = 15.0f;
const float AW_GAIN          = 1.0f;

// D-term velocity clamp (deg/s) — prevents noisy velocity spikes from
// causing excessive derivative action.
const float D_VEL_CLAMP = 200.0f;   // deg/s, tweak later

// --- TIMING ---
const unsigned long LOOP_INTERVAL_US = 8000; // 8ms -> 125 Hz
const float DT_FIXED = 0.008f;

volatile uint8_t controlTickCnt = 0;
bool balanceEnabled = false;

const float MAX_MOTOR_HZ = 5000.0f;

// --- SAFETY ---
unsigned long lastMagnetCheckMs = 0;
const unsigned long MAGNET_CHECK_INTERVAL_MS = 100;
volatile bool magnetCheckPending = false;

// --- TELEMETRY ---
unsigned long lastTelemetryMs = 0;
// At 500k baud and ~80chars/line, 8ms (~125Hz) is tight but acceptable.
const unsigned long TELEMETRY_INTERVAL_MS = 8; // ~125Hz

// --- SENSORS ---
AS5600 pendulumSensor(&Wire);
AS5600 motorSensor(&Wire);

// --- STEPPER OBJECTS (GLOBAL) ---
FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;

// --- STATE ---
float pendAngle      = 0.0f;
float pendVelocity   = 0.0f;
float lastPendAngle  = 0.0f;
float integralError  = 0.0f;   // currently unused in control law

float motorAngle     = 0.0f;
float motorVelocity  = 0.0f;
float lastMotorAngle = 0.0f;

float lastTargetHz   = 0.0f;
float lastCommandHz  = 0.0f;

// *** NEW: integrated speed command (Hz) implementing acceleration control ***
float cmdHz          = 0.0f;

bool sensorsPresent  = true;

String serialBuffer;

// ============================================================
// TIMER ISR
// ============================================================

ISR(TIMER3_COMPA_vect) {
  controlTickCnt++;
}

// ============================================================
// CALIBRATION
// ============================================================

// Defaults from your latest tuning / sine test.
const float PENDULUM_ZERO_DEFAULT = 64.34f;   // "up is up"
const float MOTOR_ZERO_DEFAULT    = 274.834f; // base center

// Runtime-calibratable zeros:
float pendulumZeroDeg = PENDULUM_ZERO_DEFAULT;
float motorZeroDeg    = MOTOR_ZERO_DEFAULT;

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
  bool doMagnetCheck = false;
  noInterrupts();
  doMagnetCheck = magnetCheckPending;
  magnetCheckPending = false;
  interrupts();

  bool sensorsHealthy = true;

  // --- 1. PENDULUM ---
  tcaSelect(0);
  if (doMagnetCheck && !pendulumSensor.detectMagnet()) {
    sensorsHealthy = false;
  }
  uint16_t pRaw = pendulumSensor.readAngle();
  float pDegRaw = (pRaw * 360.0f) / 4096.0f;
  float pDeg    = pDegRaw - pendulumZeroDeg;
  float currentPendAngle = normalizeAngle(pDeg);

  pendAngle = (ALPHA_POS * currentPendAngle) +
              ((1.0f - ALPHA_POS) * pendAngle);

  float pVelRaw = normalizeDelta(currentPendAngle, lastPendAngle) / DT_FIXED;
  pendVelocity = (ALPHA_VEL * pendVelocity) +
                 ((1.0f - ALPHA_VEL) * pVelRaw);
  lastPendAngle = currentPendAngle;

  // --- 2. MOTOR ---
  tcaSelect(1);
  if (doMagnetCheck && !motorSensor.detectMagnet()) {
    sensorsHealthy = false;
  }
  uint16_t mRaw = motorSensor.readAngle();
  float mDegRaw = (mRaw * 360.0f) / 4096.0f;
  float mDeg    = -(mDegRaw - motorZeroDeg);
  float currentMotorAngle = normalizeAngle(mDeg);

  float mVelRaw = normalizeDelta(currentMotorAngle, lastMotorAngle) / DT_FIXED;
  motorVelocity = (0.8f * motorVelocity) + (0.2f * mVelRaw);

  motorAngle     = currentMotorAngle;
  lastMotorAngle = currentMotorAngle;

  // --- SENSOR FAILURE HANDLING ---
  if (!sensorsHealthy) {
    balanceEnabled = false;
    stepper->forceStopAndNewPosition(0);
    digitalWrite(EN_PIN, HIGH);
  }
}

// ============================================================
// MOTOR COMMAND
// ============================================================

void setMotorVelocity(float targetHz) {
  // Saturate
  if (targetHz >  MAX_MOTOR_HZ) targetHz =  MAX_MOTOR_HZ;
  if (targetHz < -MAX_MOTOR_HZ) targetHz = -MAX_MOTOR_HZ;

  // Deadzone (can be 0.0 during tuning)
  if (fabs(targetHz) < HZ_DEADZONE) {
    stepper->setSpeedInHz(0);
    stepper->stopMove();
    lastTargetHz   = 0.0f;
    lastCommandHz  = 0.0f;
    return;
  }

  // Track last requested command (after saturation & deadzone)
  lastTargetHz  = targetHz;
  lastCommandHz = targetHz;

  stepper->setSpeedInHz(fabs(targetHz));

  // IMPORTANT: keep this mapping consistent with hardware
  if (targetHz > 0) {
    stepper->runBackward();
  } else {
    stepper->runForward();
  }
}

// ============================================================
// TELEMETRY (CSV)
// ============================================================

void sendTelemetryCSV() {
  if (millis() - lastTelemetryMs < TELEMETRY_INTERVAL_MS) return;
  lastTelemetryMs = millis();
  if (!balanceEnabled) return;

  // Recompute setpoint exactly like main control (for logging)
  float pendulumSetpoint = 0.0f;
  if (fabs(motorAngle) > 10.0f) {
    pendulumSetpoint = -Kp_center * motorAngle;
    pendulumSetpoint = constrain(pendulumSetpoint, -5.0f, 5.0f);
  }

  Serial.print(millis() / 1000.0f, 3); Serial.print(",");
  Serial.print(pendulumSetpoint, 2);   Serial.print(",");
  Serial.print(pendAngle, 2);          Serial.print(",");
  Serial.print(motorAngle, 2);         Serial.print(",");
  Serial.print(pendVelocity, 2);       Serial.print(",");
  Serial.print(motorVelocity, 2);      Serial.print(",");
  Serial.println((int)lastCommandHz);  // actual commanded Hz
}

// ============================================================
// BALANCE CONTROL
// ============================================================

void runBalanceControl() {
  // 1. Pendulum safety
  if (fabs(pendAngle) > PEND_LIMIT_DEG) {
    balanceEnabled = false;
    stepper->forceStopAndNewPosition(0);
    digitalWrite(EN_PIN, HIGH);
    cmdHz = 0.0f;
    Serial.println(F("⚠ Pendulum fell!"));
    Serial.println(F("[BALANCE_LOG_END],0,pendulum_fell"));
    return;
  }

  // 2. Pendulum setpoint (for base centering)
  float pendulumSetpoint = 0.0f;
  if (fabs(motorAngle) > 10.0f) {
    pendulumSetpoint = -Kp_center * motorAngle;
    pendulumSetpoint = constrain(pendulumSetpoint, -5.0f, 5.0f);
  }

  // 3. Incremental "acceleration-like" PD on pendulum angle (upright = 0°)
  float error = pendAngle - pendulumSetpoint;

  // Integral kept but not used in control (Ki_balance = 0)
  integralError += error * DT_FIXED;
  integralError = constrain(integralError, -INTEGRAL_MAX_DEG, INTEGRAL_MAX_DEG);

  // Clamp velocity used in derivative term to avoid excessive action
  float velForD = constrain(pendVelocity, -D_VEL_CLAMP, D_VEL_CLAMP);

  // ΔHz per tick from PD (derived from state-space / acceleration model)
  float deltaHz =
    (Kp_balance * error) +
    (Kd_balance * velForD);

  // Integrate "acceleration" to speed command
  cmdHz += deltaHz;

  // 4. Saturate speed command
  if (cmdHz >  MAX_MOTOR_HZ) cmdHz =  MAX_MOTOR_HZ;
  if (cmdHz < -MAX_MOTOR_HZ) cmdHz = -MAX_MOTOR_HZ;

  float targetHz = cmdHz;

  // 5. Soft base limits – exponential braking (also applied to cmdHz)
  if (fabs(motorAngle) > SOFT_LIMIT_DEG) {
    float over        = fabs(motorAngle) - SOFT_LIMIT_DEG;
    float span        = BASE_LIMIT_DEG - SOFT_LIMIT_DEG;
    float brakeFactor = exp(-3.0f * (over / span));
    brakeFactor       = constrain(brakeFactor, 0.05f, 1.0f);
    cmdHz            *= brakeFactor;
    targetHz          = cmdHz;
  }

  // 6. Hard base limit – shutdown
  if (fabs(motorAngle) > BASE_LIMIT_DEG) {
    balanceEnabled = false;
    stepper->forceStopAndNewPosition(0);
    digitalWrite(EN_PIN, HIGH);
    cmdHz = 0.0f;
    Serial.print(F("⚠ Base limit hit: "));
    Serial.println(motorAngle);
    Serial.println(F("[BALANCE_LOG_END],0,base_limit"));
    return;
  }

  // 7. Apply motor command
  setMotorVelocity(targetHz);

  // 8. Telemetry (throttled)
  sendTelemetryCSV();
}

// ============================================================
// SERIAL COMMAND HANDLER
// ============================================================

void handleCommand(const String &cmd) {
  if (cmd.length() == 0) return;
  char c = toupper(cmd.charAt(0));
  float val = cmd.substring(1).toFloat();

  switch (c) {
    case 'S': {  // start balancing
      pendVelocity   = 0.0f;
      motorVelocity  = 0.0f;
      integralError  = 0.0f;
      cmdHz          = 0.0f;
      lastTargetHz   = 0.0f;
      lastCommandHz  = 0.0f;

      updateSensors();
      lastPendAngle  = pendAngle;
      lastMotorAngle = motorAngle;

      balanceEnabled = true;
      digitalWrite(EN_PIN, LOW);

      Serial.println(F("[BALANCE_LOG_START],0,firmware"));
      Serial.print(F("[BALANCE_GAINS],"));
      Serial.print(Kp_balance, 2); Serial.print(",");
      Serial.print(Ki_balance, 2); Serial.print(",");
      Serial.print(Kd_balance, 2); Serial.print(",");
      Serial.println(Kp_center, 3);
      Serial.println(F("[BALANCE_COLUMNS],time_s,setpoint_deg,pendulum_deg,base_deg,pendulum_vel,base_vel,control_output"));
      Serial.println(F("✓ Balance ON"));
      break;
    }

    case 'X': {  // stop balancing
      balanceEnabled = false;
      stepper->forceStop();
      digitalWrite(EN_PIN, HIGH);
      cmdHz = 0.0f;
      Serial.println(F("[BALANCE_LOG_END],0,user_stop"));
      Serial.println(F("✗ Balance OFF"));
      break;
    }

    case 'P':
      Kp_balance = val;
      Serial.print(F("Kp_inc=")); Serial.println(val);
      break;

    case 'I':
      Ki_balance = val;
      Serial.print(F("Ki=")); Serial.println(val);
      break;

    case 'D':
      Kd_balance = val;
      Serial.print(F("Kd_inc=")); Serial.println(val);
      break;

    case 'M':
      Kp_center = val;
      Serial.print(F("Kp_C=")); Serial.println(val);
      break;

    case 'Y': { // calibrate pendulum zero (upright)
      tcaSelect(0);
      uint16_t pRaw = pendulumSensor.readAngle();
      float pDegRaw = (pRaw * 360.0f) / 4096.0f;
      pendulumZeroDeg = pDegRaw;
      Serial.print(F("New pendulumZeroDeg = "));
      Serial.println(pendulumZeroDeg, 4);
      break;
    }

    case 'Z': { // calibrate motor zero (base center)
      tcaSelect(1);
      uint16_t mRaw = motorSensor.readAngle();
      float mDegRaw = (mRaw * 360.0f) / 4096.0f;
      motorZeroDeg = mDegRaw;
      Serial.print(F("New motorZeroDeg = "));
      Serial.println(motorZeroDeg, 4);
      break;
    }

    case 'T': {  // diagnostic snapshot
      float pendulumSetpoint = 0.0f;
      if (fabs(motorAngle) > 10.0f) {
        pendulumSetpoint = -Kp_center * motorAngle;
        pendulumSetpoint = constrain(pendulumSetpoint, -5.0f, 5.0f);
      }
      float error   = pendAngle - pendulumSetpoint;
      float velForD = constrain(pendVelocity, -D_VEL_CLAMP, D_VEL_CLAMP);

      // For diagnostics: show incremental command and resulting next Hz
      float deltaHz   = (Kp_balance * error) + (Kd_balance * velForD);
      float nextCmdHz = cmdHz + deltaHz;

      Serial.print(F("P:"));     Serial.print(pendAngle, 2);
      Serial.print(F(" M:"));    Serial.print(motorAngle, 2);
      Serial.print(F(" Err:"));  Serial.print(error, 2);
      Serial.print(F(" Vel:"));  Serial.print(pendVelocity, 2);
      Serial.print(F(" dHz:"));  Serial.print(deltaHz, 1);
      Serial.print(F(" Hz:"));   Serial.println(nextCmdHz, 1);
      break;
    }

    case 'C': { // connection check for motor/stepper
      Serial.println(F("--- CONNECTION CHECK ---"));
      if (stepper) Serial.println(F("Stepper: OK"));
      else         Serial.println(F("Stepper: MISSING or init failed"));

      pinMode(EN_PIN, INPUT_PULLUP);
      int enState = digitalRead(EN_PIN);
      Serial.print(F("EN_PIN (pullup-read): "));
      Serial.println(enState == HIGH ? F("DISABLED/HIGH") : F("ENABLED/LOW"));
      pinMode(EN_PIN, OUTPUT); // restore

      tcaSelect(1);
      bool motorMag = motorSensor.detectMagnet();
      Serial.print(F("Motor sensor magnet: "));
      Serial.println(motorMag ? F("OK") : F("NOT DETECTED"));

      Serial.print(F("Motor zero (deg): "));
      Serial.println(motorZeroDeg, 4);
      Serial.print(F("Motor angle (deg): "));
      Serial.println(motorAngle, 4);

      tcaSelect(0);
      bool pendMag = pendulumSensor.detectMagnet();
      Serial.print(F("Pendulum sensor magnet: "));
      Serial.println(pendMag ? F("OK") : F("NOT DETECTED"));

      Serial.println(F("--- END CHECK ---"));
      break;
    }

    default:
      Serial.println(F("Cmds: S,X,P#,I#,D#,M#,Y,Z,T"));
      break;
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(500000);          // high baud to reduce blocking
  Wire.begin();
  Wire.setClock(400000);         // fast I2C

#if defined(WIRE_HAS_ENDTRANSMISSION_TIMEOUT)
  Wire.setWireTimeout(5000, true);
#endif

  Serial.println(F("Init Sensors..."));

  // Pendulum sensor (TCA channel 0)
  tcaSelect(0);
  pendulumSensor.begin();
  delay(10);
  if (!pendulumSensor.detectMagnet()) {
    Serial.println(F("⚠ ERROR: Pendulum Magnet (Ch0) NOT detected!"));
    sensorsPresent = false;
  }

  // Motor sensor (TCA channel 1)
  tcaSelect(1);
  motorSensor.begin();
  delay(10);
  if (!motorSensor.detectMagnet()) {
    Serial.println(F("⚠ ERROR: Motor Magnet (Ch1) NOT detected!"));
    sensorsPresent = false;
  }

  if (!sensorsPresent) {
    Serial.println(F("Continuing in DIAGNOSTIC mode — sensors not fully detected."));
    Serial.println(F("Use 'C' command to inspect motor/stepper status."));
  }

  // Stepper setup
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);  // disabled initially

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) {
    Serial.println(F("Stepper init failed! Check STEP_PIN wiring."));
    while (1) { /* halt */ }
  }

  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(EN_PIN, true);
  stepper->setAutoEnable(false);
  stepper->setAcceleration(100000);
  stepper->setSpeedInHz(0);

  // Timer3 -> 8ms tick (125 Hz)
  noInterrupts();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;
  TCCR3B |= (1 << WGM32);              // CTC mode
  TCCR3B |= (1 << CS31) | (1 << CS30); // prescaler 64
  OCR3A   = 1999;                      // 8ms at 16MHz/64
  TIMSK3 |= (1 << OCIE3A);             // enable compare A interrupt
  interrupts();

  Serial.println(F("\nRotary Inverted Pendulum V7.1"));
  Serial.println(F("Ready."));
  Serial.println(F("1) Center base, hold pendulum upright."));
  Serial.println(F("2) Optional: send 'Y' (pendulum zero) and 'Z' (motor zero)."));
  Serial.println(F("3) Send 'T' to verify P~0 and M~0."));
  Serial.println(F("4) Send 'S' to start balance tests."));
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  // 1. Periodic magnet check
  if (millis() - lastMagnetCheckMs >= MAGNET_CHECK_INTERVAL_MS) {
    noInterrupts();
    lastMagnetCheckMs = millis();
    magnetCheckPending = true;
    interrupts();
  }

  // 2. Serial command handling
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

  // 3. Control loop – at most one tick per main loop iteration
  uint8_t ticksToProcess;
  noInterrupts();
  ticksToProcess = controlTickCnt;
  controlTickCnt = 0;
  interrupts();

  if (ticksToProcess > 0) {
    updateSensors();
    if (balanceEnabled) {
      runBalanceControl();
    }
    // No prints here: keep loop lean.
  }
}
