#include <Arduino.h>
#include <Wire.h>
#include <FastAccelStepper.h>
#include <AS5600.h>
#include <EEPROM.h>

#define STEP_PIN 11
#define DIR_PIN  6
#define EN_PIN   7
#define MUX_ADDR 0x70

const float STEPS_PER_DEG = 4.444f;
const float LIM_MOTOR_DEG = 80.0f;
const float LIM_PEND_DEG  = 30.0f;

const float MAX_ACC_STEPS = 20000.0f;  // safe accel (steps/s^2)
const float MAX_SPEED_HZ  = 15000.0f;  // Changed from 25 to 15000 speed limit (steps/s)

const uint32_t LOOP_US = 5000;         // 200 Hz
const float ALPHADOT_CLAMP = 300.0f;   // deg/s
const float THETADOT_CLAMP = 300.0f;   // deg/s safety clamp for base rate
const float ALPHA_DEADBAND_DEG = 0.5f; // reduce sensor noise chasing

// Reject impossible one-sample jumps (usually I2C/mux glitches).
// NOTE: this is per control tick (~5 ms), not per log sample (~20 ms).
const float ALPHA_JUMP_REJECT_DEG = 30.0f;
const float THETA_JUMP_REJECT_DEG = 30.0f;

// Require multiple consecutive out-of-range ticks before declaring FALLEN.
const uint8_t FALL_COUNT_REQ = 3;

// Detect when we are commanding significant motor speed, but the base angle doesn't move.
// This usually indicates a motor/driver/mechanical issue (enable dropped, stall, loose coupling, etc.).
const float BASE_STALL_CMD_HZ = 500.0f;            // steps/s
const float BASE_STALL_MIN_DTHETA_DEG = 0.10f;     // deg per control tick
const uint16_t BASE_STALL_WARN_TICKS = 30;         // 150 ms at 200 Hz

// AS5600 status bits (not exposed as public constants in the library header).
constexpr uint8_t AS5600_STATUS_MAGNET_HIGH = 0x08;
constexpr uint8_t AS5600_STATUS_MAGNET_LOW = 0x10;
constexpr uint8_t AS5600_STATUS_MAGNET_DETECT = 0x20;

// Calibration sampling (averaging reduces ~0.5deg single-sample noise that can create large theta offsets).
const int CAL_SAMPLES = 25;
const int CAL_SAMPLE_DELAY_US = 2000;

// Auto-trim the pendulum reference very slowly to kill persistent alpha bias (prevents big theta offsets).
const float ALPHA_TRIM_MAX_DEG = 1.5f;          // clamp reference drift from calibration
const float ALPHA_TRIM_RATE = 0.20f;            // 1/s (time constant ~5s)
const float ALPHA_TRIM_ALPHA_WINDOW_DEG = 3.0f; // only trim when near upright
const float ALPHA_TRIM_ALPHADOT_WINDOW_DEG_S = 50.0f;  // only trim when pendulum is not moving fast

// Full-state feedback gains (from Section 11.7 of modelling_complete.md)
// Control law: ddot_theta = K_THETA*theta + K_ALPHA*alpha + K_THETADOT*thetaDot + K_ALPHADOT*alphaDot
// NOTE: Gains are expressed as positive magnitudes; use ALPHA_SIGN / THETA_SIGN / CTRL_SIGN
// to match the firmware state definitions to your physical directions.
// NOTE: K_THETA increased from 9.92 to 30 for stronger base centering (drift prevention)
float K_THETA     = 31.5f;     // 30.0f steps/s² per deg (base position gain)
float K_ALPHA     = 725.6f;    // 858.6f steps/s² per deg (pendulum angle gain)
float K_THETADOT  = 28.3f;     // 50.0f steps/s² per deg/s (base velocity gain)
float K_ALPHADOT  = 73.5f;     // 69.9f steps/s² per deg/s (pendulum velocity gain)

// --- PROFESSOR'S DERIVATIVE FILTER METHOD ---
// Uses bilinear transform: H(s) = s/(1 + s/wc)
// Tuning: omega_c is cutoff frequency in rad/s (20 Hz ≈ 125 rad/s)
float omega_c = 450.0f;     // Cutoff frequency (rad/s)

// Discrete filter coefficients for professor's derivative filter (computed from omega_c and nominal dt).
float d_b0 = 0.0f;  // Numerator coefficient
float d_a1 = 0.0f;  // Denominator coefficient (feedback)

// History variables for derivative filter
float last_alpha_raw = 0.0f;
float last_alpha_dot = 0.0f;
float last_theta_raw = 0.0f;
float last_theta_dot = 0.0f;
bool  derivInit    = false;

// Optional: velocity leak (bias killer)
float VEL_LEAK = 2.0f; // 1/s
const float VEL_LEAK_ALPHA_WINDOW_DEG = 5.0f; // only leak when |alpha| is small (near upright)
const float VEL_LEAK_THETA_WINDOW_DEG = 20.0f; // only leak when |theta| is small (near center)

// ---------------- BASE POSITION COMMAND ("NUDGE" MODE) ----------------
const float THETA_TARGET_LIMIT_DEG = 35.0f;
float thetaMoveMaxVelDegS = 12.0f;
float thetaMoveMaxAccDegS2 = 60.0f;

const float MOVE_ALPHA_WINDOW_DEG = 5.0f;
const float MOVE_ALPHADOT_WINDOW_DEG_S = 80.0f;

const float THETA_REF_SNAP_EPS_DEG = 0.05f;
const float THETA_REF_SNAP_VEL_EPS_DEG_S = 0.2f;

const uint32_t MOVE_ENGAGE_GRACE_MS = 200;
const uint8_t MOVE_UNSTABLE_TICKS_REQ = 10;  // 50ms @ 200Hz (debounce)
const uint8_t MOVE_STABLE_TICKS_REQ = 10;    // 50ms @ 200Hz (debounce)

// ---------------- NONLINEAR UPRIGHT CONTROL (SMC) ----------------
// Identified plant constants (tools/modelling_complete.md):
//   alphaDDot = A*sin(alpha) + sin(alpha)*cos(alpha)*thetaDot^2 - B*cos(alpha)*thetaDDot
const float MODEL_A = 100.8f;  // 1/s^2 (rad-based)
const float MODEL_B = 1.952f;  // dimensionless

float smcLambda = 15.0f;       // 1/s
float smcKDegS2 = 800.0f;      // deg/s^2
float smcPhiDegS = 50.0f;      // deg/s (boundary layer thickness)
float smcSign = +1.0f;         // SMC-only sign flip knob (+1 or -1)
float smcBaseScale = 1.0f;     // 0..2 scale for base tracking blend (default: keep arm near center)

// Full-state surface SMC (SMC4): surface includes alpha/alphaDot/thetaErr/thetaDotErr.
//
// IMPORTANT:
// - Keep `smcFullK >= 0`. This gain multiplies `thetaDotErr` in the surface. Making it negative
//   introduces anti-damping on base velocity, and has repeatedly caused immediate base run-away
//   to the mechanical limits in SMC4 experiments.
// - Keep `smcFullLambdaTheta > 0`. On the sliding manifold, it yields theta dynamics
//     thetaDotErr = -smcFullLambdaTheta * thetaErr
//   so negative values are mathematically unstable (theta drifts toward a limit).
// - If SMC4 accelerates in the wrong physical direction, flip `smcSign` (command `Q -1` / `Q 1`).
float smcFullK = 0.50f;              // >=0, dimensionless (theta coupling weight)
float smcFullLambdaTheta = 2.00f;    // 1/s (>0 for stable centering)

const float SMC_ALPHA_ABORT_DEG = 25.0f;  // upright-only hard window (no swing-up)
const float SMC_ALPHADOT_ABORT_DEG_S = 250.0f;  // abort if pendulum rate is extreme (upright-only)
const uint8_t SMC_ALPHADOT_ABORT_TICKS_REQ = 3; // debounce to avoid single-sample noise
const uint32_t SMC_ABORT_GRACE_MS = 200;        // ignore alphaDot abort briefly after ENGAGED! (engage transient)
const float SMC_ALPHADOT_ABORT_MIN_ALPHA_DEG = 5.0f;  // avoid aborting when alpha is near 0 but alphaDot spikes
// Gate the SMC base-centering blend with a looser window than MOVE_* (which is only for profile advance).
// We still want centering active during typical alphaDot transients.
const float SMC_BASE_GATE_ALPHA_WINDOW_DEG = 5.0f;
const float SMC_BASE_GATE_ALPHADOT_WINDOW_DEG_S = 150.0f;
// SMC4 needs base-centering assist more often (otherwise it can "random walk" the arm while balancing).
// Use a wider alphaDot window (up to the abort threshold) so centering is active during typical transients.
const float SMC4_BASE_GATE_ALPHADOT_WINDOW_DEG_S = SMC_ALPHADOT_ABORT_DEG_S;
const float SMC_COS_MIN = 0.2f;           // glitch guard for cos(alpha) in division
const float THETA_DDOT_MAX_DEG_S2 = MAX_ACC_STEPS / STEPS_PER_DEG;  // clamp before steps conversion

// Full-state SMC (SMC4) safeguards.
const float SMC_FULL_K_MAX = 1.20f;
const float SMC_FULL_DENOM_MIN = 0.60f;
const uint32_t SMC_FULL_K_RAMP_MS = 200;
const float SMC_FULL_THETA_ERR_CLAMP_DEG = 78.0f;
const float SMC_FULL_THETADOT_ERR_CLAMP_DEG_S = 200.0f;
const float SMC_FULL_THETA_TERM_CLAMP_DEG_S = 300.0f;

// SMC4 actuator shaping (stepper-friendly):
// - Clamp alphaDot used *inside* SMC4 math (keeps lambda*alphaDot from producing extreme thetaDDot spikes).
// - Use tighter accel/speed limits than the global ones (reduces "tick"/stall events during disturbance recovery).
const float SMC4_ALPHA_DOT_CTRL_CLAMP_DEG_S = 200.0f;
const float SMC4_MAX_ACC_STEPS = 12000.0f;  // steps/s^2 (<= MAX_ACC_STEPS)
const float SMC4_MAX_SPEED_HZ = 2000.0f;    // steps/s (<= MAX_SPEED_HZ)

// Signs
int motorSign  = +1;   // +steps = CW from above
int ALPHA_SIGN = -1;   // default (matches historical working builds); verify with 'S' and save with 'Y'
int THETA_SIGN = +1;
int CTRL_SIGN  = +1;

enum CtrlMode { CTRL_LINEAR = 0, CTRL_SMC = 1, CTRL_SMC_FULL = 2 };
CtrlMode ctrlMode = CTRL_LINEAR;

bool armEnabled   = false;
bool firstIdle    = true;
bool isCalibrated = false;

const float ENGAGE_PEND_DEG = 3.0f;
const float ENGAGE_DOT_DEG  = 20.0f;
uint32_t engageStartMs = 0;

int engageCount = 0;
const int ENGAGE_COUNT_REQ = 20;  // 100 ms at 200 Hz

FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;
AS5600 sensor;

enum State { STATE_CALIBRATE, STATE_IDLE, STATE_ACTIVE };
State currentState = STATE_CALIBRATE;

float targetRawDeg = 0.0f;
float targetRawDegCal = 0.0f;  // calibration reference (for auto-trim clamp)
float motorZeroDeg = 0.0f;

// ---------------- BASE POSITION REFERENCE STATE ----------------
float thetaTargetDeg = 0.0f;
float thetaRefDeg = 0.0f;
float thetaRefVelDegS = 0.0f;
float thetaRefAccDegS2 = 0.0f;
bool movePaused = false;
uint8_t moveStableTicks = 0;
uint8_t moveUnstableTicks = 0;

// Controller command state (now ONLY speed is used for actuation)
float thetaDotCmdMotor   = 0.0f;   // steps/s (MOTOR coordinates)

// Derivative filter outputs
float alphaDotFilt = 0.0f;
float thetaDotFilt = 0.0f;

// Timing
uint32_t lastUs = 0;
uint32_t logDecim = 0;

// ---------------- VELOCITY MODE STATE ----------------
int runDir = 0;                 // -1 backward, +1 forward, 0 stopped
// Motor velocity deadband (steps/s).
// If |vel_cmd| < speedStopHz, the motor is stopped (reduces buzz / dithering).
// Set to 0 to effectively disable the deadband (most linear control, but may be noisier).
// Tune with 'N' and save with 'Y'.
float speedStopHz = 50.0f;

// -----------------------------------------------------
uint8_t fallCount = 0;
int32_t lastAccCmdDiag = 0;
float thetaDegPrevForStall = 0.0f;
uint16_t stallNoMoveTicks = 0;
bool stallWarned = false;
uint8_t smcAlphaDotAbortTicks = 0;

// ---------------- DIAGNOSTIC COUNTERS (ACTIVE ONLY) ----------------
uint32_t dbgTicksActive = 0;
uint32_t dbgDtUsMin = 0xFFFFFFFFUL;
uint32_t dbgDtUsMax = 0;
uint32_t dbgDtOverrun = 0;

uint32_t dbgAccSat = 0;
uint32_t dbgAlphaGlitchReject = 0;
uint32_t dbgThetaGlitchReject = 0;
uint32_t dbgAlphaDerivReject = 0;
uint32_t dbgThetaDerivReject = 0;
uint32_t dbgAlphaI2cErr = 0;
uint32_t dbgThetaI2cErr = 0;

uint32_t dbgMotorStart = 0;
uint32_t dbgMotorStop = 0;
uint32_t dbgMotorReverse = 0;
uint32_t dbgMotorRunningTicks = 0;

uint32_t dbgLeakActiveTicks = 0;
uint32_t dbgTrimActiveTicks = 0;
uint32_t dbgTrimClampedTicks = 0;
uint32_t dbgBaseStallEvents = 0;
uint32_t dbgBaseStallTicks = 0;

static void resetDebugCounters() {
  dbgTicksActive = 0;
  dbgDtUsMin = 0xFFFFFFFFUL;
  dbgDtUsMax = 0;
  dbgDtOverrun = 0;

  dbgAccSat = 0;
  dbgAlphaGlitchReject = 0;
  dbgThetaGlitchReject = 0;
  dbgAlphaDerivReject = 0;
  dbgThetaDerivReject = 0;
  dbgAlphaI2cErr = 0;
  dbgThetaI2cErr = 0;

  dbgMotorStart = 0;
  dbgMotorStop = 0;
  dbgMotorReverse = 0;
  dbgMotorRunningTicks = 0;

  dbgLeakActiveTicks = 0;
  dbgTrimActiveTicks = 0;
  dbgTrimClampedTicks = 0;
  dbgBaseStallEvents = 0;
  dbgBaseStallTicks = 0;
}

// ---------------- PERSISTED SETTINGS (EEPROM) ----------------
constexpr uint32_t EEPROM_MAGIC = 0x52495031UL;  // "RIP1"
constexpr int EEPROM_ADDR = 0;

struct PersistedConfig {
  uint32_t magic;
  int8_t motorSign;
  int8_t alphaSign;
  int8_t thetaSign;
  int8_t ctrlSign;
  float omega_c;
  float speedStopHz;
};

static void updateDerivativeCoeffs();

static bool loadPersistedConfig() {
  PersistedConfig cfg{};
  EEPROM.get(EEPROM_ADDR, cfg);

  if (cfg.magic != EEPROM_MAGIC) return false;
  if (cfg.motorSign != 1 && cfg.motorSign != -1) return false;
  if (cfg.alphaSign != 1 && cfg.alphaSign != -1) return false;
  if (cfg.thetaSign != 1 && cfg.thetaSign != -1) return false;
  if (cfg.ctrlSign != 1 && cfg.ctrlSign != -1) return false;
  if (!(cfg.omega_c >= 1.0f && cfg.omega_c <= 2000.0f)) return false;
  if (!(cfg.speedStopHz >= 0.0f && cfg.speedStopHz <= 500.0f)) return false;

  motorSign = cfg.motorSign;
  ALPHA_SIGN = cfg.alphaSign;
  THETA_SIGN = cfg.thetaSign;
  CTRL_SIGN = cfg.ctrlSign;
  omega_c = cfg.omega_c;
  speedStopHz = cfg.speedStopHz;

  return true;
}

static void savePersistedConfig() {
  PersistedConfig cfg{};
  cfg.magic = EEPROM_MAGIC;
  cfg.motorSign = (int8_t)motorSign;
  cfg.alphaSign = (int8_t)ALPHA_SIGN;
  cfg.thetaSign = (int8_t)THETA_SIGN;
  cfg.ctrlSign = (int8_t)CTRL_SIGN;
  cfg.omega_c = omega_c;
  cfg.speedStopHz = speedStopHz;

  EEPROM.put(EEPROM_ADDR, cfg);
  Serial.println("Saved settings to EEPROM.");
  Serial.print("motorSign="); Serial.print(motorSign);
  Serial.print(" ALPHA_SIGN="); Serial.print(ALPHA_SIGN);
  Serial.print(" THETA_SIGN="); Serial.print(THETA_SIGN);
  Serial.print(" CTRL_SIGN="); Serial.print(CTRL_SIGN);
  Serial.print(" omega_c="); Serial.print(omega_c, 1);
  Serial.print(" speedStopHz="); Serial.println(speedStopHz, 2);
}

static void clearPersistedConfig() {
  PersistedConfig cfg{};
  cfg.magic = 0;
  EEPROM.put(EEPROM_ADDR, cfg);

  // Apply defaults immediately in RAM so the next 'Y' doesn't accidentally re-save old values.
  motorSign = +1;
  ALPHA_SIGN = -1;
  THETA_SIGN = +1;
  CTRL_SIGN = +1;
  omega_c = 450.0f;
  speedStopHz = 50.0f;
  updateDerivativeCoeffs();
  derivInit = false;

  Serial.println("Cleared EEPROM settings and applied defaults in RAM.");
  Serial.print("motorSign="); Serial.print(motorSign);
  Serial.print(" ALPHA_SIGN="); Serial.print(ALPHA_SIGN);
  Serial.print(" THETA_SIGN="); Serial.print(THETA_SIGN);
  Serial.print(" CTRL_SIGN="); Serial.print(CTRL_SIGN);
  Serial.print(" omega_c="); Serial.print(omega_c, 1);
  Serial.print(" speedStopHz="); Serial.println(speedStopHz, 2);
}

static void updateDerivativeCoeffs() {
  const float dt_nom = LOOP_US * 1e-6f;
  const float denom = 2.0f + omega_c * dt_nom;
  d_b0 = (2.0f * omega_c) / denom;
  d_a1 = (omega_c * dt_nom - 2.0f) / denom;
}

// -------------------------------------------------------------

void selectMux(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delayMicroseconds(200);
}

float readAS5600Deg() {
  return sensor.rawAngle() * 0.087890625f;
}

float wrap360(float a) {
  while (a >= 360.0f) a -= 360.0f;
  while (a < 0.0f) a += 360.0f;
  return a;
}

float getAngleDiffDeg(float currentDeg, float targetDeg) {
  float diff = currentDeg - targetDeg;
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

float averageMuxAngleDeg(uint8_t ch, int samples, int delayUs) {
  float s = 0.0f, c = 0.0f;
  const float k = PI / 180.0f;

  for (int i = 0; i < samples; i++) {
    selectMux(ch);
    float deg = wrap360(readAS5600Deg());
    float rad = deg * k;
    s += sin(rad);
    c += cos(rad);
    if (delayUs > 0) delayMicroseconds((uint32_t)delayUs);
  }

  float avgRad = atan2(s, c);
  float avgDeg = avgRad * (180.0f / PI);
  return wrap360(avgDeg);
}

static int signum(float x) {
  return (x > 0.0f) - (x < 0.0f);
}

static void stepThetaReferenceTrapezoid(float dt) {
  float dist = getAngleDiffDeg(thetaTargetDeg, thetaRefDeg);  // target - ref (wrap-safe)

  if (fabs(dist) < THETA_REF_SNAP_EPS_DEG &&
      fabs(thetaRefVelDegS) < THETA_REF_SNAP_VEL_EPS_DEG_S) {
    thetaRefDeg = thetaTargetDeg;
    thetaRefVelDegS = 0.0f;
    thetaRefAccDegS2 = 0.0f;
    return;
  }

  const float amax = max(thetaMoveMaxAccDegS2, 1e-3f);
  const float vmax = max(thetaMoveMaxVelDegS, 1e-3f);

  float stopDist = (thetaRefVelDegS * thetaRefVelDegS) / (2.0f * amax);

  if (fabs(dist) <= stopDist) {
    thetaRefAccDegS2 = -((float)signum(thetaRefVelDegS)) * amax;
  } else {
    thetaRefAccDegS2 = ((float)signum(dist)) * amax;
  }

  thetaRefVelDegS += thetaRefAccDegS2 * dt;
  thetaRefVelDegS = constrain(thetaRefVelDegS, -vmax, vmax);

  thetaRefDeg += thetaRefVelDegS * dt;

  float newDist = getAngleDiffDeg(thetaTargetDeg, thetaRefDeg);
  if (signum(dist) != 0 && signum(newDist) != signum(dist)) {
    thetaRefDeg = thetaTargetDeg;
    thetaRefVelDegS = 0.0f;
    thetaRefAccDegS2 = 0.0f;
  }
}

// Command motor in continuous velocity mode
void commandMotorSpeed(float v_steps_per_s) {
  const int prevRunDir = runDir;
  float vabs = fabs(v_steps_per_s);

  // Stop: DO NOT call setSpeedInHz(0). Use stopMove() for decel stop.
  // Always stop for truly tiny commands (avoids unintended creep if speedStopHz is set to 0).
  if (vabs < 0.5f || (speedStopHz > 0.0f && vabs < speedStopHz)) {
    if (runDir != 0) {
      stepper->stopMove();
      runDir = 0;
      dbgMotorStop++;
    }
    return;
  }

  const int desiredDir = (v_steps_per_s > 0.0f) ? +1 : -1;

  // Stop-before-reverse (reduces jerk and avoids direction chatter near zero).
  if (runDir != 0 && desiredDir != runDir) {
    stepper->stopMove();
    runDir = 0;
    dbgMotorStop++;
    dbgMotorReverse++;
    return;
  }

  uint32_t spd = (uint32_t)constrain(vabs, 1.0f, MAX_SPEED_HZ);
  stepper->setSpeedInHz(spd);

  if (desiredDir > 0) {
    if (runDir != +1) {
      stepper->runForward();
      runDir = +1;
      if (prevRunDir == 0) dbgMotorStart++;
    } else {
      stepper->applySpeedAcceleration();
    }
  } else {
    if (runDir != -1) {
      stepper->runBackward();
      runDir = -1;
      if (prevRunDir == 0) dbgMotorStart++;
    } else {
      stepper->applySpeedAcceleration();
    }
  }
}

void resetController() {
  thetaDotCmdMotor     = 0.0f;

  alphaDotFilt         = 0.0f;
  thetaDotFilt         = 0.0f;

  thetaRefVelDegS      = 0.0f;
  thetaRefAccDegS2     = 0.0f;
  movePaused           = false;
  moveStableTicks      = 0;
  moveUnstableTicks    = 0;

  // Reset history for professor's derivative filter
  last_alpha_raw       = 0.0f;
  last_alpha_dot       = 0.0f;
  last_theta_raw       = 0.0f;
  last_theta_dot       = 0.0f;

  derivInit            = false;
  logDecim             = 0;
  runDir               = 0;
  engageCount          = 0;
  fallCount            = 0;
  lastAccCmdDiag       = 0;
  thetaDegPrevForStall = 0.0f;
  stallNoMoveTicks     = 0;
  stallWarned          = false;
  smcAlphaDotAbortTicks = 0;
  resetDebugCounters();
}

void enterIdle(bool keepArmed = false) {
  if (stepper) {
    stepper->stopMove();
    stepper->disableOutputs();
  }
  runDir = 0;

  currentState = STATE_IDLE;
  if (!keepArmed) {
    armEnabled = false;
  }
  firstIdle = true;
}

void handleSerial() {
  // When we reject a command that has an argument (e.g., "C 1"), we must
  // discard the remainder of that line so the trailing bytes don't get
  // interpreted as separate one-letter commands (e.g., "1" -> K_THETA=0).
  static bool dropUntilEol = false;

  while (Serial.available()) {
    char c = Serial.read();

    if (dropUntilEol) {
      if (c == '\n' || c == '\r') dropUntilEol = false;
      continue;
    }

    // Emergency disarm (used by tools/run_balance.py on exit).
    // Stops motion, disables motor outputs, and returns to IDLE.
    if (c == '!') {
      while (Serial.available()) Serial.read();
      enterIdle(false);
      Serial.println("DISARMED");
      return;
    }

    // SIGN DIAGNOSTIC MODE (kept using moveTo for discrete moves)
    if (c == 'S' || c == 's') {
      Serial.println("\n========================================");
      Serial.println("       SIGN DIAGNOSTIC MODE");
      Serial.println("========================================\n");

      Serial.println("--- TEST 1: MOTOR DIRECTION ---");
      Serial.println("Watch the arm. I will send +100 steps.");
      Serial.println("Press any key to start...");
      while (!Serial.available()) { delay(10); }
      while (Serial.available()) Serial.read();

      selectMux(1);
      float baseStart = readAS5600Deg();

      stepper->enableOutputs();
      stepper->setCurrentPosition(0);
      stepper->setAcceleration(5000);
      stepper->setSpeedInHz(5000);
      stepper->moveTo(100);
      delay(500);

      selectMux(1);
      float baseAfterPos = readAS5600Deg();
      float baseDiffPos = getAngleDiffDeg(baseAfterPos, baseStart);

      Serial.println("\n  Did the arm move CLOCKWISE or COUNTER-CLOCKWISE?");
      Serial.println("  (View from ABOVE the arm)");
      Serial.println("  Press 'C' for Clockwise, 'A' for Anti-clockwise (CCW)...");

      char armDir = 0;
      while (armDir != 'C' && armDir != 'c' && armDir != 'A' && armDir != 'a') {
        if (Serial.available()) armDir = Serial.read();
        delay(10);
      }
      while (Serial.available()) Serial.read();

      bool posStepsCW = (armDir == 'C' || armDir == 'c');
      Serial.print("  ==> +steps moves arm "); Serial.println(posStepsCW ? "CLOCKWISE" : "COUNTER-CLOCKWISE");
      Serial.print("  ==> Encoder diff was: "); Serial.print(baseDiffPos, 2); Serial.println(" deg");

      stepper->moveTo(0);
      delay(500);
      stepper->disableOutputs();
      stepper->setAcceleration((uint32_t)MAX_ACC_STEPS);
      stepper->setSpeedInHz((uint32_t)MAX_SPEED_HZ);
      Serial.println();

      Serial.println("--- TEST 2: PENDULUM SENSOR (UPRIGHT) ---");
      Serial.println("Hold pendulum UPRIGHT (as if balancing). Press any key...");
      while (!Serial.available()) { delay(10); }
      while (Serial.available()) Serial.read();

      selectMux(0);
      float pendUp = readAS5600Deg();
      Serial.print("  Pendulum UPRIGHT reading: "); Serial.println(pendUp, 2);

      Serial.println("\n  Now tilt pendulum in the SAME direction the arm just moved (");
      Serial.print("  ");
      Serial.print(posStepsCW ? "CLOCKWISE" : "COUNTER-CLOCKWISE");
      Serial.println(" when viewed from above).");
      Serial.println("  Hold it tilted ~15-30 degrees. Press any key...");

      float pendTilted = pendUp;
      float pendDiff = 0.0f;
      while (true) {
        while (!Serial.available()) { delay(10); }
        while (Serial.available()) Serial.read();

        selectMux(0);
        pendTilted = readAS5600Deg();
        pendDiff = getAngleDiffDeg(pendTilted, pendUp);

        Serial.print("  Pendulum TILTED reading: "); Serial.println(pendTilted, 2);
        Serial.print("  Difference (tilt-from-upright): "); Serial.print(pendDiff, 2); Serial.println(" deg");

        if (fabs(pendDiff) >= 10.0f) break;
        Serial.println("  Too small (|diff| < 10 deg). Tilt more and try again...");
      }

      // We want the firmware conventions to mean:
      //  - motorSign  = +1 => +steps = CLOCKWISE (viewed from above)
      //  - THETA_SIGN = +1 => theta increases when arm moves CLOCKWISE
      //  - ALPHA_SIGN = +1 => alpha increases when pendulum tilts CLOCKWISE (about upright)
      int suggestedMotorSign = posStepsCW ? +1 : -1;
      int suggestedThetaSign = ((baseDiffPos >= 0.0f) ? +1 : -1) * suggestedMotorSign;
      int suggestedAlphaSign = ((pendDiff >= 0.0f) ? +1 : -1) * suggestedMotorSign;

      Serial.println();
      Serial.println("========================================");
      Serial.println("            RECOMMENDED SIGNS");
      Serial.println("========================================");
      Serial.print("  motorSign  = "); Serial.println(suggestedMotorSign);
      Serial.print("  ALPHA_SIGN = "); Serial.println(suggestedAlphaSign);
      Serial.print("  THETA_SIGN = "); Serial.println(suggestedThetaSign);
      Serial.print("  CTRL_SIGN  = "); Serial.print(CTRL_SIGN);
      Serial.println(" (controller sign, toggle with B if needed)");
      Serial.println("========================================\n");

      // Apply recommended signs immediately for the live test (can be saved with 'Y').
      motorSign = suggestedMotorSign;
      ALPHA_SIGN = suggestedAlphaSign;
      THETA_SIGN = suggestedThetaSign;
      Serial.print("Applied signs (not saved): motorSign="); Serial.print(motorSign);
      Serial.print(" ALPHA_SIGN="); Serial.print(ALPHA_SIGN);
      Serial.print(" THETA_SIGN="); Serial.print(THETA_SIGN);
      Serial.print(" CTRL_SIGN="); Serial.println(CTRL_SIGN);
      Serial.println("Send 'Y' anytime to save these to EEPROM.\n");

      Serial.println("--- TEST 3: LIVE UPRIGHT VERIFICATION ---");
      Serial.println("Hold pendulum UPRIGHT. Tilt it slightly and watch arm.");
      Serial.println("Press any key to start (press again to stop)...");
      while (!Serial.available()) { delay(10); }
      while (Serial.available()) Serial.read();

      selectMux(0);
      float upRef = readAS5600Deg();

      stepper->enableOutputs();
      stepper->setCurrentPosition(0);
      stepper->setAcceleration(10000);
      stepper->setSpeedInHz(10000);

      Serial.println("\n  Tilting pendulum should move arm in SAME direction!");
      Serial.println("  Press any key to stop...\n");

      float vel = 0, pos = 0;
      while (!Serial.available()) {
        selectMux(0);
        float pNow = readAS5600Deg();
        float alphaRaw = getAngleDiffDeg(pNow, upRef);
        float alpha = ALPHA_SIGN * alphaRaw;

        float acc = 500.0f * alpha;
        if (acc > 5000) acc = 5000;
        if (acc < -5000) acc = -5000;

        vel += acc * 0.02f;
        vel *= 0.95f;
        pos += vel * 0.02f;
        if (pos > 200) pos = 200;
        if (pos < -200) pos = -200;

        stepper->moveTo((int32_t)pos);

        Serial.print("  alphaRaw="); Serial.print(alphaRaw, 1);
        Serial.print(" deg  alpha="); Serial.print(alpha, 1);
        Serial.print(" deg  accCmd="); Serial.print((int)acc);
        Serial.print("  armPos="); Serial.print((int)pos);
        Serial.println(" steps");

        delay(100);
      }
      while (Serial.available()) Serial.read();

      stepper->moveTo(0);
      delay(300);
      stepper->disableOutputs();
      stepper->setAcceleration((uint32_t)MAX_ACC_STEPS);
      stepper->setSpeedInHz((uint32_t)MAX_SPEED_HZ);

      Serial.println("\n  Arm followed tilt?");
      Serial.println("  If NO: flip ALPHA_SIGN (A 1 or A -1) and rerun S.");
      Serial.println("  If YES: send Y to save, then Z (calibrate), then E (engage).");
      Serial.println("========================================\n");
      return;
    }

    if (c == 'M' || c == 'm') {
      int v = (int)Serial.parseInt();
      if (v == 1 || v == -1) motorSign = v;
      Serial.print("motorSign="); Serial.println(motorSign);
      return;
    }

    if (c == 'Z' || c == 'z') {
      // Flush any trailing characters (e.g. old scripts sending "ZU") so they don't get
      // interpreted as a different command after calibration.
      while (Serial.available()) Serial.read();

      Serial.println("\n--- CALIBRATION (Z) ---");
      Serial.println("Put ARM at CENTER and hold pendulum UPRIGHT.");
      Serial.println("Sampling sensors...");

      float oldTarget = targetRawDeg;
      targetRawDeg = averageMuxAngleDeg(0, CAL_SAMPLES, CAL_SAMPLE_DELAY_US);
      targetRawDegCal = targetRawDeg;
      motorZeroDeg = averageMuxAngleDeg(1, CAL_SAMPLES, CAL_SAMPLE_DELAY_US);

      Serial.print("pendUp="); Serial.print(targetRawDeg, 2);
      Serial.print(" targetRawDeg="); Serial.print(targetRawDeg, 2);
      Serial.print(" (old="); Serial.print(oldTarget, 2); Serial.println(")");
      Serial.print("motorZeroDeg="); Serial.println(motorZeroDeg, 2);

      thetaTargetDeg = 0.0f;
      thetaRefDeg = 0.0f;
      thetaRefVelDegS = 0.0f;
      thetaRefAccDegS2 = 0.0f;
      movePaused = false;
      moveStableTicks = 0;
      moveUnstableTicks = 0;

      stepper->setCurrentPosition(0);
      resetController();

      isCalibrated = true;
      currentState = STATE_IDLE;
      firstIdle = true;
      stepper->disableOutputs();

      Serial.print("motorSign="); Serial.print(motorSign);
      Serial.print(" ALPHA_SIGN="); Serial.print(ALPHA_SIGN);
      Serial.print(" THETA_SIGN="); Serial.print(THETA_SIGN);
      Serial.print(" CTRL_SIGN="); Serial.println(CTRL_SIGN);
      Serial.print("omega_c="); Serial.print(omega_c, 1);
      Serial.print(" (b0="); Serial.print(d_b0, 4);
      Serial.print(", a1="); Serial.print(d_a1, 4);
      Serial.print(") speedStopHz="); Serial.println(speedStopHz, 2);

      Serial.println("CALIBRATED. Hold pendulum UPRIGHT, then press E to arm.");
      return;
    }

    if (c == 'E' || c == 'e') {
      if (!isCalibrated) {
        Serial.println("ERR: Press Z to calibrate first.");
        return;
      }
      armEnabled = !armEnabled;
      Serial.print("armEnabled="); Serial.println(armEnabled ? "ON" : "OFF");
      if (!armEnabled) enterIdle();
      return;
    }

    if (c == 'A' || c == 'a') {
      int v = (int)Serial.parseInt();
      if (v == 1 || v == -1) ALPHA_SIGN = v;
      Serial.print("ALPHA_SIGN="); Serial.println(ALPHA_SIGN);
      return;
    }

    if (c == 'H' || c == 'h') {
      THETA_SIGN *= -1;
      Serial.print("THETA_SIGN="); Serial.println(THETA_SIGN);
      return;
    }

    if (c == 'B' || c == 'b') {
      CTRL_SIGN *= -1;
      Serial.print("CTRL_SIGN="); Serial.println(CTRL_SIGN);
      return;
    }

    // Full-state feedback gain tuning
    if (c == '1') { float v = Serial.parseFloat(); K_THETA=v; Serial.print("K_THETA="); Serial.println(K_THETA,2); return; }
    if (c == '2') { float v = Serial.parseFloat(); K_ALPHA=v; Serial.print("K_ALPHA="); Serial.println(K_ALPHA,2); return; }
    if (c == '4') { float v = Serial.parseFloat(); K_THETADOT=v; Serial.print("K_THETADOT="); Serial.println(K_THETADOT,2); return; }
    if (c == '5') { float v = Serial.parseFloat(); K_ALPHADOT=v; Serial.print("K_ALPHADOT="); Serial.println(K_ALPHADOT,2); return; }

    if (c == 'Y' || c == 'y') { savePersistedConfig(); return; }
    if (c == 'R' || c == 'r') { clearPersistedConfig(); return; }

	    if (c == 'G' || c == 'g') {
	      Serial.print("motorSign="); Serial.print(motorSign);
	      Serial.print(" ALPHA_SIGN="); Serial.print(ALPHA_SIGN);
	      Serial.print(" THETA_SIGN="); Serial.print(THETA_SIGN);
	      Serial.print(" CTRL_SIGN="); Serial.println(CTRL_SIGN);

	      // Raw sensors + stored zeros (helps debug off-center settling / calibration bias).
	      selectMux(0);
	      float pendRawNow = wrap360(readAS5600Deg());
	      uint8_t pendSt = sensor.readStatus();
	      uint8_t pendAgc = sensor.readAGC();
	      uint16_t pendMag = sensor.readMagnitude();
	      selectMux(1);
	      float baseRawNow = wrap360(readAS5600Deg());
	      uint8_t baseSt = sensor.readStatus();
	      uint8_t baseAgc = sensor.readAGC();
	      uint16_t baseMag = sensor.readMagnitude();
	      Serial.print("pendRawDeg="); Serial.print(pendRawNow, 2);
	      Serial.print(" targetRawDeg="); Serial.println(targetRawDeg, 2);
	      Serial.print("pendMag="); Serial.print(pendMag);
	      Serial.print(" agc="); Serial.print(pendAgc);
	      Serial.print(" st=0x"); Serial.print(pendSt, HEX);
	      Serial.print(" md/hi/lo=");
	      Serial.print((pendSt & AS5600_STATUS_MAGNET_DETECT) ? 1 : 0);
	      Serial.print("/");
	      Serial.print((pendSt & AS5600_STATUS_MAGNET_HIGH) ? 1 : 0);
	      Serial.print("/");
	      Serial.println((pendSt & AS5600_STATUS_MAGNET_LOW) ? 1 : 0);
	      Serial.print("targetRawDegCal="); Serial.print(targetRawDegCal, 2);
	      Serial.print(" (diff="); Serial.print(getAngleDiffDeg(targetRawDeg, targetRawDegCal), 2);
	      Serial.println(")");
	      Serial.print("baseRawDeg="); Serial.print(baseRawNow, 2);
	      Serial.print(" motorZeroDeg="); Serial.println(motorZeroDeg, 2);
	      Serial.print("baseMag="); Serial.print(baseMag);
	      Serial.print(" agc="); Serial.print(baseAgc);
	      Serial.print(" st=0x"); Serial.print(baseSt, HEX);
	      Serial.print(" md/hi/lo=");
	      Serial.print((baseSt & AS5600_STATUS_MAGNET_DETECT) ? 1 : 0);
	      Serial.print("/");
	      Serial.print((baseSt & AS5600_STATUS_MAGNET_HIGH) ? 1 : 0);
	      Serial.print("/");
	      Serial.println((baseSt & AS5600_STATUS_MAGNET_LOW) ? 1 : 0);

	      Serial.print("omega_c="); Serial.print(omega_c, 1);
	      Serial.print(" (b0="); Serial.print(d_b0, 4);
	      Serial.print(", a1="); Serial.print(d_a1, 4);
      Serial.print(") speedStopHz="); Serial.println(speedStopHz, 2);
      Serial.print("VEL_LEAK="); Serial.print(VEL_LEAK, 3);
      Serial.print(" leakAlphaWinDeg="); Serial.print(VEL_LEAK_ALPHA_WINDOW_DEG, 2);
      Serial.print(" leakThetaWinDeg="); Serial.println(VEL_LEAK_THETA_WINDOW_DEG, 2);
      Serial.print("K_THETA="); Serial.print(K_THETA, 2);
      Serial.print(" K_ALPHA="); Serial.print(K_ALPHA, 2);
      Serial.print(" K_THETADOT="); Serial.print(K_THETADOT, 2);
      Serial.print(" K_ALPHADOT="); Serial.println(K_ALPHADOT, 2);

      Serial.print("thetaTargetDeg="); Serial.print(thetaTargetDeg, 2);
      Serial.print(" thetaRefDeg="); Serial.print(thetaRefDeg, 2);
      Serial.print(" thetaRefVelDegS="); Serial.print(thetaRefVelDegS, 2);
      Serial.print(" thetaRefAccDegS2="); Serial.println(thetaRefAccDegS2, 2);
      Serial.print("thetaMoveMaxVelDegS="); Serial.print(thetaMoveMaxVelDegS, 2);
      Serial.print(" thetaMoveMaxAccDegS2="); Serial.print(thetaMoveMaxAccDegS2, 2);
      Serial.print(" movePaused="); Serial.println(movePaused ? 1 : 0);

      Serial.print("ctrlMode=");
      if (ctrlMode == CTRL_SMC) Serial.println("SMC");
      else if (ctrlMode == CTRL_SMC_FULL) Serial.println("SMC4");
      else Serial.println("LIN");
      Serial.print("smcLambda="); Serial.print(smcLambda, 2);
      Serial.print(" smcKDegS2="); Serial.print(smcKDegS2, 2);
      Serial.print(" smcPhiDegS="); Serial.print(smcPhiDegS, 2);
      Serial.print(" smcSign="); Serial.print(smcSign, 1);
      Serial.print(" smcBaseScale="); Serial.println(smcBaseScale, 2);
      Serial.print("smcFullK="); Serial.print(smcFullK, 2);
      Serial.print(" smcFullLambdaTheta="); Serial.println(smcFullLambdaTheta, 2);
      return;
    }

    if (c == 'U' || c == 'u') {
      float v = Serial.parseFloat();
      if (v >= 0.0f && v <= 20.0f) VEL_LEAK = v;
      Serial.print("VEL_LEAK="); Serial.println(VEL_LEAK, 3);
      return;
    }

    // Controller mode select (linear full-state vs nonlinear SMC)
    if (c == 'C' || c == 'c') {
      if (currentState == STATE_ACTIVE) {
        dropUntilEol = true;
        Serial.println("ERR: SMC mode changes only allowed when DISARMED (send '!' first).");
        return;
      }
      int v = (int)Serial.parseInt();
      if (v == 2) ctrlMode = CTRL_SMC_FULL;
      else if (v == 1) ctrlMode = CTRL_SMC;
      else ctrlMode = CTRL_LINEAR;

      Serial.print("[CTRL] mode=");
      if (ctrlMode == CTRL_SMC) Serial.println("SMC");
      else if (ctrlMode == CTRL_SMC_FULL) Serial.println("SMC4");
      else Serial.println("LIN");

      if (ctrlMode == CTRL_SMC_FULL) {
        Serial.print("smcFullK="); Serial.print(smcFullK, 2);
        Serial.print(" smcFullLambdaTheta="); Serial.println(smcFullLambdaTheta, 2);
      }
      return;
    }

    // Nonlinear upright SMC tuning
    if (c == 'J' || c == 'j') {
      if (currentState == STATE_ACTIVE) {
        dropUntilEol = true;
        Serial.println("ERR: SMC tuning only allowed when DISARMED (send '!' first).");
        return;
      }
      float v = Serial.parseFloat();
      if (v > 0.0f && v <= 200.0f) smcLambda = v;
      Serial.print("smcLambda="); Serial.println(smcLambda, 2);
      return;
    }

    if (c == 'K' || c == 'k') {
      if (currentState == STATE_ACTIVE) {
        dropUntilEol = true;
        Serial.println("ERR: SMC tuning only allowed when DISARMED (send '!' first).");
        return;
      }
      float v = Serial.parseFloat();
      if (v > 0.0f && v <= 20000.0f) smcKDegS2 = v;
      Serial.print("smcKDegS2="); Serial.println(smcKDegS2, 2);
      return;
    }

    if (c == 'P' || c == 'p') {
      if (currentState == STATE_ACTIVE) {
        dropUntilEol = true;
        Serial.println("ERR: SMC tuning only allowed when DISARMED (send '!' first).");
        return;
      }
      float v = Serial.parseFloat();
      if (v > 0.0f && v <= 10000.0f) smcPhiDegS = v;
      Serial.print("smcPhiDegS="); Serial.println(smcPhiDegS, 2);
      return;
    }

    if (c == 'Q' || c == 'q') {
      if (currentState == STATE_ACTIVE) {
        dropUntilEol = true;
        Serial.println("ERR: SMC tuning only allowed when DISARMED (send '!' first).");
        return;
      }
      int v = (int)Serial.parseInt();
      if (v == 1 || v == -1) smcSign = (float)v;
      Serial.print("smcSign="); Serial.println(smcSign, 1);
      return;
    }

    if (c == 'O' || c == 'o') {
      if (currentState == STATE_ACTIVE) {
        dropUntilEol = true;
        Serial.println("ERR: SMC tuning only allowed when DISARMED (send '!' first).");
        return;
      }
      float v = Serial.parseFloat();
      if (v >= 0.0f && v <= 2.0f) smcBaseScale = v;
      Serial.print("smcBaseScale="); Serial.println(smcBaseScale, 2);
      return;
    }

    // Full-state surface SMC (SMC4) tuning
    if (c == 'D' || c == 'd') {
      if (currentState == STATE_ACTIVE) {
        dropUntilEol = true;
        Serial.println("ERR: SMC4 tuning only allowed when DISARMED (send '!' first).");
        return;
      }
      float v = Serial.parseFloat();
      if (v < 0.0f) {
        Serial.println("ERR: smcFullK must be >= 0 (negative k causes base velocity anti-damping).");
      } else if (v <= SMC_FULL_K_MAX) {
        smcFullK = v;
      }
      Serial.print("smcFullK="); Serial.println(smcFullK, 2);
      return;
    }

    if (c == 'L' || c == 'l') {
      if (currentState == STATE_ACTIVE) {
        dropUntilEol = true;
        Serial.println("ERR: SMC4 tuning only allowed when DISARMED (send '!' first).");
        return;
      }
      float v = Serial.parseFloat();
      if (v <= 0.0f) {
        Serial.println("ERR: smcFullLambdaTheta must be > 0 (negative/zero makes theta centering unstable).");
      } else if (v <= 50.0f) {
        smcFullLambdaTheta = v;
      }
      Serial.print("smcFullLambdaTheta="); Serial.println(smcFullLambdaTheta, 2);
      return;
    }

    // Base position command (nudge mode)
    if (c == 'T' || c == 't') {
      float v = Serial.parseFloat();
      const float lim = min(THETA_TARGET_LIMIT_DEG, (LIM_MOTOR_DEG - 2.0f));
      thetaTargetDeg = constrain(v, -lim, lim);
      Serial.print("thetaTargetDeg="); Serial.println(thetaTargetDeg, 2);
      return;
    }

    if (c == 'V' || c == 'v') {
      float v = Serial.parseFloat();
      if (v > 0.0f && v <= 200.0f) thetaMoveMaxVelDegS = v;
      Serial.print("thetaMoveMaxVelDegS="); Serial.println(thetaMoveMaxVelDegS, 2);
      return;
    }

    if (c == 'X' || c == 'x') {
      float v = Serial.parseFloat();
      if (v > 0.0f && v <= 2000.0f) thetaMoveMaxAccDegS2 = v;
      Serial.print("thetaMoveMaxAccDegS2="); Serial.println(thetaMoveMaxAccDegS2, 2);
      return;
    }

    // Motor command dead-zone tuning (in steps/s).
    if (c == 'N' || c == 'n') {
      float v = Serial.parseFloat();
      if (v >= 0.0f && v <= 500.0f) speedStopHz = v;
      Serial.print("speedStopHz="); Serial.println(speedStopHz, 2);
      return;
    }

    // Derivative filter cutoff tuning (professor's method)
    if (c == 'W' || c == 'w') {
      float v = Serial.parseFloat();
      if (v >= 1.0f && v <= 2000.0f) omega_c = v;

      updateDerivativeCoeffs();
      Serial.print("Derivative Filter: omega_c="); Serial.print(omega_c, 1);
      Serial.print(" rad/s (f="); Serial.print(omega_c / (2.0f * PI), 1);
      Serial.print(" Hz), b0="); Serial.print(d_b0, 4);
      Serial.print(", a1="); Serial.println(d_a1, 4);

      // Re-init derivative history on next tick for a clean transition.
      derivInit = false;
      return;
    }

  }
}

void setup() {
  Serial.begin(500000);
  Wire.begin();
  Wire.setClock(400000);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(EN_PIN);
  stepper->setAutoEnable(false);

  stepper->setSpeedInHz((uint32_t)MAX_SPEED_HZ);
  stepper->setAcceleration((uint32_t)MAX_ACC_STEPS);
  stepper->disableOutputs();

  Serial.println("--- RIP FULL-STATE FEEDBACK (Section 11) ---");
  Serial.println("Quick run:");
  Serial.println("  Z  -> Calibrate (arm centered; pendulum UPRIGHT)");
  Serial.println("  E  -> Arm/disarm (auto-engages when upright & still)");
  Serial.println("Motion:");
  Serial.println("  T <deg>     -> Base target angle (deg)");
  Serial.println("  V <deg/s>   -> Move max velocity");
  Serial.println("  X <deg/s^2> -> Move max acceleration");
  Serial.println("Control (SMC commands only when DISARMED):");
  Serial.println("  C <0|1|2>   -> Controller mode (0=linear, 1=SMC, 2=SMC4)");
  Serial.println("  J <val>     -> SMC lambda (1/s)");
  Serial.println("  K <val>     -> SMC K (deg/s^2)");
  Serial.println("  P <val>     -> SMC phi (deg/s)");
  Serial.println("  Q <+1|-1>   -> SMC sign flip");
  Serial.println("  O <0..2>    -> Base-centering assist scale (SMC/SMC4)");
  Serial.println("  D <val>     -> SMC4 k (>=0; thetaDot coupling)");
  Serial.println("  L <val>     -> SMC4 lambda_theta (1/s; must be > 0)");
  Serial.println("Useful:");
  Serial.println("  S (sign wizard), G (print config), R (clear EEPROM), Y (save), W/N/U (tuning)");
  Serial.println("Log: t_ms,alphaRaw100,alphaDot100,theta100,thetaDot100,accCmd,velCmd,posMeasSteps,clamped");

  bool loaded = loadPersistedConfig();
  Serial.print("[EEPROM] ");
  Serial.println(loaded ? "Loaded saved settings" : "No saved settings; using defaults");

  updateDerivativeCoeffs();

  Serial.print("motorSign="); Serial.print(motorSign);
  Serial.print(" ALPHA_SIGN="); Serial.print(ALPHA_SIGN);
  Serial.print(" THETA_SIGN="); Serial.print(THETA_SIGN);
  Serial.print(" CTRL_SIGN="); Serial.println(CTRL_SIGN);
  Serial.print("speedStopHz="); Serial.println(speedStopHz, 2);
  Serial.print("K_THETA="); Serial.print(K_THETA,2);
  Serial.print(" K_ALPHA="); Serial.print(K_ALPHA,2);
  Serial.print(" K_THETADOT="); Serial.print(K_THETADOT,2);
  Serial.print(" K_ALPHADOT="); Serial.println(K_ALPHADOT,2);

  Serial.print("Derivative Filter: omega_c="); Serial.print(omega_c,1);
  Serial.print(" rad/s (f="); Serial.print(omega_c/(2.0f*PI),1);
  Serial.print(" Hz), b0="); Serial.print(d_b0,4);
  Serial.print(", a1="); Serial.println(d_a1,4);

  lastUs = micros();
}

void loop() {
  handleSerial();

  uint32_t nowUs = micros();
  uint32_t dtUs = nowUs - lastUs;
  if (dtUs < LOOP_US) return;
  lastUs = nowUs;

  float dt = dtUs * 1e-6f;

  // Pendulum
  selectMux(0);
  float pendDeg = readAS5600Deg();
  int pendErr = sensor.lastError();
  if (currentState == STATE_ACTIVE && pendErr != AS5600_OK) dbgAlphaI2cErr++;
  float alphaDegRaw = getAngleDiffDeg(pendDeg, targetRawDeg);
  float alphaRawSigned = (float)ALPHA_SIGN * alphaDegRaw;

  // Base
  selectMux(1);
  float baseAbsDeg = readAS5600Deg();
  int baseErr = sensor.lastError();
  if (currentState == STATE_ACTIVE && baseErr != AS5600_OK) dbgThetaI2cErr++;
  float baseDegRaw = getAngleDiffDeg(baseAbsDeg, motorZeroDeg);
  float baseDeg = baseDegRaw; // for safety check
  float thetaDeg = THETA_SIGN * baseDegRaw;

  // Glitch rejection on raw angles (prevents random I2C/mux jumps from nuking control).
  if (derivInit) {
    float dAlphaPre = alphaRawSigned - last_alpha_raw;
    while (dAlphaPre > 180.0f) dAlphaPre -= 360.0f;
    while (dAlphaPre < -180.0f) dAlphaPre += 360.0f;
    if (fabs(dAlphaPre) > ALPHA_JUMP_REJECT_DEG) {
      if (currentState == STATE_ACTIVE) dbgAlphaGlitchReject++;
      alphaRawSigned = last_alpha_raw;
      alphaDegRaw = alphaRawSigned * (float)ALPHA_SIGN;
    }

    float dThetaPre = thetaDeg - last_theta_raw;
    if (fabs(dThetaPre) > THETA_JUMP_REJECT_DEG) {
      if (currentState == STATE_ACTIVE) dbgThetaGlitchReject++;
      thetaDeg = last_theta_raw;
      baseDegRaw = thetaDeg * (float)THETA_SIGN;
      baseDeg = baseDegRaw;
    }
  }

  switch (currentState) {

    case STATE_CALIBRATE: {
      // wait for Z
    } break;

    case STATE_IDLE: {
      if (firstIdle) {
        resetController();
        firstIdle = false;
        stepper->stopMove();
        stepper->disableOutputs();
      }

      // One-shot derivative init (initialize history)
      if (!derivInit) {
        last_alpha_raw = alphaRawSigned;
        last_alpha_dot = 0.0f;
        last_theta_raw = thetaDeg;
        last_theta_dot = 0.0f;
        alphaDotFilt = 0.0f;
        thetaDotFilt = 0.0f;
        thetaDegPrevForStall = thetaDeg;
        stallNoMoveTicks = 0;
        stallWarned = false;
        derivInit = true;
      }

      // ============================================================
      // PROFESSOR'S DERIVATIVE FILTER METHOD
      // Formula: y[n] = b0*(x[n] - x[n-1]) - a1*y[n-1]
      // ============================================================

      // Alpha derivative (with angle wrap-around handling)
      float dAlpha = alphaRawSigned - last_alpha_raw;
      // Handle ±180° discontinuities
      while (dAlpha > 180.0f) dAlpha -= 360.0f;
      while (dAlpha < -180.0f) dAlpha += 360.0f;
      
      if (fabs(dAlpha) > ALPHA_JUMP_REJECT_DEG) dAlpha = 0.0f;

      alphaDotFilt = d_b0 * dAlpha - d_a1 * last_alpha_dot;
      alphaDotFilt = constrain(alphaDotFilt, -ALPHADOT_CLAMP, ALPHADOT_CLAMP);
      
      last_alpha_raw = alphaRawSigned;
      last_alpha_dot = alphaDotFilt;

      // Theta derivative (continuous, no wrap-around needed)
      float dTheta = thetaDeg - last_theta_raw;
      
      if (fabs(dTheta) > THETA_JUMP_REJECT_DEG) dTheta = 0.0f;

      thetaDotFilt = d_b0 * dTheta - d_a1 * last_theta_dot;
      thetaDotFilt = constrain(thetaDotFilt, -THETADOT_CLAMP, THETADOT_CLAMP);
      
      last_theta_raw = thetaDeg;
      last_theta_dot = thetaDotFilt;

      bool ok = (fabs(alphaDegRaw) < ENGAGE_PEND_DEG) && (fabs(alphaDotFilt) < ENGAGE_DOT_DEG);

      static uint32_t dec2 = 0;
      if (!armEnabled) {
        dec2 = 0;
      } else if (++dec2 >= 200) {  // 1 Hz at 200 Hz loop
        dec2 = 0;
        Serial.print("[IDLE] alpha="); Serial.print(alphaDegRaw, 2);
        Serial.print(" alphaDot="); Serial.print(alphaDotFilt, 1);
        Serial.print(" | theta="); Serial.print(thetaDeg, 2);
        Serial.print(" thetaDot="); Serial.print(thetaDotFilt, 1);
        Serial.print(" | enable="); Serial.print(ok ? 1 : 0);
        Serial.print(" cnt="); Serial.println(engageCount);
      }

      if (ok) engageCount++;
      else engageCount = 0;

      if (isCalibrated && armEnabled && engageCount >= ENGAGE_COUNT_REQ) {
        // Keep warm derivatives; reset only controller integrators/commands
        thetaDotCmdMotor = 0.0f;
        runDir = 0;
        fallCount = 0;
        lastAccCmdDiag = 0;

        logDecim = 0;
        engageStartMs = millis();
        currentState = STATE_ACTIVE;

        stepper->enableOutputs();
        stepper->stopMove(); // ensure clean start
        Serial.println("ENGAGED!");
        engageCount = 0;
      }
    } break;

    case STATE_ACTIVE: {
      // Track timing jitter inside ACTIVE (helps decide whether filter coeffs need per-tick recompute).
      dbgTicksActive++;
      if (dtUs < dbgDtUsMin) dbgDtUsMin = dtUs;
      if (dtUs > dbgDtUsMax) dbgDtUsMax = dtUs;
      if (dtUs > (LOOP_US + 1000)) dbgDtOverrun++;

      // Allow runtime derivative tuning without requiring an idle transition.
      if (!derivInit) {
        last_alpha_raw = alphaRawSigned;
        last_alpha_dot = 0.0f;
        last_theta_raw = thetaDeg;
        last_theta_dot = 0.0f;
        alphaDotFilt = 0.0f;
        thetaDotFilt = 0.0f;
        derivInit = true;
      }

      const bool outOfRange = (fabs(baseDeg) > LIM_MOTOR_DEG) || (fabs(alphaDegRaw) > LIM_PEND_DEG);
      if (outOfRange) {
        if (++fallCount >= FALL_COUNT_REQ) {
          Serial.print("FALLEN limit=");
          if (fabs(alphaDegRaw) > LIM_PEND_DEG) Serial.print("alpha ");
          if (fabs(baseDeg) > LIM_MOTOR_DEG) Serial.print("base ");
          Serial.print("| alpha="); Serial.print(alphaRawSigned, 2);
          Serial.print(" (pendRaw="); Serial.print(pendDeg, 2);
          Serial.print(" targetRaw="); Serial.print(targetRawDeg, 2);
          Serial.print(" cal="); Serial.print(targetRawDegCal, 2);
          Serial.print(") | theta="); Serial.print(thetaDeg, 2);
          Serial.print(" (baseRaw="); Serial.print(baseAbsDeg, 2);
          Serial.print(" zero="); Serial.print(motorZeroDeg, 2);
          Serial.print(") | alphaDot="); Serial.print(alphaDotFilt, 1);
          Serial.print(" thetaDot="); Serial.print(thetaDotFilt, 1);
          Serial.print(" | accPrev="); Serial.print(lastAccCmdDiag);
          Serial.print(" velCmd="); Serial.print((int)thetaDotCmdMotor);
          Serial.print(" runDir="); Serial.println(runDir);

          enterIdle(true);  // Keep armEnabled=true for auto-rearm when pendulum returns to window
          Serial.println("FALLEN (will auto-rearm when upright)");
          fallCount = 0;
          break;
        }
      } else {
        fallCount = 0;
      }

      // Upright-only nonlinear controller validity window (no swing-up).
      // Abort immediately rather than relying on cos clamps or the wider mechanical limits.
      if (ctrlMode != CTRL_LINEAR && fabs(alphaRawSigned) > SMC_ALPHA_ABORT_DEG) {
        Serial.print("FALLEN limit=smc_alpha");
        Serial.print(" | alpha="); Serial.print(alphaRawSigned, 2);
        Serial.print(" alphaDot="); Serial.print(alphaDotFilt, 1);
        Serial.print(" | theta="); Serial.print(thetaDeg, 2);
        Serial.print(" thetaDot="); Serial.println(thetaDotFilt, 1);
        enterIdle(true);
        Serial.println("FALLEN (will auto-rearm when upright)");
        fallCount = 0;
        break;
      }

      // ============================================================
      // PROFESSOR'S DERIVATIVE FILTER METHOD
      // Formula: y[n] = b0*(x[n] - x[n-1]) - a1*y[n-1]
      // ============================================================

      // Alpha derivative (with angle wrap-around handling)
      float dAlpha = alphaRawSigned - last_alpha_raw;
      // Handle ±180° discontinuities
      while (dAlpha > 180.0f) dAlpha -= 360.0f;
      while (dAlpha < -180.0f) dAlpha += 360.0f;
      
      if (fabs(dAlpha) > ALPHA_JUMP_REJECT_DEG) dAlpha = 0.0f;
      if (dAlpha == 0.0f && fabs(alphaRawSigned - last_alpha_raw) > ALPHA_JUMP_REJECT_DEG) dbgAlphaDerivReject++;

      alphaDotFilt = d_b0 * dAlpha - d_a1 * last_alpha_dot;
      alphaDotFilt = constrain(alphaDotFilt, -ALPHADOT_CLAMP, ALPHADOT_CLAMP);
      
      last_alpha_raw = alphaRawSigned;
      last_alpha_dot = alphaDotFilt;

      // Theta derivative (continuous, no wrap-around needed)
      float dTheta = thetaDeg - last_theta_raw;
      
      if (fabs(dTheta) > THETA_JUMP_REJECT_DEG) dTheta = 0.0f;
      if (dTheta == 0.0f && fabs(thetaDeg - last_theta_raw) > THETA_JUMP_REJECT_DEG) dbgThetaDerivReject++;

      thetaDotFilt = d_b0 * dTheta - d_a1 * last_theta_dot;
      thetaDotFilt = constrain(thetaDotFilt, -THETADOT_CLAMP, THETADOT_CLAMP);
      
      last_theta_raw = thetaDeg;
      last_theta_dot = thetaDotFilt;

      // Additional upright-only SMC abort: extreme alphaDot inside the angle window tends to rail accel
      // and can walk the base. Debounced to avoid single-sample noise.
      if (ctrlMode != CTRL_LINEAR) {
        const bool abortGraceOver = (millis() - engageStartMs) > SMC_ABORT_GRACE_MS;
        if (!abortGraceOver) {
          smcAlphaDotAbortTicks = 0;
        } else if (fabs(alphaDotFilt) > SMC_ALPHADOT_ABORT_DEG_S &&
                   fabs(alphaRawSigned) > SMC_ALPHADOT_ABORT_MIN_ALPHA_DEG) {
          if (smcAlphaDotAbortTicks < 255) smcAlphaDotAbortTicks++;
          if (smcAlphaDotAbortTicks >= SMC_ALPHADOT_ABORT_TICKS_REQ) {
            Serial.print("FALLEN limit=smc_alphaDot");
            Serial.print(" | alpha="); Serial.print(alphaRawSigned, 2);
            Serial.print(" alphaDot="); Serial.print(alphaDotFilt, 1);
            Serial.print(" | theta="); Serial.print(thetaDeg, 2);
            Serial.print(" thetaDot="); Serial.println(thetaDotFilt, 1);
            enterIdle(true);
            Serial.println("FALLEN (will auto-rearm when upright)");
            fallCount = 0;
            smcAlphaDotAbortTicks = 0;
            break;
          }
        } else {
          smcAlphaDotAbortTicks = 0;
        }
      } else {
        smcAlphaDotAbortTicks = 0;
      }

      // Base motion watchdog: if we're commanding significant speed but theta doesn't move,
      // this strongly suggests a motor/driver/mechanical dropout (or the base sensor is stuck).
      float dThetaMon = thetaDeg - thetaDegPrevForStall;
      while (dThetaMon > 180.0f) dThetaMon -= 360.0f;
      while (dThetaMon < -180.0f) dThetaMon += 360.0f;

      const bool expectMotion = (runDir != 0) && (fabs(thetaDotCmdMotor) >= BASE_STALL_CMD_HZ);
      const bool movedEnough = fabs(dThetaMon) >= BASE_STALL_MIN_DTHETA_DEG;

      if (expectMotion && !movedEnough) {
        stallNoMoveTicks++;
        dbgBaseStallTicks++;

        if (!stallWarned && stallNoMoveTicks >= BASE_STALL_WARN_TICKS) {
          stallWarned = true;
          dbgBaseStallEvents++;

          // Read magnet diagnostics for the BASE sensor (mux=1) while we're here.
          // (No I2C error will be raised if the magnet is misaligned; the STATUS bits and MAGNITUDE help diagnose that.)
          uint8_t st = sensor.readStatus();
          uint8_t agc = sensor.readAGC();
          uint16_t mag = sensor.readMagnitude();
          const int md = (st & AS5600_STATUS_MAGNET_DETECT) ? 1 : 0;
          const int hi = (st & AS5600_STATUS_MAGNET_HIGH) ? 1 : 0;
          const int lo = (st & AS5600_STATUS_MAGNET_LOW) ? 1 : 0;

          Serial.print("[WARN] BASE_STALL? dTheta="); Serial.print(dThetaMon, 3);
          Serial.print(" deg | velCmd="); Serial.print((int)thetaDotCmdMotor);
          Serial.print(" runDir="); Serial.print(runDir);
          Serial.print(" | theta="); Serial.print(thetaDeg, 2);
          Serial.print(" thetaDot="); Serial.print(thetaDotFilt, 1);
          Serial.print(" | baseRaw="); Serial.print(baseAbsDeg, 2);
          Serial.print(" zero="); Serial.print(motorZeroDeg, 2);
          Serial.print(" | mag="); Serial.print(mag);
          Serial.print(" agc="); Serial.print(agc);
          Serial.print(" st=0x"); Serial.print(st, HEX);
          Serial.print(" md/hi/lo="); Serial.print(md);
          Serial.print("/"); Serial.print(hi);
          Serial.print("/"); Serial.print(lo);
          Serial.print(" | alpha="); Serial.print(alphaRawSigned, 2);
          Serial.print(" alphaDot="); Serial.print(alphaDotFilt, 1);
          Serial.print(" | accPrev="); Serial.println(lastAccCmdDiag);
        }
      } else {
        stallNoMoveTicks = 0;
        stallWarned = false;
      }
      thetaDegPrevForStall = thetaDeg;

      // ============================================================
      // BASE POSITION REFERENCE ("NUDGE" MODE): trapezoidal theta_ref
      // ============================================================
      const float distNow = getAngleDiffDeg(thetaTargetDeg, thetaRefDeg);
      const bool moveInProgress = movePaused ||
          (fabs(distNow) > THETA_REF_SNAP_EPS_DEG) ||
          (fabs(thetaRefVelDegS) > THETA_REF_SNAP_VEL_EPS_DEG_S);

      const bool moveStable =
          (fabs(alphaRawSigned) < MOVE_ALPHA_WINDOW_DEG) &&
          (fabs(alphaDotFilt) < MOVE_ALPHADOT_WINDOW_DEG_S);

      const bool engageGraceOver = (millis() - engageStartMs) > MOVE_ENGAGE_GRACE_MS;
      const bool stableNow = engageGraceOver && moveStable;

      if (moveInProgress) {
        if (movePaused) {
          if (stableNow) {
            if (moveStableTicks < 255) moveStableTicks++;
            if (moveStableTicks >= MOVE_STABLE_TICKS_REQ) {
              movePaused = false;
              moveStableTicks = 0;
              moveUnstableTicks = 0;
              Serial.println("[MOVE] RESUMED");
            }
          } else {
            moveStableTicks = 0;
          }
        } else {
          if (!stableNow) {
            // Freeze the reference kinematics while we debounce instability (prevents stale refVel/refAcc usage).
            thetaRefVelDegS = 0.0f;
            thetaRefAccDegS2 = 0.0f;

            if (moveUnstableTicks < 255) moveUnstableTicks++;
            moveStableTicks = 0;
            if (moveUnstableTicks >= MOVE_UNSTABLE_TICKS_REQ) {
              movePaused = true;
              moveUnstableTicks = 0;
              moveStableTicks = 0;
              thetaRefDeg = thetaDeg;  // one-shot handoff (no reference chasing while paused)
              thetaRefVelDegS = 0.0f;
              thetaRefAccDegS2 = 0.0f;
              Serial.println("[MOVE] PAUSED");
            }
          } else {
            moveUnstableTicks = 0;
            stepThetaReferenceTrapezoid(dt);
          }
        }
      } else {
        movePaused = false;
        moveStableTicks = 0;
        moveUnstableTicks = 0;
        thetaRefDeg = thetaTargetDeg;
        thetaRefVelDegS = 0.0f;
        thetaRefAccDegS2 = 0.0f;
      }

      const bool moveActive = moveInProgress && !movePaused;

      const float thetaErr = getAngleDiffDeg(thetaDeg, thetaRefDeg);
      const float thetaDotErr = thetaDotFilt - thetaRefVelDegS;
      const float accFF_steps_s2 = STEPS_PER_DEG * thetaRefAccDegS2;

      // ============================================================
      // CONTROL LAW:
      //  - CTRL_LINEAR: full-state feedback about upright (Section 11.7)
      //  - CTRL_SMC: nonlinear sliding mode control about upright (no swing-up)
      //  - CTRL_SMC_FULL: full-state surface SMC (alpha/alphaDot/thetaErr/thetaDotErr)
      // ============================================================
      float accCmdPhysical = 0.0f;  // steps/s^2 (physical coords, before motorSign)
      float smcS = 0.0f;            // deg/s (diag)
      float thetaDDotSmcDegS2 = 0.0f;  // deg/s^2 (diag)
      float accSmcStepsS2Diag = 0.0f;
      float accBaseStepsS2Diag = 0.0f;
      int smcBaseGateDiag = 0;
      float smcFullDenDiag = 0.0f;
      float smcFullKEffDiag = 0.0f;
      float smcFullKRampDiag = 0.0f;
      float smcFullThetaErrCDiag = 0.0f;
      float smcFullThetaTermDiag = 0.0f;

      if (ctrlMode == CTRL_SMC) {
        const float deg2rad = PI / 180.0f;
        const float rad2deg = 180.0f / PI;

        const float alphaDeg = alphaRawSigned;
        const float alphaDotDegS = alphaDotFilt;
        const float thetaDotDegS = thetaDotFilt;  // measured (not commanded)

        const float alphaRad = alphaDeg * deg2rad;
        const float sinA = sin(alphaRad);
        const float cosA = cos(alphaRad);
        const float cosSafe = copysignf(max(fabs(cosA), SMC_COS_MIN), cosA);

        const float A_deg = MODEL_A * rad2deg;  // deg/s^2
        const float fDeg =
            A_deg * sinA +
            (sinA * cosA) * (thetaDotDegS * thetaDotDegS) * deg2rad;

        smcS = alphaDotDegS + smcLambda * alphaDeg;
        const float phi = max(smcPhiDegS, 1e-3f);
        const float smcSat = constrain(smcS / phi, -1.0f, 1.0f);

        thetaDDotSmcDegS2 =
            (fDeg + smcLambda * alphaDotDegS + smcKDegS2 * smcSat) /
            (MODEL_B * cosSafe);

        thetaDDotSmcDegS2 *= smcSign;
        thetaDDotSmcDegS2 = constrain(thetaDDotSmcDegS2, -THETA_DDOT_MAX_DEG_S2, THETA_DDOT_MAX_DEG_S2);

        const float accSMC_steps_s2 = STEPS_PER_DEG * thetaDDotSmcDegS2;
        accSmcStepsS2Diag = accSMC_steps_s2;

        // Secondary objective: gentle base tracking (reference + feedforward), optionally gated by stability.
        float accBase_steps_s2 =
            accFF_steps_s2 +
            K_THETA * thetaErr +
            K_THETADOT * thetaDotErr;

        accBase_steps_s2 *= constrain(smcBaseScale, 0.0f, 2.0f);
        const bool smcBaseGate =
            engageGraceOver &&
            (fabs(alphaRawSigned) < SMC_BASE_GATE_ALPHA_WINDOW_DEG) &&
            (fabs(alphaDotFilt) < SMC_BASE_GATE_ALPHADOT_WINDOW_DEG_S);
        if (!smcBaseGate) accBase_steps_s2 = 0.0f;
        accBaseStepsS2Diag = accBase_steps_s2;
        smcBaseGateDiag = smcBaseGate ? 1 : 0;

        accCmdPhysical = accSMC_steps_s2 + accBase_steps_s2;
      } else if (ctrlMode == CTRL_SMC_FULL) {
        const float deg2rad = PI / 180.0f;
        const float rad2deg = 180.0f / PI;

        const float alphaDeg = alphaRawSigned;
        const float alphaDotDegS = constrain(alphaDotFilt, -SMC4_ALPHA_DOT_CTRL_CLAMP_DEG_S, SMC4_ALPHA_DOT_CTRL_CLAMP_DEG_S);
        const float thetaDotDegS = thetaDotFilt;  // measured (not commanded)

        const float alphaRad = alphaDeg * deg2rad;
        const float sinA = sin(alphaRad);
        const float cosA = cos(alphaRad);
        const float cosSafe = copysignf(max(fabs(cosA), SMC_COS_MIN), cosA);

        const float A_deg = MODEL_A * rad2deg;  // deg/s^2
        const float fDeg =
            A_deg * sinA +
            (sinA * cosA) * (thetaDotDegS * thetaDotDegS) * deg2rad;

        const float thetaErrC =
            constrain(thetaErr, -SMC_FULL_THETA_ERR_CLAMP_DEG, SMC_FULL_THETA_ERR_CLAMP_DEG);
        const float thetaDotErrC =
            constrain(thetaDotErr, -SMC_FULL_THETADOT_ERR_CLAMP_DEG_S, SMC_FULL_THETADOT_ERR_CLAMP_DEG_S);
        float thetaTerm = thetaDotErrC + smcFullLambdaTheta * thetaErrC;
        thetaTerm = constrain(thetaTerm, -SMC_FULL_THETA_TERM_CLAMP_DEG_S, SMC_FULL_THETA_TERM_CLAMP_DEG_S);
        smcFullThetaErrCDiag = thetaErrC;
        smcFullThetaTermDiag = thetaTerm;

        const float kRamp = constrain((float)(millis() - engageStartMs) * (1.0f / (float)SMC_FULL_K_RAMP_MS), 0.0f, 1.0f);
        float kReq = constrain(smcFullK, 0.0f, SMC_FULL_K_MAX) * kRamp;

        float kEff = kReq;
        float den = (MODEL_B * cosSafe) - kEff;
        // Denominator safety: never divide by a too-small effective input coefficient.
        if (fabs(den) < SMC_FULL_DENOM_MIN) {
          if (den >= 0.0f) {
            kEff = (MODEL_B * cosSafe) - SMC_FULL_DENOM_MIN;
            den = SMC_FULL_DENOM_MIN;
          } else {
            kEff = (MODEL_B * cosSafe) + SMC_FULL_DENOM_MIN;
            den = -SMC_FULL_DENOM_MIN;
          }
        }

        smcFullDenDiag = den;
        smcFullKEffDiag = kEff;
        smcFullKRampDiag = kRamp;

        smcS = alphaDotDegS + smcLambda * alphaDeg + kEff * thetaTerm;
        const float phi = max(smcPhiDegS, 1e-3f);
        const float smcSat = constrain(smcS / phi, -1.0f, 1.0f);

        thetaDDotSmcDegS2 =
            (fDeg +
             smcLambda * alphaDotDegS +
             (kEff * smcFullLambdaTheta * thetaDotErrC) -
             (kEff * thetaRefAccDegS2) +
             smcKDegS2 * smcSat) /
            den;

        thetaDDotSmcDegS2 *= smcSign;
        thetaDDotSmcDegS2 = constrain(thetaDDotSmcDegS2, -THETA_DDOT_MAX_DEG_S2, THETA_DDOT_MAX_DEG_S2);

        const float accSMC_steps_s2 = STEPS_PER_DEG * thetaDDotSmcDegS2;
        accSmcStepsS2Diag = accSMC_steps_s2;

        // Base centering assist (same mechanism as hybrid SMC): keeps theta near thetaRef, prevents random-walk drift.
        // This is gated so it doesn't fight recovery when the pendulum is far from upright.
        float accBase_steps_s2 =
            accFF_steps_s2 +
            K_THETA * thetaErr +
            K_THETADOT * thetaDotErr;

        accBase_steps_s2 *= constrain(smcBaseScale, 0.0f, 2.0f);
        const bool smcBaseGate =
            engageGraceOver &&
            (fabs(alphaRawSigned) < SMC_BASE_GATE_ALPHA_WINDOW_DEG) &&
            (fabs(alphaDotFilt) < SMC4_BASE_GATE_ALPHADOT_WINDOW_DEG_S);
        if (!smcBaseGate) accBase_steps_s2 = 0.0f;
        accBaseStepsS2Diag = accBase_steps_s2;
        smcBaseGateDiag = smcBaseGate ? 1 : 0;

        accCmdPhysical = accSMC_steps_s2 + accBase_steps_s2;
      } else {
        // FULL-STATE FEEDBACK CONTROLLER (Section 11.7 modelling_complete.md)
        // Control law (reference tracking):
        //   accCmdPhysical = accFF + K_THETA*(theta-thetaRef) + K_ALPHA*alpha
        //                  + K_THETADOT*(thetaDot-thetaRefDot) + K_ALPHADOT*alphaDot
        float alphaCtrl = alphaRawSigned;
        if (fabs(alphaCtrl) < ALPHA_DEADBAND_DEG) alphaCtrl = 0.0f;

        accCmdPhysical =
            accFF_steps_s2 +
            K_THETA * thetaErr + K_ALPHA * alphaCtrl +
            K_THETADOT * thetaDotErr + K_ALPHADOT * alphaDotFilt;
      }

      // Apply control sign convention
      accCmdPhysical *= CTRL_SIGN;

      // ramp on engage
      float ramp = (millis() - engageStartMs) * (1.0f / 100.0f);
      if (ramp > 1.0f) ramp = 1.0f;
      accCmdPhysical *= ramp;

      const float accLimitStepsS2 =
          (ctrlMode == CTRL_SMC_FULL) ? min(SMC4_MAX_ACC_STEPS, MAX_ACC_STEPS) : MAX_ACC_STEPS;

      int sat = 0;
      if (accCmdPhysical >  accLimitStepsS2) { accCmdPhysical =  accLimitStepsS2; sat = 1; }
      if (accCmdPhysical < -accLimitStepsS2) { accCmdPhysical = -accLimitStepsS2; sat = 1; }
      lastAccCmdDiag = (int32_t)accCmdPhysical;
      if (sat) dbgAccSat++;

      // Auto-trim pendulum reference when stable (kills small alpha bias that otherwise creates large theta offset).
      if (!sat &&
          !moveActive &&
          fabs(alphaRawSigned) < ALPHA_TRIM_ALPHA_WINDOW_DEG &&
          fabs(alphaDotFilt) < ALPHA_TRIM_ALPHADOT_WINDOW_DEG_S) {
        dbgTrimActiveTicks++;
        targetRawDeg = wrap360(targetRawDeg + (ALPHA_TRIM_RATE * alphaDegRaw * dt));
        float trDiff = getAngleDiffDeg(targetRawDeg, targetRawDegCal);
        if (trDiff > ALPHA_TRIM_MAX_DEG) { targetRawDeg = wrap360(targetRawDegCal + ALPHA_TRIM_MAX_DEG); dbgTrimClampedTicks++; }
        if (trDiff < -ALPHA_TRIM_MAX_DEG) { targetRawDeg = wrap360(targetRawDegCal - ALPHA_TRIM_MAX_DEG); dbgTrimClampedTicks++; }
      }

      // Apply motor sign
      float accCmdMotor = motorSign * accCmdPhysical;

      // Integrate accel -> desired speed (leaky integrator near upright to prevent slow drift).
      float leak = 0.0f;
      if (ctrlMode == CTRL_LINEAR &&
          !moveActive &&
          VEL_LEAK > 0.0f &&
          fabs(alphaRawSigned) < VEL_LEAK_ALPHA_WINDOW_DEG &&
          fabs(thetaErr) < VEL_LEAK_THETA_WINDOW_DEG) {
        leak = VEL_LEAK;
        dbgLeakActiveTicks++;
      }
      thetaDotCmdMotor += (accCmdMotor - leak * thetaDotCmdMotor) * dt;
      const float speedLimitHz =
          (ctrlMode == CTRL_SMC_FULL) ? min(SMC4_MAX_SPEED_HZ, MAX_SPEED_HZ) : MAX_SPEED_HZ;
      thetaDotCmdMotor = constrain(thetaDotCmdMotor, -speedLimitHz, speedLimitHz);

      // soft limit using MEASURED theta
      const float LIM_MARGIN_DEG = 2.0f;
      if ((thetaDeg > (LIM_MOTOR_DEG - LIM_MARGIN_DEG) && thetaDotCmdMotor > 0) ||
          (thetaDeg < (-LIM_MOTOR_DEG + LIM_MARGIN_DEG) && thetaDotCmdMotor < 0)) {
        thetaDotCmdMotor = 0.0f;
      }

      // Command continuous velocity
      commandMotorSpeed(thetaDotCmdMotor);
      if (runDir != 0) dbgMotorRunningTicks++;

      // Human-readable status (1 Hz for monitoring)
      static uint32_t statusDecim = 0;
      if (++statusDecim >= 200) {  // Every 200 loops = 1 Hz at 200 Hz
        statusDecim = 0;
        Serial.print("[STATUS] alpha="); Serial.print(alphaRawSigned,2);
        Serial.print(" alphaDot="); Serial.print(alphaDotFilt,1);
        Serial.print(" | theta="); Serial.print(thetaDeg,2);
        Serial.print(" thetaDot="); Serial.print(thetaDotFilt,1);
        Serial.print(" | acc="); Serial.print((int)accCmdPhysical);
        Serial.print(" vel="); Serial.print((int)thetaDotCmdMotor);
        Serial.print(" | ref="); Serial.print(thetaRefDeg, 2);
        Serial.print(" tgt="); Serial.print(thetaTargetDeg, 2);
        Serial.print(" paused="); Serial.print(movePaused ? 1 : 0);
        Serial.print(" mode=");
        if (ctrlMode == CTRL_SMC) Serial.print("SMC");
        else if (ctrlMode == CTRL_SMC_FULL) Serial.print("SMC4");
        else Serial.print("LIN");
        if (ctrlMode == CTRL_SMC) {
          Serial.print(" s="); Serial.print(smcS, 1);
          Serial.print(" thetaDDot="); Serial.print(thetaDDotSmcDegS2, 0);
          Serial.print(" accSMC="); Serial.print((int)accSmcStepsS2Diag);
          Serial.print(" accBase="); Serial.print((int)accBaseStepsS2Diag);
          Serial.print(" gate="); Serial.print(smcBaseGateDiag);
        } else if (ctrlMode == CTRL_SMC_FULL) {
          Serial.print(" s="); Serial.print(smcS, 1);
          Serial.print(" thetaDDot="); Serial.print(thetaDDotSmcDegS2, 0);
          Serial.print(" accSMC="); Serial.print((int)accSmcStepsS2Diag);
          Serial.print(" accBase="); Serial.print((int)accBaseStepsS2Diag);
          Serial.print(" gate="); Serial.print(smcBaseGateDiag);
          Serial.print(" den="); Serial.print(smcFullDenDiag, 2);
          Serial.print(" kEff="); Serial.print(smcFullKEffDiag, 2);
          Serial.print(" kRamp="); Serial.print(smcFullKRampDiag, 2);
          Serial.print(" thetaErrC="); Serial.print(smcFullThetaErrCDiag, 1);
          Serial.print(" thetaTerm="); Serial.print(smcFullThetaTermDiag, 1);
        }
        Serial.println();

        // Low-rate diagnostics so we can tell what protections/nonlinearities are active.
        uint32_t ticks = (dbgTicksActive > 0) ? dbgTicksActive : 1;
        uint32_t runPct = (100UL * dbgMotorRunningTicks) / ticks;
        uint32_t leakPct = (100UL * dbgLeakActiveTicks) / ticks;
        uint32_t trimPct = (100UL * dbgTrimActiveTicks) / ticks;
        float trimDiff = getAngleDiffDeg(targetRawDeg, targetRawDegCal);

        Serial.print("[DBG] dtUsMin="); Serial.print(dbgDtUsMin);
        Serial.print(" dtUsMax="); Serial.print(dbgDtUsMax);
        Serial.print(" over="); Serial.print(dbgDtOverrun);
        Serial.print(" sat="); Serial.print(dbgAccSat);
        Serial.print(" | run%="); Serial.print(runPct);
        Serial.print(" start="); Serial.print(dbgMotorStart);
        Serial.print(" stop="); Serial.print(dbgMotorStop);
        Serial.print(" rev="); Serial.print(dbgMotorReverse);
        Serial.print(" | leak%="); Serial.print(leakPct);
        Serial.print(" trim%="); Serial.print(trimPct);
        Serial.print(" trimDiff="); Serial.print(trimDiff, 2);
        Serial.print(" trimClamp="); Serial.print(dbgTrimClampedTicks);
        Serial.print(" | glitchA/T="); Serial.print(dbgAlphaGlitchReject);
        Serial.print("/"); Serial.print(dbgThetaGlitchReject);
        Serial.print(" derivA/T="); Serial.print(dbgAlphaDerivReject);
        Serial.print("/"); Serial.print(dbgThetaDerivReject);
        Serial.print(" i2cA/T="); Serial.print(dbgAlphaI2cErr);
        Serial.print("/"); Serial.print(dbgThetaI2cErr);
        Serial.print(" stallE/T="); Serial.print(dbgBaseStallEvents);
        Serial.print("/"); Serial.println(dbgBaseStallTicks);
      }

      // Logging
      if (++logDecim >= 4) {  // 50 Hz
        logDecim = 0;
        uint32_t tMs = millis();
        int32_t alphaRaw100 = (int32_t)(alphaRawSigned * 100.0f);
        int32_t alphaDot100 = (int32_t)(alphaDotFilt * 100.0f);
        int32_t theta100 = (int32_t)(thetaDeg * 100.0f);
        int32_t thetaDot100 = (int32_t)(thetaDotFilt * 100.0f);
        int32_t accCmd = (int32_t)(accCmdPhysical);
        int32_t velCmd = (int32_t)(thetaDotCmdMotor);
        int32_t posMeasSteps = (int32_t)(thetaDeg * STEPS_PER_DEG);
        int clamped = (fabs(thetaDeg) >= (LIM_MOTOR_DEG - 0.5f)) ? 1 : 0;

        Serial.print(tMs); Serial.print(",");
        Serial.print(alphaRaw100); Serial.print(",");
        Serial.print(alphaDot100); Serial.print(",");
        Serial.print(theta100); Serial.print(",");
        Serial.print(thetaDot100); Serial.print(",");
        Serial.print(accCmd); Serial.print(",");
        Serial.print(velCmd); Serial.print(",");
        Serial.print(posMeasSteps); Serial.print(",");
        Serial.println(clamped);
      }
    } break;
  }
}
