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

// Full-state feedback gains (from Section 11.7 of modelling_complete.md)
// Control law: ddot_theta = K_THETA*theta + K_ALPHA*alpha + K_THETADOT*thetaDot + K_ALPHADOT*alphaDot
// NOTE: Gains are expressed as positive magnitudes; use ALPHA_SIGN / THETA_SIGN / CTRL_SIGN
// to match the firmware state definitions to your physical directions.
// NOTE: K_THETA increased from 9.92 to 30 for stronger base centering (drift prevention)
float K_THETA     = 31.5f;     // 30.0f steps/s² per deg (base position gain)
float K_ALPHA     = 725.6f;    // 858.6f steps/s² per deg (pendulum angle gain)
float K_THETADOT  = 28.3f;     // 50.0f steps/s² per deg/s (base velocity gain)
float K_ALPHADOT  = 73.5f;     // 69.9f steps/s² per deg/s (pendulum velocity gain)

// Legacy gains (kept for backward compatibility with serial commands)
float ACC_KP = 742.0f;   // Equivalent to inner loop if needed
float ACC_KD = 54.6f;
float ACC_KI = 0.0f;
float KTHETA = 5.0f;     // Equivalent to outer loop if needed
float KTHETADOT = 10.0f;
float KV_THETA = 0.0f;

// --- PROFESSOR'S DERIVATIVE FILTER METHOD ---
// Uses bilinear transform: H(s) = s/(1 + s/wc)
// Tuning: omega_c is cutoff frequency in rad/s (20 Hz ≈ 125 rad/s)
float omega_c = 180.0f;     // Cutoff frequency (rad/s)

// Discrete filter coefficients for professor's derivative filter (computed from omega_c and nominal dt).
float d_b0 = 0.0f;  // Numerator coefficient
float d_a1 = 0.0f;  // Denominator coefficient (feedback)

// History variables for derivative filter
float last_alpha_raw = 0.0f;
float last_alpha_dot = 0.0f;
float last_theta_raw = 0.0f;
float last_theta_dot = 0.0f;

// Legacy variables (kept for serial command compatibility)
float D_FILT_ALPHA = 0.35f;  // Not used with professor's method
float D_FILT_THETA = 0.20f;  // Not used with professor's method
bool  filterFirst  = true;   // Not used with professor's method
bool  derivInit    = false;

float ALPHA_GATE_DEG = 18.0f;
const float OUTER_MAX_ACC = 1000.0f;
bool outerLoopEnabled = true;  // Use full-state feedback when true

// Optional: velocity leak (bias killer)
float VEL_LEAK = 2.0f; // 1/s
const float VEL_LEAK_ALPHA_WINDOW_DEG = 5.0f; // only leak when |alpha| is small (near upright)
const float VEL_LEAK_THETA_WINDOW_DEG = 20.0f; // only leak when |theta| is small (near center)

float alphaInt = 0.0f;
const float ALPHA_INT_CLAMP = 3.0f;

// Signs
int motorSign  = +1;   // +steps = CW from above
int ALPHA_SIGN = -1;   // default (matches historical working builds); verify with 'S' and save with 'Y'
int THETA_SIGN = +1;
int CTRL_SIGN  = +1;

bool debugAlpha   = false;
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
float motorZeroDeg = 0.0f;

// Controller command state (now ONLY speed is used for actuation)
float thetaDotCmdMotor   = 0.0f;   // steps/s (MOTOR coordinates)

// (kept for compatibility/logging but NOT used to command motor anymore)
float thetaCmdStepsMotor = 0.0f;

// Derivative filter outputs
float alphaDotFilt = 0.0f;
float thetaDotFilt = 0.0f;

// Timing
uint32_t lastUs = 0;
uint32_t logDecim = 0;

// ---------------- VELOCITY MODE STATE ----------------
int runDir = 0;                 // -1 backward, +1 forward, 0 stopped
// Motor velocity dead-zone with hysteresis:
// - When STOPPED: start only if |vel_cmd| >= speedStopHz
// - When RUNNING: stop only if |vel_cmd| < (speedStopHz * SPEED_STOP_HYST_RATIO)
// This keeps the "noise gate" behavior of a higher threshold while reducing stop/start twitch.
// Tune with 'N' and save with 'Y'.
float speedStopHz = 50.0f;
const float SPEED_STOP_HYST_RATIO = 0.50f;

// -----------------------------------------------------

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
  Serial.print(" speedStopHz="); Serial.print(speedStopHz, 2);
  Serial.print(" speedStopHzStop="); Serial.println(speedStopHz * SPEED_STOP_HYST_RATIO, 2);
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
  omega_c = 180.0f;
  speedStopHz = 50.0f;
  updateDerivativeCoeffs();
  derivInit = false;

  Serial.println("Cleared EEPROM settings and applied defaults in RAM.");
  Serial.print("motorSign="); Serial.print(motorSign);
  Serial.print(" ALPHA_SIGN="); Serial.print(ALPHA_SIGN);
  Serial.print(" THETA_SIGN="); Serial.print(THETA_SIGN);
  Serial.print(" CTRL_SIGN="); Serial.print(CTRL_SIGN);
  Serial.print(" omega_c="); Serial.print(omega_c, 1);
  Serial.print(" speedStopHz="); Serial.print(speedStopHz, 2);
  Serial.print(" speedStopHzStop="); Serial.println(speedStopHz * SPEED_STOP_HYST_RATIO, 2);
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

// Command motor in continuous velocity mode
void commandMotorSpeed(float v_steps_per_s) {
  float vabs = fabs(v_steps_per_s);

  // Stop: DO NOT call setSpeedInHz(0). Use stopMove() for decel stop.
  // Always stop for truly tiny commands (avoids unintended creep if speedStopHz is set to 0).
  if (vabs < 0.5f) {
    if (runDir != 0) {
      stepper->stopMove();
      runDir = 0;
    }
    return;
  }

  const float stopHz = speedStopHz * SPEED_STOP_HYST_RATIO;
  const int desiredDir = (v_steps_per_s > 0.0f) ? +1 : -1;

  // Dead-zone hysteresis:
  // - From STOPPED: require >= speedStopHz to start
  // - While RUNNING in same direction: allow down to stopHz
  // - If command asks to REVERSE: require >= speedStopHz, otherwise stop first
  if (runDir == 0) {
    if (vabs < speedStopHz) return;
  } else if (desiredDir != runDir) {
    if (vabs < speedStopHz) {
      stepper->stopMove();
      runDir = 0;
      return;
    }
  } else {
    if (vabs < stopHz) {
      stepper->stopMove();
      runDir = 0;
      return;
    }
  }

  uint32_t spd = (uint32_t)constrain(vabs, 1.0f, MAX_SPEED_HZ);
  stepper->setSpeedInHz(spd);

  if (desiredDir > 0) {
    if (runDir != +1) {
      stepper->runForward();
      runDir = +1;
    } else {
      stepper->applySpeedAcceleration();
    }
  } else {
    if (runDir != -1) {
      stepper->runBackward();
      runDir = -1;
    } else {
      stepper->applySpeedAcceleration();
    }
  }
}

void resetController() {
  thetaCmdStepsMotor   = 0.0f;
  thetaDotCmdMotor     = 0.0f;

  alphaDotFilt         = 0.0f;
  alphaInt             = 0.0f;
  thetaDotFilt         = 0.0f;

  // Reset history for professor's derivative filter
  last_alpha_raw       = 0.0f;
  last_alpha_dot       = 0.0f;
  last_theta_raw       = 0.0f;
  last_theta_dot       = 0.0f;

  derivInit            = false;
  logDecim             = 0;
  runDir               = 0;
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
  while (Serial.available()) {
    char c = Serial.read();

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

    if (c == '3') {
      Serial.println("\n--- TEST 3: LIVE UPRIGHT VERIFICATION ---");
      Serial.println("Hold pendulum UPRIGHT. Tilt it and watch if arm follows.");
      Serial.println("Press any key to start (press again to stop)...");
      while (!Serial.available()) { delay(10); }
      while (Serial.available()) Serial.read();

      selectMux(0);
      float upRef = readAS5600Deg();

      stepper->enableOutputs();
      stepper->setCurrentPosition(0);
      stepper->setAcceleration(10000);
      stepper->setSpeedInHz(10000);

      Serial.println("\n  Tilt pendulum -> arm should follow SAME direction!");
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
      Serial.println("  If NO: flip ALPHA_SIGN (A 1 or A -1) and try again.");
      Serial.println("  If YES: send Y to save, then Z (calibrate), then E (engage).");
      return;
    }

    if (c == 'Z' || c == 'z') {
      selectMux(0);
      float pendUp = wrap360(readAS5600Deg());
      targetRawDeg = pendUp;

      Serial.print("pendUp="); Serial.print(pendUp,2);
      Serial.print(" targetRawDeg="); Serial.println(targetRawDeg,2);

      selectMux(1);
      motorZeroDeg = readAS5600Deg();

      stepper->setCurrentPosition(0);
      resetController();

      isCalibrated = true;

      Serial.print("motorSign fixed = ");
      Serial.println(motorSign);

      Serial.print("motorSign="); Serial.print(motorSign);
      Serial.print(" ALPHA_SIGN="); Serial.print(ALPHA_SIGN);
      Serial.print(" THETA_SIGN="); Serial.print(THETA_SIGN);
      Serial.print(" CTRL_SIGN="); Serial.println(CTRL_SIGN);
      Serial.print("omega_c="); Serial.print(omega_c, 1);
      Serial.print(" (b0="); Serial.print(d_b0, 4);
      Serial.print(", a1="); Serial.print(d_a1, 4);
      Serial.print(") speedStopHz="); Serial.print(speedStopHz, 2);
      Serial.print(" speedStopHzStop="); Serial.println(speedStopHz * SPEED_STOP_HYST_RATIO, 2);

      currentState = STATE_IDLE;
      firstIdle = true;
      stepper->disableOutputs();
      Serial.println("CALIBRATED at upright! Press E to arm, then release.");
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

    if (c == 'T' || c == 't') {
      debugAlpha = !debugAlpha;
      Serial.print("debugAlpha="); Serial.println(debugAlpha ? "ON" : "OFF");
      return;
    }

    if (c == 'J' || c == 'j') {
      float deg = Serial.parseFloat();
      deg = constrain(deg, -20.0f, 20.0f);

      int32_t steps = (int32_t)(deg * STEPS_PER_DEG);
      stepper->enableOutputs();
      stepper->moveTo(steps);
      delay(300);
      stepper->moveTo(0);
      delay(300);
      stepper->disableOutputs();

      selectMux(1);
      float baseDeg = getAngleDiffDeg(readAS5600Deg(), motorZeroDeg);
      Serial.print("JOG done. baseDeg="); Serial.println(baseDeg, 2);
      return;
    }

    // Full-state feedback gain tuning
    if (c == '1') { float v = Serial.parseFloat(); K_THETA=v; Serial.print("K_THETA="); Serial.println(K_THETA,2); return; }
    if (c == '2') { float v = Serial.parseFloat(); K_ALPHA=v; Serial.print("K_ALPHA="); Serial.println(K_ALPHA,2); return; }
    if (c == '4') { float v = Serial.parseFloat(); K_THETADOT=v; Serial.print("K_THETADOT="); Serial.println(K_THETADOT,2); return; }
    if (c == '5') { float v = Serial.parseFloat(); K_ALPHADOT=v; Serial.print("K_ALPHADOT="); Serial.println(K_ALPHADOT,2); return; }

    // Legacy gain commands (for reference)
    if (c == 'P' || c == 'p') { float v = Serial.parseFloat(); if (v>0) ACC_KP=v; Serial.print("ACC_KP="); Serial.println(ACC_KP,1); return; }
    if (c == 'D' || c == 'd') { float v = Serial.parseFloat(); if (v>0) ACC_KD=v; Serial.print("ACC_KD="); Serial.println(ACC_KD,1); return; }
    if (c == 'I' || c == 'i') { float v = Serial.parseFloat(); if (v>=0) ACC_KI=v; Serial.print("ACC_KI="); Serial.println(ACC_KI,1); return; }
    if (c == 'V' || c == 'v') { float v = Serial.parseFloat(); if (v>=0) KV_THETA=v; Serial.print("KV_THETA="); Serial.println(KV_THETA,2); return; }
    if (c == 'F' || c == 'f') { float v = Serial.parseFloat(); if (v>=0.01f && v<=0.9f) D_FILT_ALPHA=v; Serial.print("D_FILT_ALPHA="); Serial.println(D_FILT_ALPHA,2); return; }

    if (c == 'O' || c == 'o') { outerLoopEnabled = !outerLoopEnabled; Serial.print("stateFeedback="); Serial.println(outerLoopEnabled ? "ON" : "OFF"); return; }
    if (c == 'K' || c == 'k') { float v = Serial.parseFloat(); if (v>=0) KTHETA=v; Serial.print("KTHETA="); Serial.println(KTHETA,1); return; }
    if (c == 'L' || c == 'l') { float v = Serial.parseFloat(); if (v>=0) KTHETADOT=v; Serial.print("KTHETADOT="); Serial.println(KTHETADOT,1); return; }

    if (c == 'Y' || c == 'y') { savePersistedConfig(); return; }
    if (c == 'R' || c == 'r') { clearPersistedConfig(); return; }

    if (c == 'G' || c == 'g') {
      Serial.print("motorSign="); Serial.print(motorSign);
      Serial.print(" ALPHA_SIGN="); Serial.print(ALPHA_SIGN);
      Serial.print(" THETA_SIGN="); Serial.print(THETA_SIGN);
      Serial.print(" CTRL_SIGN="); Serial.println(CTRL_SIGN);
      Serial.print("omega_c="); Serial.print(omega_c, 1);
      Serial.print(" (b0="); Serial.print(d_b0, 4);
      Serial.print(", a1="); Serial.print(d_a1, 4);
      Serial.print(") speedStopHz="); Serial.print(speedStopHz, 2);
      Serial.print(" speedStopHzStop="); Serial.println(speedStopHz * SPEED_STOP_HYST_RATIO, 2);
      Serial.print("VEL_LEAK="); Serial.print(VEL_LEAK, 3);
      Serial.print(" leakAlphaWinDeg="); Serial.print(VEL_LEAK_ALPHA_WINDOW_DEG, 2);
      Serial.print(" leakThetaWinDeg="); Serial.println(VEL_LEAK_THETA_WINDOW_DEG, 2);
      Serial.print("StateFeedback="); Serial.print(outerLoopEnabled ? "ON" : "OFF");
      Serial.print(" K_THETA="); Serial.print(K_THETA, 2);
      Serial.print(" K_ALPHA="); Serial.print(K_ALPHA, 2);
      Serial.print(" K_THETADOT="); Serial.print(K_THETADOT, 2);
      Serial.print(" K_ALPHADOT="); Serial.println(K_ALPHADOT, 2);
      return;
    }

    if (c == 'U' || c == 'u') {
      float v = Serial.parseFloat();
      if (v >= 0.0f && v <= 20.0f) VEL_LEAK = v;
      Serial.print("VEL_LEAK="); Serial.println(VEL_LEAK, 3);
      return;
    }

    // Motor command dead-zone tuning (in steps/s).
    if (c == 'N' || c == 'n') {
      float v = Serial.parseFloat();
      if (v >= 0.0f && v <= 500.0f) speedStopHz = v;
      Serial.print("speedStopHz="); Serial.print(speedStopHz, 2);
      Serial.print(" speedStopHzStop="); Serial.println(speedStopHz * SPEED_STOP_HYST_RATIO, 2);
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

    if (c == 'X' || c == 'x') {
      filterFirst = !filterFirst;
      derivInit = false;
      Serial.print("Derivative mode: ");
      Serial.println(filterFirst ? "FILTER-FIRST (smoother)" : "DIFF-FIRST (faster)");
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
  Serial.println("Commands:");
  Serial.println("  S       -> SIGN DIAGNOSTIC");
  Serial.println("  Z       -> Calibrate (hold pendulum UPRIGHT)");
  Serial.println("  E       -> Toggle engage");
  Serial.println("  M 1/-1  -> Set motorSign");
  Serial.println("  A 1/-1  -> Set ALPHA_SIGN");
  Serial.println("  H       -> Toggle THETA_SIGN");
  Serial.println("  B       -> Toggle CTRL_SIGN");
  Serial.println("  T       -> Toggle alpha debug prints");
  Serial.println("  J 10    -> Jog test (+10deg)");
  Serial.println("  1#, 2#, 4#, 5# -> K_THETA, K_ALPHA, K_THETADOT, K_ALPHADOT");
  Serial.println("  P#, D#, I#, F# -> Legacy gains (reference)");
  Serial.println("  O       -> Toggle state feedback");
  Serial.println("  K#, L#  -> Legacy outer loop gains");
  Serial.println("  X       -> Toggle derivative mode");
  Serial.println("  W #     -> Set derivative cutoff omega_c (rad/s)");
  Serial.println("  N #     -> Set motor dead-zone start threshold (steps/s)");
  Serial.println("  U #     -> Set VEL_LEAK (1/s)");
  Serial.println("  Y       -> Save signs/tuning to EEPROM");
  Serial.println("  R       -> Clear EEPROM (defaults on reset)");
  Serial.println("  G       -> Print current config");
  Serial.println("Log: t_ms,alphaRaw100,alphaDot100,theta100,thetaDot100,accCmd,velCmd,posMeasSteps,clamped");

  bool loaded = loadPersistedConfig();
  Serial.print("[EEPROM] ");
  Serial.println(loaded ? "Loaded saved settings" : "No saved settings; using defaults");

  updateDerivativeCoeffs();

  Serial.print("motorSign="); Serial.print(motorSign);
  Serial.print(" ALPHA_SIGN="); Serial.print(ALPHA_SIGN);
  Serial.print(" THETA_SIGN="); Serial.print(THETA_SIGN);
  Serial.print(" CTRL_SIGN="); Serial.println(CTRL_SIGN);
  Serial.print("speedStopHz="); Serial.print(speedStopHz, 2);
  Serial.print(" speedStopHzStop="); Serial.println(speedStopHz * SPEED_STOP_HYST_RATIO, 2);
  Serial.print("StateFeedback="); Serial.print(outerLoopEnabled ? "ON" : "OFF");
  Serial.print(" K_THETA="); Serial.print(K_THETA,2);
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
  float alphaDegRaw = getAngleDiffDeg(pendDeg, targetRawDeg);
  float alphaRawSigned = (float)ALPHA_SIGN * alphaDegRaw;

  // Base
  selectMux(1);
  float baseDegRaw = getAngleDiffDeg(readAS5600Deg(), motorZeroDeg);
  float baseDeg = baseDegRaw; // for safety check
  float thetaDeg = THETA_SIGN * baseDegRaw;

  // Glitch rejection on raw angles (prevents random I2C/mux jumps from nuking control).
  if (derivInit) {
    float dAlphaPre = alphaRawSigned - last_alpha_raw;
    while (dAlphaPre > 180.0f) dAlphaPre -= 360.0f;
    while (dAlphaPre < -180.0f) dAlphaPre += 360.0f;
    if (fabs(dAlphaPre) > ALPHA_JUMP_REJECT_DEG) {
      alphaRawSigned = last_alpha_raw;
      alphaDegRaw = alphaRawSigned * (float)ALPHA_SIGN;
    }

    float dThetaPre = thetaDeg - last_theta_raw;
    if (fabs(dThetaPre) > THETA_JUMP_REJECT_DEG) {
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

      if (debugAlpha) {
        static uint32_t dec = 0;
        if (++dec >= 10) {
          dec = 0;
          int32_t a100raw = (int32_t)(alphaDegRaw * 100.0f);
          Serial.print("ALPHA_RAW_x100="); Serial.println(a100raw);
        }
      }

      bool ok = (fabs(alphaDegRaw) < ENGAGE_PEND_DEG) && (fabs(alphaDotFilt) < ENGAGE_DOT_DEG);

      static uint32_t dec2 = 0;
      if (++dec2 >= 20) {
        dec2 = 0;
        Serial.print("alpha="); Serial.print(alphaDegRaw,2);
        Serial.print(" alphaDot="); Serial.print(alphaDotFilt,1);
        Serial.print(" | theta="); Serial.print(thetaDeg,2);
        Serial.print(" thetaDot="); Serial.print(thetaDotFilt,1);
        Serial.print(" | enable="); Serial.println(ok ? 1 : 0);
      }

      if (ok) engageCount++;
      else engageCount = 0;

      if (isCalibrated && armEnabled && engageCount >= ENGAGE_COUNT_REQ) {
        // Keep warm derivatives; reset only controller integrators/commands
        alphaInt = 0.0f;
        thetaDotCmdMotor = 0.0f;
        thetaCmdStepsMotor = 0.0f;
        runDir = 0;

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

      if (fabs(baseDeg) > LIM_MOTOR_DEG || fabs(alphaDegRaw) > LIM_PEND_DEG) {
        enterIdle(true);  // Keep armEnabled=true for auto-rearm when pendulum returns to window
        Serial.println("FALLEN (will auto-rearm when upright)");
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

      // ============================================================
      // FULL-STATE FEEDBACK CONTROLLER (Section 11.7 modelling_complete.md)
      // Control law: accCmdPhysical = K_THETA*theta + K_ALPHA*alpha + K_THETADOT*thetaDot + K_ALPHADOT*alphaDot
      // Gains are negative (from pole placement), providing restoring force
      // ============================================================
      
      float accCmdPhysical = 0.0f;
      
      if (outerLoopEnabled) {
        // Full-state feedback: direct application of state feedback law
        accCmdPhysical = K_THETA * thetaDeg + K_ALPHA * alphaRawSigned + 
                         K_THETADOT * thetaDotFilt + K_ALPHADOT * alphaDotFilt;
        
        // Apply control sign convention
        accCmdPhysical *= CTRL_SIGN;
      } else {
        // Fallback: legacy two-loop controller for comparison
        float alphaCtrl = alphaRawSigned;
        if (fabs(alphaCtrl) < ALPHA_DEADBAND_DEG) alphaCtrl = 0.0f;
        
        alphaInt += alphaCtrl * dt;
        alphaInt = constrain(alphaInt, -ALPHA_INT_CLAMP, ALPHA_INT_CLAMP);
        
        float inner = CTRL_SIGN * (ACC_KP * alphaCtrl + ACC_KD * alphaDotFilt + ACC_KI * alphaInt);
        
        if (fabs(alphaCtrl) < ALPHA_GATE_DEG) {
          float outer = -(KTHETA * thetaDeg + KTHETADOT * thetaDotFilt);
          outer = constrain(outer, -OUTER_MAX_ACC, OUTER_MAX_ACC);
          inner += CTRL_SIGN * outer;
        }
        
        accCmdPhysical = inner;
      }

      // ramp on engage
      float ramp = (millis() - engageStartMs) * (1.0f / 100.0f);
      if (ramp > 1.0f) ramp = 1.0f;
      accCmdPhysical *= ramp;

      if (KV_THETA > 0.0f) {
        accCmdPhysical += -(KV_THETA * thetaDotFilt);
      }

      int sat = 0;
      if (accCmdPhysical >  MAX_ACC_STEPS) { accCmdPhysical =  MAX_ACC_STEPS; sat = 1; }
      if (accCmdPhysical < -MAX_ACC_STEPS) { accCmdPhysical = -MAX_ACC_STEPS; sat = 1; }

      // Anti-windup for legacy integral term (only used if outerLoopEnabled=false)
      if (sat && !outerLoopEnabled) {
        float alphaCtrl = alphaRawSigned;
        if (fabs(alphaCtrl) < ALPHA_DEADBAND_DEG) alphaCtrl = 0.0f;
        alphaInt -= alphaCtrl * dt;
        alphaInt = constrain(alphaInt, -ALPHA_INT_CLAMP, ALPHA_INT_CLAMP);
      }

      // Apply motor sign
      float accCmdMotor = motorSign * accCmdPhysical;

      // Integrate accel -> desired speed (leaky integrator near upright to prevent slow drift).
      float leak = 0.0f;
      if (VEL_LEAK > 0.0f &&
          fabs(alphaRawSigned) < VEL_LEAK_ALPHA_WINDOW_DEG &&
          fabs(thetaDeg) < VEL_LEAK_THETA_WINDOW_DEG) {
        leak = VEL_LEAK;
      }
      thetaDotCmdMotor += (accCmdMotor - leak * thetaDotCmdMotor) * dt;
      thetaDotCmdMotor = constrain(thetaDotCmdMotor, -MAX_SPEED_HZ, MAX_SPEED_HZ);

      // soft limit using MEASURED theta
      const float LIM_MARGIN_DEG = 2.0f;
      if ((thetaDeg > (LIM_MOTOR_DEG - LIM_MARGIN_DEG) && thetaDotCmdMotor > 0) ||
          (thetaDeg < (-LIM_MOTOR_DEG + LIM_MARGIN_DEG) && thetaDotCmdMotor < 0)) {
        thetaDotCmdMotor = 0.0f;
      }

      // Command continuous velocity
      commandMotorSpeed(thetaDotCmdMotor);

      // Human-readable status (1 Hz for monitoring)
      static uint32_t statusDecim = 0;
      if (++statusDecim >= 200) {  // Every 200 loops = 1 Hz at 200 Hz
        statusDecim = 0;
        Serial.print("[STATUS] alpha="); Serial.print(alphaRawSigned,2);
        Serial.print(" alphaDot="); Serial.print(alphaDotFilt,1);
        Serial.print(" | theta="); Serial.print(thetaDeg,2);
        Serial.print(" thetaDot="); Serial.print(thetaDotFilt,1);
        Serial.print(" | acc="); Serial.print((int)accCmdPhysical);
        Serial.print(" vel="); Serial.println((int)thetaDotCmdMotor);
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
