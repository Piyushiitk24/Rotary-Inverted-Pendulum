#include <Arduino.h>
#include <Wire.h>
#include <FastAccelStepper.h>
#include <AS5600.h>

#define STEP_PIN 11
#define DIR_PIN  6
#define EN_PIN   7
#define MUX_ADDR 0x70

const float STEPS_PER_DEG = 4.444f;
const float LIM_MOTOR_DEG = 80.0f;
const float LIM_PEND_DEG  = 30.0f;

const float MAX_ACC_STEPS = 20000.0f;  // safe accel (steps/s^2)
const float MAX_SPEED_HZ  = 25000.0f;  // speed limit (steps/s)

const uint32_t LOOP_US = 5000;         // 200 Hz
const float ALPHADOT_CLAMP = 300.0f;   // deg/s
const float THETADOT_CLAMP = 300.0f;   // deg/s safety clamp for base rate
const float ALPHA_DEADBAND_DEG = 0.5f; // reduce sensor noise chasing

// Gains
float ACC_KP = 742.0f;
float ACC_KD = 54.6f;
float ACC_KI = 0.0f;
float KV_THETA = 0.0f;

float D_FILT_ALPHA = 0.35f;
bool  filterFirst  = true;
bool  derivInit    = false;

// Outer loop (base centering)
float KTHETA      = 5.0f;
float KTHETADOT   = 10.0f;
float D_FILT_THETA = 0.20f;

float ALPHA_GATE_DEG = 18.0f;
const float OUTER_MAX_ACC = 1000.0f;
bool outerLoopEnabled = true;

// Optional: velocity leak (bias killer)
float VEL_LEAK = 2.0f; // 1/s

float alphaInt = 0.0f;
const float ALPHA_INT_CLAMP = 3.0f;

// Signs
int motorSign  = +1;   // +steps = CW from above
int ALPHA_SIGN = -1;   // inverted for upright (as per your setup)
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

// Filters
float alphaDotFilt = 0.0f;
float lastAlphaRawSigned = 0.0f;
float alphaFilt = 0.0f;
float lastAlphaFilt = 0.0f;

float thetaDotFilt = 0.0f;
float lastThetaDeg = 0.0f;
float thetaFilt = 0.0f;
float lastThetaFilt = 0.0f;

// Timing
uint32_t lastUs = 0;
uint32_t logDecim = 0;

// ---------------- VELOCITY MODE STATE ----------------
int runDir = 0;                 // -1 backward, +1 forward, 0 stopped
const float SPEED_STOP_HZ = 50; // below this we stopMove() to avoid dithering

// -----------------------------------------------------

void selectMux(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delayMicroseconds(100);
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
  if (vabs < SPEED_STOP_HZ) {
    if (runDir != 0) {
      stepper->stopMove();
      runDir = 0;
    }
    return;
  }

  uint32_t spd = (uint32_t)constrain(vabs, 1.0f, MAX_SPEED_HZ);
  stepper->setSpeedInHz(spd);

  if (v_steps_per_s > 0) {
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
  lastAlphaRawSigned   = 0.0f;
  alphaInt             = 0.0f;

  thetaDotFilt         = 0.0f;
  lastThetaDeg         = 0.0f;

  alphaFilt            = 0.0f;
  lastAlphaFilt        = 0.0f;
  thetaFilt            = 0.0f;
  lastThetaFilt        = 0.0f;

  derivInit            = false;
  logDecim             = 0;

  runDir               = 0;
}

void enterIdle() {
  if (stepper) {
    stepper->stopMove();
    stepper->disableOutputs();
  }
  runDir = 0;

  currentState = STATE_IDLE;
  armEnabled = false;
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

      Serial.println("--- TEST 2: PENDULUM SENSOR ---");
      Serial.println("Let pendulum hang DOWN. Press any key...");
      while (!Serial.available()) { delay(10); }
      while (Serial.available()) Serial.read();

      selectMux(0);
      float pendDown = readAS5600Deg();
      Serial.print("  Pendulum DOWN reading: "); Serial.println(pendDown, 2);

      Serial.println("\n  Now tilt pendulum in the SAME rotational direction");
      Serial.print("  as the arm just moved (");
      Serial.print(posStepsCW ? "CLOCKWISE" : "COUNTER-CLOCKWISE");
      Serial.println(" when viewed from above).");
      Serial.println("  Hold it tilted ~30-45 degrees. Press any key...");
      while (!Serial.available()) { delay(10); }
      while (Serial.available()) Serial.read();

      selectMux(0);
      float pendTilted = readAS5600Deg();
      float pendDiff = getAngleDiffDeg(pendTilted, pendDown);
      Serial.print("  Pendulum TILTED reading: "); Serial.println(pendTilted, 2);
      Serial.print("  Difference: "); Serial.print(pendDiff, 2); Serial.println(" deg");

      int suggestedAlphaSign = (pendDiff > 0) ? +1 : -1;
      int suggestedThetaSign = (baseDiffPos > 0) ? +1 : -1;
      int suggestedMotorSign = +1;

      Serial.println();
      Serial.println("========================================");
      Serial.println("            RECOMMENDED SIGNS");
      Serial.println("========================================");
      Serial.print("  motorSign  = "); Serial.println(suggestedMotorSign);
      Serial.print("  ALPHA_SIGN = "); Serial.println(suggestedAlphaSign);
      Serial.print("  THETA_SIGN = "); Serial.println(suggestedThetaSign);
      Serial.println("  CTRL_SIGN  = +1 (start here)");
      Serial.println("========================================\n");

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

        Serial.print("  alpha="); Serial.print(alpha, 1);
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

      Serial.println("\n  Arm followed tilt? YES=signs ok, NO=flip CTRL_SIGN (press B)");
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

        Serial.print("  alpha="); Serial.print(alpha, 1);
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

      Serial.println("\n  Arm followed tilt? YES=signs correct, NO=press B to flip CTRL_SIGN");
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

    if (c == 'P' || c == 'p') { float v = Serial.parseFloat(); if (v>0) ACC_KP=v; Serial.print("ACC_KP="); Serial.println(ACC_KP,1); return; }
    if (c == 'D' || c == 'd') { float v = Serial.parseFloat(); if (v>0) ACC_KD=v; Serial.print("ACC_KD="); Serial.println(ACC_KD,1); return; }
    if (c == 'I' || c == 'i') { float v = Serial.parseFloat(); if (v>=0) ACC_KI=v; Serial.print("ACC_KI="); Serial.println(ACC_KI,1); return; }
    if (c == 'V' || c == 'v') { float v = Serial.parseFloat(); if (v>=0) KV_THETA=v; Serial.print("KV_THETA="); Serial.println(KV_THETA,2); return; }
    if (c == 'F' || c == 'f') { float v = Serial.parseFloat(); if (v>=0.01f && v<=0.9f) D_FILT_ALPHA=v; Serial.print("D_FILT_ALPHA="); Serial.println(D_FILT_ALPHA,2); return; }

    if (c == 'O' || c == 'o') { outerLoopEnabled = !outerLoopEnabled; Serial.print("outerLoop="); Serial.println(outerLoopEnabled ? "ON" : "OFF"); return; }
    if (c == 'K' || c == 'k') { float v = Serial.parseFloat(); if (v>=0) KTHETA=v; Serial.print("KTHETA="); Serial.println(KTHETA,1); return; }
    if (c == 'L' || c == 'l') { float v = Serial.parseFloat(); if (v>=0) KTHETADOT=v; Serial.print("KTHETADOT="); Serial.println(KTHETADOT,1); return; }

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

  Serial.println("--- RIP ACCEL CONTROL (VELOCITY MODE) ---");
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
  Serial.println("  P#, D#, I#, F# -> inner loop gains/filter");
  Serial.println("  O       -> Toggle outer loop");
  Serial.println("  K#, L#  -> KTHETA, KTHETADOT");
  Serial.println("  X       -> Toggle derivative mode");
  Serial.println("Log: t_ms,alphaRaw100,alphaDot100,theta100,thetaDot100,accCmd,velCmd,posMeasSteps,clamped");

  Serial.print("motorSign="); Serial.print(motorSign);
  Serial.print(" ALPHA_SIGN="); Serial.print(ALPHA_SIGN);
  Serial.print(" THETA_SIGN="); Serial.print(THETA_SIGN);
  Serial.print(" CTRL_SIGN="); Serial.println(CTRL_SIGN);
  Serial.print("OuterLoop="); Serial.print(outerLoopEnabled ? "ON" : "OFF");
  Serial.print(" KTHETA="); Serial.print(KTHETA);
  Serial.print(" KTHETADOT="); Serial.println(KTHETADOT);

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

      // One-shot derivative init
      if (!derivInit) {
        alphaFilt = alphaRawSigned;
        lastAlphaFilt = alphaRawSigned;
        thetaFilt = thetaDeg;
        lastThetaFilt = thetaDeg;
        lastAlphaRawSigned = alphaRawSigned;
        lastThetaDeg = thetaDeg;
        alphaDotFilt = 0.0f;
        thetaDotFilt = 0.0f;
        derivInit = true;
      }

      // alpha derivative
      if (filterFirst) {
        alphaFilt += D_FILT_ALPHA * (alphaRawSigned - alphaFilt);
        float dAlphaFilt = alphaFilt - lastAlphaFilt;
        while (dAlphaFilt > 180.0f) dAlphaFilt -= 360.0f;
        while (dAlphaFilt < -180.0f) dAlphaFilt += 360.0f;
        alphaDotFilt = dAlphaFilt / dt;
        lastAlphaFilt = alphaFilt;
      } else {
        float dAlpha = alphaRawSigned - lastAlphaRawSigned;
        while (dAlpha > 180.0f) dAlpha -= 360.0f;
        while (dAlpha < -180.0f) dAlpha += 360.0f;
        float alphaDot = dAlpha / dt;
        alphaDotFilt += D_FILT_ALPHA * (alphaDot - alphaDotFilt);
      }
      lastAlphaRawSigned = alphaRawSigned;
      alphaDotFilt = constrain(alphaDotFilt, -ALPHADOT_CLAMP, ALPHADOT_CLAMP);

      // theta derivative
      if (filterFirst) {
        thetaFilt += D_FILT_THETA * (thetaDeg - thetaFilt);
        thetaDotFilt = (thetaFilt - lastThetaFilt) / dt;
        lastThetaFilt = thetaFilt;
      } else {
        float thetaDot = (thetaDeg - lastThetaDeg) / dt;
        thetaDotFilt += D_FILT_THETA * (thetaDot - thetaDotFilt);
      }
      lastThetaDeg = thetaDeg;
      thetaDotFilt = constrain(thetaDotFilt, -THETADOT_CLAMP, THETADOT_CLAMP);

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
        Serial.print("raw="); Serial.print(alphaDegRaw,2);
        Serial.print(" dot="); Serial.print(alphaDotFilt,1);
        Serial.print(" enable="); Serial.println(ok ? 1 : 0);
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
      if (fabs(baseDeg) > LIM_MOTOR_DEG || fabs(alphaDegRaw) > LIM_PEND_DEG) {
        enterIdle();
        Serial.println("FALLEN (reset + outputs off)");
        break;
      }

      // alpha derivative
      if (filterFirst) {
        alphaFilt += D_FILT_ALPHA * (alphaRawSigned - alphaFilt);
        float dAlphaFilt = alphaFilt - lastAlphaFilt;
        while (dAlphaFilt > 180.0f) dAlphaFilt -= 360.0f;
        while (dAlphaFilt < -180.0f) dAlphaFilt += 360.0f;
        alphaDotFilt = dAlphaFilt / dt;
        lastAlphaFilt = alphaFilt;
      } else {
        float dAlpha = alphaRawSigned - lastAlphaRawSigned;
        while (dAlpha > 180.0f) dAlpha -= 360.0f;
        while (dAlpha < -180.0f) dAlpha += 360.0f;
        float alphaDot = dAlpha / dt;
        alphaDotFilt += D_FILT_ALPHA * (alphaDot - alphaDotFilt);
      }
      lastAlphaRawSigned = alphaRawSigned;
      alphaDotFilt = constrain(alphaDotFilt, -ALPHADOT_CLAMP, ALPHADOT_CLAMP);

      // theta derivative
      if (filterFirst) {
        thetaFilt += D_FILT_THETA * (thetaDeg - thetaFilt);
        thetaDotFilt = (thetaFilt - lastThetaFilt) / dt;
        lastThetaFilt = thetaFilt;
      } else {
        float thetaDot = (thetaDeg - lastThetaDeg) / dt;
        thetaDotFilt += D_FILT_THETA * (thetaDot - thetaDotFilt);
      }
      lastThetaDeg = thetaDeg;
      thetaDotFilt = constrain(thetaDotFilt, -THETADOT_CLAMP, THETADOT_CLAMP);

      // Deadband only for P/I
      float alphaCtrl = alphaRawSigned;
      if (fabs(alphaCtrl) < ALPHA_DEADBAND_DEG) alphaCtrl = 0.0f;

      alphaInt += alphaCtrl * dt;
      alphaInt = constrain(alphaInt, -ALPHA_INT_CLAMP, ALPHA_INT_CLAMP);

      // Acc command (steps/s^2)
      float accCmdPhysical = CTRL_SIGN * (ACC_KP * alphaCtrl + ACC_KD * alphaDotFilt + ACC_KI * alphaInt);

      if (outerLoopEnabled && fabs(alphaCtrl) < ALPHA_GATE_DEG) {
        // Restoring force: oppose position and velocity
        float outer = -(KTHETA * thetaDeg + KTHETADOT * thetaDotFilt);
        outer = constrain(outer, -OUTER_MAX_ACC, OUTER_MAX_ACC);
        // Apply CTRL_SIGN to outer loop as well
        accCmdPhysical += CTRL_SIGN * outer;
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

      if (sat) {
        alphaInt -= alphaCtrl * dt;
        alphaInt = constrain(alphaInt, -ALPHA_INT_CLAMP, ALPHA_INT_CLAMP);
      }

      // Apply motor sign
      float accCmdMotor = motorSign * accCmdPhysical;

      // Integrate accel -> desired speed (DO EVEN IF SATURATED, since acc already clamped)
      thetaDotCmdMotor += accCmdMotor * dt;
      thetaDotCmdMotor = constrain(thetaDotCmdMotor, -MAX_SPEED_HZ, MAX_SPEED_HZ);

      // leak
      if (VEL_LEAK > 0.0f) {
        thetaDotCmdMotor *= (1.0f - VEL_LEAK * dt);
      }

      // soft limit using MEASURED theta
      const float LIM_MARGIN_DEG = 2.0f;
      if ((thetaDeg > (LIM_MOTOR_DEG - LIM_MARGIN_DEG) && thetaDotCmdMotor > 0) ||
          (thetaDeg < (-LIM_MOTOR_DEG + LIM_MARGIN_DEG) && thetaDotCmdMotor < 0)) {
        thetaDotCmdMotor = 0.0f;
      }

      // Command continuous velocity
      commandMotorSpeed(thetaDotCmdMotor);

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
