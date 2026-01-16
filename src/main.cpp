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

const float MAX_ACC_STEPS = 20000.0f;  // safe accel
const float MAX_SPEED_HZ  = 25000.0f;  // speed limit (steps/s)

const uint32_t LOOP_US = 5000;         // 200 Hz
const float ALPHADOT_CLAMP = 300.0f;   // deg/s
const float THETADOT_CLAMP = 300.0f;   // deg/s safety clamp for base rate
const float ALPHA_DEADBAND_DEG = 0.3f; // try 0.2..0.6

// Gains
float ACC_KP = 742.0f;    // backed off for stability proof
float ACC_KD = 54.6f;
float ACC_KI = 0.0f;      // OFF until stable
float KV_THETA = 0.0f;    // keep OFF for now
float D_FILT_ALPHA = 0.20f;

// Theta (base) outer-loop gains (start small)
float KTHETA     = 15.0f;  // steps/s^2 per deg (start 10..25)
float KTHETADOT  = 60.0f;  // steps/s^2 per (deg/s) (30..120)
float D_FILT_THETA = 0.20f;

float ALPHA_GATE_DEG = 3.0f;   // tighter gate
const float OUTER_MAX_ACC = 1500.0f;  // steps/s^2 cap for outer loop

// Optional: velocity leak (prevents slow walk)
float VEL_LEAK = 1.0f; // 1/s  (start here: 0.5..2.0)

float alphaInt = 0.0f;
const float ALPHA_INT_CLAMP = 3.0f;

// Signs
int motorSign  = -1;      // FIXED by your jog test earlier
int ALPHA_SIGN = +1;

bool debugAlpha   = false;
bool armEnabled   = false;
bool firstIdle    = true;
bool isCalibrated = false;

int engageCount = 0;
const int ENGAGE_COUNT_REQ = 10;  // 10 cycles at 200 Hz = 50 ms stable

FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;
AS5600 sensor;

enum State { STATE_CALIBRATE, STATE_IDLE, STATE_ACTIVE };
State currentState = STATE_CALIBRATE;

float targetRawDeg = 0.0f;
float motorZeroDeg = 0.0f;

float thetaCmdStepsMotor = 0.0f;   // command in MOTOR step coordinates
float thetaDotCmdMotor   = 0.0f;   // steps/s in MOTOR step coordinates

float alphaDotFilt = 0.0f;
float lastAlphaDeg = 0.0f;
float lastAlphaRawSigned = 0.0f;  // for derivative (signed, no deadband)

// Theta rate filter state
float thetaDotFilt = 0.0f;
float lastThetaDeg = 0.0f;

uint32_t lastUs = 0;
uint32_t logDecim = 0;

void selectMux(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
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

void resetController() {
  thetaCmdStepsMotor = 0.0f;
  thetaDotCmdMotor   = 0.0f;
  alphaDotFilt          = 0.0f;
  lastAlphaDeg          = 0.0f;
  lastAlphaRawSigned    = 0.0f;
  alphaInt              = 0.0f;
  thetaDotFilt          = 0.0f;
  lastThetaDeg          = 0.0f;
  logDecim              = 0;
}

void enterIdle() {
  stepper->stopMove();
  stepper->disableOutputs();
  currentState = STATE_IDLE;
  armEnabled = false;    // require explicit E each time
  firstIdle = true;
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == 'Z' || c == 'z') {
      // Calibrate: pendulum DOWN so UP is 180 deg away
      selectMux(0);
      float pendDown = wrap360(readAS5600Deg());
      targetRawDeg = wrap360(pendDown - 180.0f);

      Serial.print("pendDown="); Serial.print(pendDown,2);
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
      Serial.println("CALIBRATED! Lift near upright to engage.");
      return;
    }

    if (c == 'E' || c == 'e') {
      if (!isCalibrated) {
        Serial.println("ERR: Press Z to calibrate first.");
        return;
      }
      armEnabled = !armEnabled;
      Serial.print("armEnabled="); Serial.println(armEnabled ? "ON" : "OFF");
      if (!armEnabled) {
        enterIdle();
      }
      return;
    }

    if (c == 'A' || c == 'a') {
      int v = (int)Serial.parseInt();
      if (v == 1 || v == -1) ALPHA_SIGN = v;
      Serial.print("ALPHA_SIGN="); Serial.println(ALPHA_SIGN);
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

  Serial.println("--- RIP ACCEL CONTROL (SIGN-PROOFED) ---");
  Serial.println("Commands:");
  Serial.println("  Z       -> Calibrate (hold pendulum DOWN)");
  Serial.println("  E       -> Toggle manual engage");
  Serial.println("  A 1/-1  -> Set pendulum sign");
  Serial.println("  T       -> Toggle alpha debug prints");
  Serial.println("  J 10    -> Jog test (+10deg)");
  Serial.println("  P#, D#, I#, V#, F# -> gains/filter");
  Serial.println("Log: t_ms,alphaDeg,alphaDotFilt,accCmd,thetaDotCmd,sat");
  Serial.print("motorSign fixed = "); Serial.println(motorSign);

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
  float alphaDeg = (float)ALPHA_SIGN * alphaDegRaw;
  float alphaRawSigned = (float)ALPHA_SIGN * alphaDegRaw;  // signed raw angle

  // Deadband: ignore tiny alpha near upright to avoid chasing quantization/noise
  if (fabs(alphaDeg) < ALPHA_DEADBAND_DEG) alphaDeg = 0.0f;

  // Base for safety only
  selectMux(1);
  float baseDeg = getAngleDiffDeg(readAS5600Deg(), motorZeroDeg);

  switch (currentState) {

    case STATE_CALIBRATE: {
      // do nothing until Z
    } break;

    case STATE_IDLE: {
      if (firstIdle) {
        resetController();
        firstIdle = false;
        stepper->disableOutputs();
      }

      // Derivative from RAW signed angle (NO deadband!)
      float dAlpha = alphaRawSigned - lastAlphaRawSigned;
      while (dAlpha > 180.0f) dAlpha -= 360.0f;
      while (dAlpha < -180.0f) dAlpha += 360.0f;

      float alphaDot = dAlpha / dt;
      lastAlphaRawSigned = alphaRawSigned;

      alphaDotFilt += D_FILT_ALPHA * (alphaDot - alphaDotFilt);
      alphaDotFilt = constrain(alphaDotFilt, -ALPHADOT_CLAMP, ALPHADOT_CLAMP);

      // Now create alpha for control with deadband (only affects P and I)
      float alphaCtrl = alphaRawSigned;
      if (fabs(alphaCtrl) < ALPHA_DEADBAND_DEG) alphaCtrl = 0.0f;

      // keep theta rate filter alive
      float thetaDot = (baseDeg - lastThetaDeg) / dt;
      lastThetaDeg = baseDeg;
      thetaDotFilt += D_FILT_THETA * (thetaDot - thetaDotFilt);
      thetaDotFilt = constrain(thetaDotFilt, -THETADOT_CLAMP, THETADOT_CLAMP);

      if (debugAlpha) {
        static uint32_t dec = 0;
        if (++dec >= 10) {
          dec = 0;
          int32_t a100raw = (int32_t)(alphaDegRaw * 100.0f);
          Serial.print("ALPHA_RAW_x100="); Serial.println(a100raw);
        }
      }

      // Print engage gate signals (Test 1)
      static uint32_t dec2=0;
      if (++dec2 >= 20) { // 10 Hz
        dec2 = 0;
        Serial.print("raw="); Serial.print(alphaDegRaw,2);
        Serial.print(" dot="); Serial.print(alphaDotFilt,1);
        Serial.print(" enable="); Serial.println((fabs(alphaDegRaw) < 2.0f && fabs(alphaDotFilt) < 20.0f) ? 1 : 0);
      }

      // Engage only if calibrated + enabled + near upright + calm
      bool ok = (fabs(alphaDegRaw) < 3.0f) && (fabs(alphaDotFilt) < 30.0f);

      if (ok) engageCount++;
      else engageCount = 0;

      if (isCalibrated && armEnabled && engageCount >= ENGAGE_COUNT_REQ) {
        // init states
        lastAlphaRawSigned = alphaRawSigned;
        alphaDotFilt = 0.0f;
        alphaInt     = 0.0f;
        thetaDotCmdMotor = 0.0f;
        thetaCmdStepsMotor = 0.0f;
        lastThetaDeg = baseDeg;
        thetaDotFilt = 0.0f;
        logDecim = 0;

        currentState = STATE_ACTIVE;
        stepper->enableOutputs();
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

      float dAlpha = alphaDeg - lastAlphaDeg;
      while (dAlpha > 180.0f) dAlpha -= 360.0f;
      while (dAlpha < -180.0f) dAlpha += 360.0f;

      float alphaDot = dAlpha / dt;
      lastAlphaDeg = alphaDeg;
      alphaDotFilt += D_FILT_ALPHA * (alphaDot - alphaDotFilt);
      alphaDotFilt = constrain(alphaDotFilt, -ALPHADOT_CLAMP, ALPHADOT_CLAMP);

      // Compute measured theta rate
      float thetaDot = (baseDeg - lastThetaDeg) / dt;
      lastThetaDeg = baseDeg;
      thetaDotFilt += D_FILT_THETA * (thetaDot - thetaDotFilt);
      thetaDotFilt = constrain(thetaDotFilt, -THETADOT_CLAMP, THETADOT_CLAMP);

      alphaInt += alphaCtrl * dt;
      alphaInt = constrain(alphaInt, -ALPHA_INT_CLAMP, ALPHA_INT_CLAMP);

      // Compute acceleration in physical units
      float accCmdPhysical = -(ACC_KP * alphaCtrl + ACC_KD * alphaDotFilt + ACC_KI * alphaInt);

      // Outer loop for base (gated and capped)
      if (fabs(alphaDegRaw) < ALPHA_GATE_DEG && fabs(alphaDotFilt) < 40.0f) {
        float outer = -(KTHETA * baseDeg + KTHETADOT * thetaDotFilt);
        outer = constrain(outer, -OUTER_MAX_ACC, OUTER_MAX_ACC);
        accCmdPhysical += outer;
      }

      // Optional: use measured velocity damping instead of commanded
      if (KV_THETA > 0.0f) {
        accCmdPhysical += -(KV_THETA * thetaDotFilt);
      }

      int sat = 0;
      if (accCmdPhysical >  MAX_ACC_STEPS) { accCmdPhysical =  MAX_ACC_STEPS; sat = 1; }
      if (accCmdPhysical < -MAX_ACC_STEPS) { accCmdPhysical = -MAX_ACC_STEPS; sat = 1; }

      if (sat) {
        alphaInt -= alphaCtrl * dt; // anti-windup
        alphaInt = constrain(alphaInt, -ALPHA_INT_CLAMP, ALPHA_INT_CLAMP);
      }

      // Apply motorSign BEFORE integrating
      float accCmdMotor = motorSign * accCmdPhysical;

      // Optional velocity leak (prevents slow walk)
      if (VEL_LEAK > 0.0f) {
        thetaDotCmdMotor *= (1.0f - VEL_LEAK * dt);
      }

      if (!sat) {
        thetaDotCmdMotor += accCmdMotor * dt;
        thetaDotCmdMotor = constrain(thetaDotCmdMotor, -MAX_SPEED_HZ, MAX_SPEED_HZ);
        thetaCmdStepsMotor += thetaDotCmdMotor * dt;
      }

      float limitSteps = LIM_MOTOR_DEG * STEPS_PER_DEG;
      thetaCmdStepsMotor = constrain(thetaCmdStepsMotor, -limitSteps, limitSteps);

      stepper->moveTo((int32_t)thetaCmdStepsMotor);

      if (++logDecim >= 4) {
        logDecim = 0;
        uint32_t tMs = millis();
        int32_t a100  = (int32_t)(alphaDeg * 100.0f);
        int32_t ad100 = (int32_t)(alphaDotFilt * 100.0f);
        int32_t accI  = (int32_t)(accCmdPhysical);
        int32_t vI    = (int32_t)(thetaDotCmdMotor);
        int32_t base100 = (int32_t)(baseDeg * 100.0f);
        int32_t tdot100 = (int32_t)(thetaDotFilt * 100.0f);

        Serial.print(tMs); Serial.print(",");
        Serial.print(a100); Serial.print(",");
        Serial.print(ad100); Serial.print(",");
        Serial.print(accI); Serial.print(",");
        Serial.print(vI); Serial.print(",");
        Serial.print(sat); Serial.print(",");
        Serial.print(base100); Serial.print(",");
        Serial.println(tdot100);
      }
    } break;
  }
}
