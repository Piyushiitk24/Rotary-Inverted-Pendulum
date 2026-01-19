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

// Gains - increased for faster response
float ACC_KP = 742.0f;   // higher P for stiffer response
float ACC_KD = 54.6f;     // slightly higher D
float ACC_KI = 0.0f;      // OFF until stable
float KV_THETA = 0.0f;    // keep OFF for now
float D_FILT_ALPHA = 0.35f;  // faster filter (was 0.20, too slow)

// Theta (base) outer-loop gains (start small)
float KTHETA     = 15.0f;  // steps/s^2 per deg (start 10..25)
float KTHETADOT  = 60.0f;  // steps/s^2 per (deg/s) (30..120)
float D_FILT_THETA = 0.20f;

float ALPHA_GATE_DEG = 3.0f;   // tighter gate
const float OUTER_MAX_ACC = 1500.0f;  // steps/s^2 cap for outer loop
bool outerLoopEnabled = true;  // toggle with 'O' command

// Optional: velocity leak (prevents slow walk)
float VEL_LEAK = 2.0f; // 1/s  (increased from 1.0 for drift control)

float alphaInt = 0.0f;
const float ALPHA_INT_CLAMP = 3.0f;

// Signs
int motorSign  = +1;      // +steps = CW from above
int ALPHA_SIGN = -1;      // INVERTED for upright (180° from down test)

int THETA_SIGN = +1;     // sign for base encoder so +theta is consistent
int CTRL_SIGN  = +1;     // keep at +1

bool debugAlpha   = false;
bool armEnabled   = false;
bool firstIdle    = true;
bool isCalibrated = false;

const float ENGAGE_PEND_DEG = 5.0f;    // tighter: must be closer to upright
const float ENGAGE_DOT_DEG  = 30.0f;   // tighter: must be calmer before engage
uint32_t engageStartMs = 0;

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

    // NEW: Sign diagnostic mode - press 'S' to run
    if (c == 'S' || c == 's') {
      Serial.println("\n========================================");
      Serial.println("       SIGN DIAGNOSTIC MODE");
      Serial.println("========================================\n");
      
      // === TEST 1: Motor direction (establish reference) ===
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
      stepper->moveTo(100);  // +100 steps
      delay(500);
      
      selectMux(1);
      float baseAfterPos = readAS5600Deg();
      float baseDiffPos = getAngleDiffDeg(baseAfterPos, baseStart);
      
      Serial.println("\n  Did the arm move CLOCKWISE or COUNTER-CLOCKWISE?");
      Serial.println("  (View from ABOVE the arm)");
      Serial.println("  Press 'C' for Clockwise, 'A' for Anti-clockwise (CCW)...");
      
      char armDir = 0;
      while (armDir != 'C' && armDir != 'c' && armDir != 'A' && armDir != 'a') {
        if (Serial.available()) {
          armDir = Serial.read();
        }
        delay(10);
      }
      while (Serial.available()) Serial.read();
      
      bool posStepsCW = (armDir == 'C' || armDir == 'c');
      Serial.print("  ==> +steps moves arm "); Serial.println(posStepsCW ? "CLOCKWISE" : "COUNTER-CLOCKWISE");
      Serial.print("  ==> Encoder diff was: "); Serial.print(baseDiffPos, 2); Serial.println(" deg");
      
      // Return to start
      stepper->moveTo(0);
      delay(500);
      stepper->disableOutputs();
      stepper->setAcceleration((uint32_t)MAX_ACC_STEPS);
      stepper->setSpeedInHz((uint32_t)MAX_SPEED_HZ);
      Serial.println();
      
      // === TEST 2: Pendulum sensor direction ===
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
      
      // === CALCULATE SIGNS ===
      // We want: when pendulum tilts in +motor direction, alpha should be positive
      // So if pendulum sensor reads POSITIVE when tilted in motor's positive direction,
      // ALPHA_SIGN should be +1. Otherwise -1.
      int suggestedAlphaSign = (pendDiff > 0) ? +1 : -1;
      
      // For THETA_SIGN: we want +theta when arm moves in its +steps direction
      // If encoder reads positive when we sent +steps, THETA_SIGN = +1
      int suggestedThetaSign = (baseDiffPos > 0) ? +1 : -1;
      
      // motorSign: we want positive accCmd to produce positive arm movement
      // With this diagnostic, we're defining positive as the direction +steps goes
      // So motorSign = +1
      int suggestedMotorSign = +1;
      
      Serial.println();
      
      // === SUMMARY ===
      Serial.println("========================================");
      Serial.println("            RECOMMENDED SIGNS");
      Serial.println("========================================");
      Serial.print("  motorSign  = "); Serial.println(suggestedMotorSign);
      Serial.print("  ALPHA_SIGN = "); Serial.println(suggestedAlphaSign);
      Serial.print("  THETA_SIGN = "); Serial.println(suggestedThetaSign);
      Serial.println("  CTRL_SIGN  = +1 (start here)");
      Serial.println();
      Serial.println("PHYSICS CHECK:");
      Serial.println("  When pendulum falls in same direction as arm's +rotation:");
      Serial.println("    alpha > 0 --> accCmd > 0 --> arm accelerates in + direction");
      Serial.println("  This makes arm CHASE the falling pendulum (correct!)");
      Serial.println();
      Serial.println("  If arm goes OPPOSITE to fall, flip CTRL_SIGN (press B)");
      Serial.println();
      Serial.println("To apply these settings, type:");
      Serial.print("  M"); Serial.println(suggestedMotorSign);
      Serial.print("  A"); Serial.println(suggestedAlphaSign);
      if (suggestedThetaSign == -1) Serial.println("  H  (to flip THETA_SIGN to -1)");
      Serial.println("========================================\n");
      
      // === TEST 3: LIVE UPRIGHT VERIFICATION ===
      Serial.println("--- TEST 3: LIVE UPRIGHT VERIFICATION ---");
      Serial.println("This is the REAL test. Hold pendulum UPRIGHT.");
      Serial.println("I will show alpha in real-time. Tilt pendulum slightly");
      Serial.println("and watch if arm moves in SAME direction.");
      Serial.println("Press any key to start (press again to stop)...");
      while (!Serial.available()) { delay(10); }
      while (Serial.available()) Serial.read();
      
      // Capture current position as "upright"
      selectMux(0);
      float upRef = readAS5600Deg();
      selectMux(1);
      float baseRef = readAS5600Deg();
      
      stepper->enableOutputs();
      stepper->setCurrentPosition(0);
      stepper->setAcceleration(10000);
      stepper->setSpeedInHz(10000);
      
      Serial.println("\n  Tilting pendulum should move arm in SAME direction!");
      Serial.println("  Format: alpha(deg) -> accCmd -> armPos(steps)");
      Serial.println("  Press any key to stop...\n");
      
      while (!Serial.available()) {
        selectMux(0);
        float pNow = readAS5600Deg();
        float alphaRaw = getAngleDiffDeg(pNow, upRef);
        float alpha = ALPHA_SIGN * alphaRaw;  // APPLY THE SIGN!
        
        // Simple P controller for visualization
        float acc = 500.0f * alpha;  // Just P term for clarity
        if (acc > 5000) acc = 5000;
        if (acc < -5000) acc = -5000;
        
        // Integrate to position
        static float vel = 0;
        static float pos = 0;
        vel += acc * 0.02f;  // 50Hz
        vel *= 0.95f;  // damping
        pos += vel * 0.02f;
        if (pos > 200) pos = 200;
        if (pos < -200) pos = -200;
        
        stepper->moveTo((int32_t)pos);
        
        Serial.print("  alpha="); Serial.print(alpha, 1);
        Serial.print(" deg  accCmd="); Serial.print((int)acc);
        Serial.print("  armPos="); Serial.print((int)pos);
        Serial.println(" steps");
        
        delay(100);  // 10 Hz update
      }
      while (Serial.available()) Serial.read();
      
      stepper->moveTo(0);
      delay(300);
      stepper->disableOutputs();
      stepper->setAcceleration((uint32_t)MAX_ACC_STEPS);
      stepper->setSpeedInHz((uint32_t)MAX_SPEED_HZ);
      
      Serial.println("\n  Did arm move in SAME direction as pendulum tilt?");
      Serial.println("  YES -> Signs are correct, issue is gains/dynamics");
      Serial.println("  NO  -> Flip CTRL_SIGN by pressing B");
      Serial.println("========================================\n");
      return;
    }

    // NEW: Set motorSign directly
    if (c == 'M' || c == 'm') {
      int v = (int)Serial.parseInt();
      if (v == 1 || v == -1) motorSign = v;
      Serial.print("motorSign="); Serial.println(motorSign);
      return;
    }

    // Quick jump to TEST 3 only (upright live test)
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
      // Calibrate: pendulum UPRIGHT - directly use current angle as target
      selectMux(0);
      float pendUp = wrap360(readAS5600Deg());
      targetRawDeg = pendUp;  // Zero is where pendulum is NOW (upright)

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
    
    // Outer loop controls
    if (c == 'O' || c == 'o') { outerLoopEnabled = !outerLoopEnabled; Serial.print("outerLoop="); Serial.println(outerLoopEnabled ? "ON" : "OFF"); return; }
    if (c == 'K' || c == 'k') { float v = Serial.parseFloat(); if (v>=0) KTHETA=v; Serial.print("KTHETA="); Serial.println(KTHETA,1); return; }
    if (c == 'L' || c == 'l') { float v = Serial.parseFloat(); if (v>=0) KTHETADOT=v; Serial.print("KTHETADOT="); Serial.println(KTHETADOT,1); return; }
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
  Serial.println("  S       -> SIGN DIAGNOSTIC (run this first!)");
  Serial.println("  Z       -> Calibrate (hold pendulum UPRIGHT)");
  Serial.println("  E       -> Toggle manual engage");
  Serial.println("  M 1/-1  -> Set motorSign");
  Serial.println("  A 1/-1  -> Set ALPHA_SIGN (pendulum)");
  Serial.println("  H       -> Toggle THETA_SIGN (base encoder)");
  Serial.println("  B       -> Toggle CTRL_SIGN (controller)");
  Serial.println("  T       -> Toggle alpha debug prints");
  Serial.println("  J 10    -> Jog test (+10deg)");
  Serial.println("  P#, D#, I#, F# -> inner loop gains/filter");
  Serial.println("  O       -> Toggle outer loop (position correction)");
  Serial.println("  K#, L#  -> KTHETA, KTHETADOT (outer loop gains)");
  Serial.println("Log: t_ms,alphaRaw100,alphaDot100,theta100,thetaDot100,accCmd,velCmd,posCmd,clamped");
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
  float alphaDeg = (float)ALPHA_SIGN * alphaDegRaw;
  float alphaRawSigned = (float)ALPHA_SIGN * alphaDegRaw;  // signed raw angle

  // Deadband: ignore tiny alpha near upright to avoid chasing quantization/noise
  if (fabs(alphaDeg) < ALPHA_DEADBAND_DEG) alphaDeg = 0.0f;

  // Base for safety only
  selectMux(1);
  float baseDegRaw = getAngleDiffDeg(readAS5600Deg(), motorZeroDeg);
  float baseDeg = baseDegRaw;  // keep for backward compat / safety checks
  float thetaDeg = THETA_SIGN * baseDegRaw;

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
      float thetaDot = (thetaDeg - lastThetaDeg) / dt;
      lastThetaDeg = thetaDeg;
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

      // Engage only if calibrated + enabled + near upright + calm
      bool ok = (fabs(alphaDegRaw) < ENGAGE_PEND_DEG) && (fabs(alphaDotFilt) < ENGAGE_DOT_DEG);

      // Print engage gate signals
      static uint32_t dec2=0;
      if (++dec2 >= 20) { // 10 Hz
        dec2 = 0;
        Serial.print("raw="); Serial.print(alphaDegRaw,2);
        Serial.print(" dot="); Serial.print(alphaDotFilt,1);
        Serial.print(" enable="); Serial.println(ok ? 1 : 0);
      }

      if (ok) engageCount++;
      else engageCount = 0;

      if (isCalibrated && armEnabled && engageCount >= ENGAGE_COUNT_REQ) {
        // DON'T reset alphaDotFilt - keep the warm filter from IDLE!
        // Only reset the integrator and position commands
        lastAlphaRawSigned = alphaRawSigned;
        // alphaDotFilt already warm from IDLE - don't zero it!
        alphaInt     = 0.0f;
        thetaDotCmdMotor = 0.0f;
        thetaCmdStepsMotor = 0.0f;
        lastThetaDeg = thetaDeg;
        // thetaDotFilt already warm from IDLE - don't zero it!
        logDecim = 0;

        engageStartMs = millis();
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

      // Derivative from RAW signed angle (NO deadband!)
      float dAlpha = alphaRawSigned - lastAlphaRawSigned;
      while (dAlpha > 180.0f) dAlpha -= 360.0f;
      while (dAlpha < -180.0f) dAlpha += 360.0f;

      float alphaDot = dAlpha / dt;
      lastAlphaRawSigned = alphaRawSigned;

      alphaDotFilt += D_FILT_ALPHA * (alphaDot - alphaDotFilt);
      alphaDotFilt = constrain(alphaDotFilt, -ALPHADOT_CLAMP, ALPHADOT_CLAMP);

      // Deadband only for P/I (NOT for D)
      float alphaCtrl = alphaRawSigned;
      if (fabs(alphaCtrl) < ALPHA_DEADBAND_DEG) alphaCtrl = 0.0f;

      // Compute measured theta rate
      float thetaDot = (thetaDeg - lastThetaDeg) / dt;
      lastThetaDeg = thetaDeg;
      thetaDotFilt += D_FILT_THETA * (thetaDot - thetaDotFilt);
      thetaDotFilt = constrain(thetaDotFilt, -THETADOT_CLAMP, THETADOT_CLAMP);

      alphaInt += alphaCtrl * dt;
      alphaInt = constrain(alphaInt, -ALPHA_INT_CLAMP, ALPHA_INT_CLAMP);

      // Compute acceleration in physical units with CTRL_SIGN
      float accCmdPhysical = CTRL_SIGN * (ACC_KP * alphaCtrl + ACC_KD * alphaDotFilt + ACC_KI * alphaInt);

      // Ramp output on engage (short ramp to reach full power quickly)
      float ramp = (millis() - engageStartMs) * (1.0f / 50.0f); // 50 ms ramp (was 100ms)
      if (ramp > 1.0f) ramp = 1.0f;
      accCmdPhysical *= ramp;

      // Outer loop for base - pushes arm back toward center
      // Only active when pendulum is relatively stable (not fighting a fall)
      if (outerLoopEnabled && fabs(alphaDegRaw) < ALPHA_GATE_DEG && fabs(alphaDotFilt) < 50.0f) {
        // FIXED: Positive sign needed for stability (see simulation/gain_search.py)
        // Physics: outer loop must ADD to inner loop, not subtract
        float outer = +(KTHETA * thetaDeg + KTHETADOT * thetaDotFilt);
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

      if (thetaCmdStepsMotor >= limitSteps) {
        thetaCmdStepsMotor = limitSteps;
        if (thetaDotCmdMotor > 0) thetaDotCmdMotor = 0;   // STOP windup into +limit
      }
      if (thetaCmdStepsMotor <= -limitSteps) {
        thetaCmdStepsMotor = -limitSteps;
        if (thetaDotCmdMotor < 0) thetaDotCmdMotor = 0;   // STOP windup into -limit
      }

      stepper->moveTo((int32_t)thetaCmdStepsMotor);

      if (++logDecim >= 4) {  // 50 Hz logging
        logDecim = 0;
        uint32_t tMs = millis();
        int32_t alphaRaw100 = (int32_t)(alphaRawSigned * 100.0f);
        int32_t alphaDot100 = (int32_t)(alphaDotFilt * 100.0f);
        int32_t theta100 = (int32_t)(thetaDeg * 100.0f);
        int32_t thetaDot100 = (int32_t)(thetaDotFilt * 100.0f);
        int32_t accCmd = (int32_t)(accCmdPhysical);
        int32_t velCmd = (int32_t)(thetaDotCmdMotor);
        int32_t posCmd = (int32_t)(thetaCmdStepsMotor);
        int clamped = (fabs(thetaCmdStepsMotor) >= (LIM_MOTOR_DEG * STEPS_PER_DEG - 0.1f)) ? 1 : 0;

        Serial.print(tMs); Serial.print(",");
        Serial.print(alphaRaw100); Serial.print(",");
        Serial.print(alphaDot100); Serial.print(",");
        Serial.print(theta100); Serial.print(",");
        Serial.print(thetaDot100); Serial.print(",");
        Serial.print(accCmd); Serial.print(",");
        Serial.print(velCmd); Serial.print(",");
        Serial.print(posCmd); Serial.print(",");
        Serial.println(clamped);
      }
    } break;
  }
}