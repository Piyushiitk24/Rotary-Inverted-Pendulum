#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <FastAccelStepper.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>

// ============================================================
// Hysteresis Sweep Test (60° span, 5° steps, high speed/accel)
// - Span: [-30°, +30°] (60° total)
// - Step size: 5° (commanded by step count via DEG_PER_STEP)
// - Logs at each point after dwell: steps + expected_deg + sensor angle + raw counts
// - Repeats NUM_CYCLES full cycles (one cycle = -end -> +end -> -end)
// ============================================================

// --- PINS ---
#define STEP_PIN 11
#define DIR_PIN  6
#define EN_PIN   7

static const uint32_t SERIAL_BAUD = 500000;

// --- I2C MUX ---
static const uint8_t TCA_ADDR = 0x70;
static inline void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// --- SENSOR ---
AS5600 motorSensor(&Wire);

// --- MECHANICS / SIGNS ---
static const float DEG_PER_STEP = 0.225f; // microstep geometry
static const int   SENSOR_SIGN  = -1;     // if target and sensor have opposite signs, flip this
static const int   STEP_SIGN    = +1;

// --- TEST SETTINGS ---
static const float SPAN_DEG      = 60.0f;     // total span
static const float STEP_DEG      = 5.0f;      // sample spacing
static const float HALF_SPAN_DEG = SPAN_DEG * 0.5f; // 30 deg

// NUM_CYCLES: one cycle = -end -> +end -> -end
static const uint8_t NUM_CYCLES  = 5;

static const float BASE_LIMIT_DEG = 80.0f;    // safety limit (encoder-based)
static const uint32_t DWELL_MS = 150;

// "Higher speed & accel"
static const uint32_t MAX_SPEED_HZ = 6000;
static const uint32_t ACCEL_HZ2    = 80000;

// --- LOGGING TAGS (so tools/balance_plot.py can save it) ---
static const char* TAG_START = "[BALANCE_LOG_START]";
static const char* TAG_COLS  = "[BALANCE_COLUMNS]";
static const char* TAG_END   = "[BALANCE_LOG_END]";

// --- Objects ---
FastAccelStepperEngine engine;
FastAccelStepper* stepper = nullptr;

// --- Zero reference ---
static uint16_t motorZeroRaw = 0;
static float    motorZeroDegRaw = 0.0f;

// --- Sweep state ---
static bool running = false;
static int8_t dir = +1;            // +1: idx increasing, -1: idx decreasing
static uint8_t cycle = 0;          // completed full cycles
static uint32_t lastMoveMs = 0;
static bool waitingForSettle = false;

// We avoid float comparisons by stepping an integer index.
static const int N_STEPS = (int)lroundf(SPAN_DEG / STEP_DEG); // 60/5 = 12
static int idx = 0; // 0..N_STEPS, where 0 => -30deg, N_STEPS => +30deg

// ---------------- Helpers ----------------
static inline float normalizeAngle(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

static inline float rawCountToDeg(uint16_t raw) {
  return (raw * 360.0f) / 4096.0f;
}

static inline long degToSteps(float deg) {
  float s = (deg / DEG_PER_STEP) * (float)STEP_SIGN;
  return lroundf(s);
}

static inline float stepsToDeg(long steps) {
  return ((float)steps * DEG_PER_STEP) / (float)STEP_SIGN;
}

static inline float idxToTargetDeg(int idxVal) {
  // idx=0 => -HALF_SPAN, idx=N_STEPS => +HALF_SPAN
  return -HALF_SPAN_DEG + (float)idxVal * STEP_DEG;
}

static void readMotor(uint16_t& raw, float& degRaw, float& angleDeg) {
  tcaSelect(1);
  raw = motorSensor.readAngle();
  degRaw = rawCountToDeg(raw);
  float delta = degRaw - motorZeroDegRaw;
  angleDeg = normalizeAngle((float)SENSOR_SIGN * delta);
}

static void logRow() {
  const uint32_t nowMs = millis();

  const float targetDeg = idxToTargetDeg(idx);
  const long targetSteps = degToSteps(targetDeg);

  const long stepPos = stepper ? stepper->getCurrentPosition() : 0;
  const float expectedDeg = stepsToDeg(stepPos);

  uint16_t encRaw = 0;
  float encDegRaw = 0.0f;
  float sensorDeg = 0.0f;
  readMotor(encRaw, encDegRaw, sensorDeg);

  const float errorDeg = expectedDeg - sensorDeg;

  // NUMERIC-ONLY columns (logger-friendly):
  // time_ms,cycle,dir,idx,target_deg,target_steps,step_pos,expected_deg,sensor_deg,enc_raw,enc_deg_raw,error_deg
  Serial.print(nowMs); Serial.print(",");
  Serial.print((int)cycle); Serial.print(",");
  Serial.print((int)dir); Serial.print(",");
  Serial.print(idx); Serial.print(",");
  Serial.print(targetDeg, 2); Serial.print(",");
  Serial.print(targetSteps); Serial.print(",");
  Serial.print(stepPos); Serial.print(",");
  Serial.print(expectedDeg, 2); Serial.print(",");
  Serial.print(sensorDeg, 2); Serial.print(",");
  Serial.print(encRaw); Serial.print(",");
  Serial.print(encDegRaw, 3); Serial.print(",");
  Serial.println(errorDeg, 2);
}

static void stopTest(const __FlashStringHelper* reason) {
  running = false;
  waitingForSettle = false;
  if (stepper) stepper->forceStop();
  digitalWrite(EN_PIN, HIGH);
  Serial.print(TAG_END); Serial.print(",0,"); Serial.println(reason);
}

static void commandCurrentIdxMove() {
  const float targetDeg = idxToTargetDeg(idx);
  const long targetSteps = degToSteps(targetDeg);
  stepper->moveTo(targetSteps);
  lastMoveMs = millis();
  waitingForSettle = true;
}

static void startTest() {
  if (!stepper) return;

  digitalWrite(EN_PIN, LOW); // enable motor

  running = true;
  cycle = 0;
  dir = +1;
  idx = 0; // start at -end

  Serial.print(TAG_START); Serial.println(",0,hysteresis_60deg_5deg");
  Serial.print(TAG_COLS);
  Serial.println(",time_ms,cycle,dir,idx,target_deg,target_steps,step_pos,expected_deg,sensor_deg,enc_raw,enc_deg_raw,error_deg");

  commandCurrentIdxMove(); // go to start point (in case you're not already there)
}

// ---------------- Commands ----------------
static void doZero() {
  tcaSelect(1);
  motorZeroRaw = motorSensor.readAngle();
  motorZeroDegRaw = rawCountToDeg(motorZeroRaw);

  if (stepper) stepper->setCurrentPosition(0);

  Serial.print(F("[INFO],zero_enc_raw,")); Serial.println(motorZeroRaw);
  Serial.print(F("[INFO],zero_enc_deg_raw,")); Serial.println(motorZeroDegRaw, 3);
  Serial.println(F("[INFO],stepper_pos_set_to_0"));
}

static void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  Z : zero encoder + set stepper position to 0 (do this at center)"));
  Serial.println(F("  S : start sweep test"));
  Serial.println(F("  X : stop test"));
  Serial.println(F("  T : snapshot"));
}

static void snapshot() {
  uint16_t encRaw = 0; float encDegRaw = 0; float sensorDeg = 0;
  readMotor(encRaw, encDegRaw, sensorDeg);
  long stepPos = stepper ? stepper->getCurrentPosition() : 0;
  float expectedDeg = stepsToDeg(stepPos);

  Serial.print(F("[SNAP],steps,")); Serial.print(stepPos);
  Serial.print(F(",expected_deg,")); Serial.print(expectedDeg, 2);
  Serial.print(F(",sensor_deg,")); Serial.print(sensorDeg, 2);
  Serial.print(F(",enc_raw,")); Serial.print(encRaw);
  Serial.print(F(",enc_deg_raw,")); Serial.println(encDegRaw, 3);
}

static void handleCommand(char c) {
  c = (char)toupper((unsigned char)c);
  switch (c) {
    case 'H': printHelp(); break;
    case 'Z': doZero(); break;
    case 'S': if (!running) startTest(); break;
    case 'X': stopTest(F("user_stop")); break;
    case 'T': snapshot(); break;
    default:
      Serial.print(F("[ERR],unknown_cmd,")); Serial.println(c);
      printHelp();
      break;
  }
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(SERIAL_BAUD);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);

  tcaSelect(1);
  motorSensor.begin();

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); // disabled by default (active-low)

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) {
    Serial.println(F("[FATAL] stepperConnectToPin failed"));
    while (1) delay(100);
  }
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(EN_PIN, true);
  stepper->setAutoEnable(false);

  stepper->setSpeedInHz(MAX_SPEED_HZ);
  stepper->setAcceleration(ACCEL_HZ2);

  Serial.println(F("Hysteresis Sweep Test Ready."));
  Serial.print(F("[INFO],SPAN_DEG,")); Serial.println(SPAN_DEG, 1);
  Serial.print(F("[INFO],STEP_DEG,")); Serial.println(STEP_DEG, 1);
  Serial.print(F("[INFO],N_STEPS,")); Serial.println(N_STEPS);
  Serial.print(F("[INFO],NUM_CYCLES,")); Serial.println(NUM_CYCLES);
  Serial.print(F("[INFO],MAX_SPEED_HZ,")); Serial.println(MAX_SPEED_HZ);
  Serial.print(F("[INFO],ACCEL_HZ2,")); Serial.println(ACCEL_HZ2);
  Serial.print(F("[INFO],DEG_PER_STEP,")); Serial.println(DEG_PER_STEP, 6);
  Serial.print(F("[INFO],SENSOR_SIGN,")); Serial.println(SENSOR_SIGN);
  Serial.print(F("[INFO],STEP_SIGN,")); Serial.println(STEP_SIGN);
  printHelp();
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') continue;
    handleCommand(c);
  }

  if (!running || !stepper) return;

  // Safety
  uint16_t encRaw = 0; float encDegRaw = 0.0f; float sensorDeg = 0.0f;
  readMotor(encRaw, encDegRaw, sensorDeg);
  if (fabs(sensorDeg) > BASE_LIMIT_DEG) {
    stopTest(F("base_limit"));
    return;
  }

  // Wait until movement finishes, then dwell, then log & advance
  if (!stepper->isRunning()) {
    if (!waitingForSettle) {
      waitingForSettle = true;
      lastMoveMs = millis();
      return;
    }

    if (millis() - lastMoveMs < DWELL_MS) return;

    // Log this point
    logRow();

    // Advance index
    if (dir > 0) {
      if (idx < N_STEPS) idx++;
      else dir = -1; // already at +end: reverse
    } else { // dir < 0
      if (idx > 0) idx--;
      else {
        // We arrived back at -end => completed one full cycle
        cycle++;
        if (cycle >= NUM_CYCLES) {
          stopTest(F("test_complete"));
          return;
        }
        dir = +1; // start next cycle
      }
    }

    commandCurrentIdxMove();
  } else {
    waitingForSettle = false; // moving
  }
}