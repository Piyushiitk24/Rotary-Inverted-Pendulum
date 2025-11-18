#include <Arduino.h>
#include <SoftwareWire.h>
#include <AccelStepper.h>
#include <math.h>

// ============================================================
//  MOTOR SENSOR vs STEPPER STEPS CALIBRATION TOOL
// ============================================================
// Flow:
//   1. Option 1 -> quick sensor diagnostics
//   2. Option 2 -> Zero both stepper count and sensor (press 'Z')
//   3. Option 3 -> Jog calibration (A/D jog, L/R set limits, Q to exit)
// Every jog completion prints:
//   - Stepper position (steps)
//   - Expected° = steps * 1.8 (no microstep scaling)
//   - AS5600 reading (absolute + zeroed relative)
//   - Δ = sensor(rel) - expected
// ============================================================

// Stepper pins
constexpr uint8_t STEP_PIN = 5;
constexpr uint8_t DIR_PIN  = 6;
constexpr uint8_t EN_PIN   = 7;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Jog behaviour
constexpr float  JOG_SPEED           = 2000; // steps/sec
constexpr float  JOG_ACCEL           = 8000; // steps/sec^2
constexpr float  EXPECTED_DEG_PER_STEP = 1.8f; // fallback assumption
constexpr float  TARGET_JOG_DEGREES    = 10.0f; // desired diagnostic move size once scale known
constexpr long   MIN_JOG_STEPS = 5;
constexpr long   MAX_JOG_STEPS = 4000;
constexpr float  EXPERIMENT_SPEED      = 1200.0f; // conservative speed to avoid missed steps
constexpr float  EXPERIMENT_ACCEL      = 6000.0f; // gentle acceleration for correlation sweep
constexpr int    EXPERIMENT_SAMPLES    = 24;      // number of measurement points per sweep
constexpr long   EXPERIMENT_MARGIN_STEPS = 50;    // stay away from end stops by this many steps
float calibratedDegPerStep = EXPECTED_DEG_PER_STEP;
bool  degPerStepDerived = false;
long  jogIncrementSteps = 50;  // default until auto-tuned

// Software I2C AS5600 (motor shaft sensor)
constexpr uint8_t MOTOR_SDA_PIN = 22;
constexpr uint8_t MOTOR_SCL_PIN = 24;
constexpr uint8_t AS5600_ADDRESS = 0x36;
constexpr uint8_t AS5600_RAW_ANGLE_REG = 0x0C;
constexpr uint8_t AS5600_STATUS_REG    = 0x0B;

SoftwareWire motorWire(MOTOR_SDA_PIN, MOTOR_SCL_PIN);

// State machine
enum SystemState {
  STATE_STARTUP,
  STATE_IDLE,
  STATE_ZEROING,
  STATE_CALIBRATION
};

SystemState currentState = STATE_STARTUP;

// Zero + limit tracking
bool motorZeroSet = false;
float motorZeroAngle = 0.0f;

long stepsAtLeftLimit = 0;
long stepsAtRightLimit = 0;
bool leftLimitSet = false;
bool rightLimitSet = false;
float sensorLeftDeg = 0.0f;
float sensorRightDeg = 0.0f;
float sensorLeftAbsDeg = 0.0f;
float sensorRightAbsDeg = 0.0f;

bool jogReportPending = false;
const char* pendingReportLabel = nullptr;

long computeExperimentSpanSteps();
void runSensorCorrelationExperiment();

// ------------------------------------------------------------
// Utility helpers
// ------------------------------------------------------------

float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float signedAngleDelta(float fromDeg, float toDeg) {
  float delta = toDeg - fromDeg;
  while (delta > 180.0f) delta -= 360.0f;
  while (delta < -180.0f) delta += 360.0f;
  return delta;
}

float getDegPerStep() {
  return degPerStepDerived ? calibratedDegPerStep : EXPECTED_DEG_PER_STEP;
}

void announceJogIncrement() {
  Serial.print(F("Jog increment: "));
  Serial.print(jogIncrementSteps);
  Serial.print(F(" steps (~"));
  Serial.print(jogIncrementSteps * getDegPerStep(), 2);
  Serial.println(F("°)."));
}

void adjustJogIncrementByFactor(float factor) {
  long newSteps = lroundf(jogIncrementSteps * factor);
  if (newSteps < MIN_JOG_STEPS) newSteps = MIN_JOG_STEPS;
  if (newSteps > MAX_JOG_STEPS) newSteps = MAX_JOG_STEPS;
  jogIncrementSteps = newSteps;
  announceJogIncrement();
}

bool autoSetJogIncrementForDegrees(float desiredDeg) {
  if (!degPerStepDerived) {
    Serial.println(F("Need a measured deg/step scale before degree-based jogs."));
    return false;
  }
  float scale = getDegPerStep();
  long steps = lroundf(desiredDeg / scale);
  if (steps < MIN_JOG_STEPS) steps = MIN_JOG_STEPS;
  if (steps > MAX_JOG_STEPS) steps = MAX_JOG_STEPS;
  jogIncrementSteps = steps;
  Serial.print(F("Jog size tuned for ~"));
  Serial.print(desiredDeg, 2);
  Serial.println(F("° moves."));
  announceJogIncrement();
  return true;
}

void insertionSort(uint16_t arr[], int n) {
  for (int i = 1; i < n; ++i) {
    uint16_t key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      --j;
    }
    arr[j + 1] = key;
  }
}

uint16_t readMotorRawAngleSingle() {
  motorWire.beginTransmission(AS5600_ADDRESS);
  motorWire.write(AS5600_RAW_ANGLE_REG);
  if (motorWire.endTransmission() != 0) {
    return 0xFFFF;
  }
  if (motorWire.requestFrom(AS5600_ADDRESS, (uint8_t)2) != 2) {
    return 0xFFFF;
  }
  if (!motorWire.available()) {
    return 0xFFFF;
  }
  uint16_t msb = motorWire.read();
  if (!motorWire.available()) {
    return 0xFFFF;
  }
  uint16_t lsb = motorWire.read();
  return (msb << 8) | lsb;
}

uint16_t readMotorRawAngleMedian() {
  uint16_t samples[3];
  for (int i = 0; i < 3; ++i) {
    samples[i] = readMotorRawAngleSingle();
    if (samples[i] == 0xFFFF) {
      return 0xFFFF;
    }
    delayMicroseconds(100);
  }
  insertionSort(samples, 3);
  return samples[1];
}

bool detectAS5600Magnet() {
  motorWire.beginTransmission(AS5600_ADDRESS);
  motorWire.write(AS5600_STATUS_REG);
  if (motorWire.endTransmission() != 0) {
    return false;
  }
  if (motorWire.requestFrom(AS5600_ADDRESS, (uint8_t)1) != 1) {
    return false;
  }
  if (!motorWire.available()) {
    return false;
  }
  uint8_t status = motorWire.read();
  return (status & 0x20) != 0; // MD bit
}

float rawToDegrees(uint16_t raw) {
  return (raw * 360.0f) / 4096.0f;
}

float readMotorAbsoluteDegrees() {
  uint16_t raw = readMotorRawAngleMedian();
  if (raw == 0xFFFF) {
    return NAN;
  }
  return rawToDegrees(raw);
}

float readMotorRelativeDegrees() {
  if (!motorZeroSet) {
    return NAN;
  }
  float absDeg = readMotorAbsoluteDegrees();
  if (isnan(absDeg)) {
    return NAN;
  }
  return normalizeAngle(absDeg - motorZeroAngle);
}

void enableMotor() {
  digitalWrite(EN_PIN, LOW);
}

void disableMotor() {
  digitalWrite(EN_PIN, HIGH);
  stepper.stop();
}

void printPositionReport(const char* label) {
  long steps = stepper.currentPosition();
  float expectedDeg = steps * getDegPerStep();
  uint16_t raw = readMotorRawAngleMedian();
  if (raw == 0xFFFF) {
    Serial.println(F("I2C read failed (SoftwareWire). Check wiring/pull-ups."));
    return;
  }
  float absDeg = rawToDegrees(raw);
  float relDeg = motorZeroSet ? normalizeAngle(absDeg - motorZeroAngle) : NAN;

  Serial.println(F("────────────────────────────────────────────────"));
  Serial.print(label);
  Serial.print(F(" | Steps: "));
  Serial.print(steps);
  Serial.print(F(" | Expected° (steps*scale): "));
  Serial.print(expectedDeg, 2);
  Serial.print(F(" | Sensor° abs: "));
  Serial.print(absDeg, 2);

  if (motorZeroSet) {
    Serial.print(F(" | Sensor° rel: "));
    Serial.print(relDeg, 2);
    Serial.print(F(" | Δ (rel - expected): "));
    Serial.print(relDeg - expectedDeg, 2);
  } else {
    Serial.print(F(" | Sensor rel: (zero not set)"));
  }

  Serial.print(F(" | Raw: "));
  Serial.print(raw);
  Serial.println();
  Serial.println(F("────────────────────────────────────────────────"));
}

void printMenu() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║  MOTOR SENSOR CALIBRATION MENU             ║"));
  Serial.println(F("╚════════════════════════════════════════════╝"));
  Serial.print(F("State: "));
  switch (currentState) {
    case STATE_STARTUP: Serial.println(F("STARTUP")); break;
    case STATE_IDLE: Serial.println(F("IDLE")); break;
    case STATE_ZEROING: Serial.println(F("ZEROING")); break;
    case STATE_CALIBRATION: Serial.println(F("CALIBRATION")); break;
  }
  Serial.print(F("Zero set: "));
  Serial.println(motorZeroSet ? F("YES") : F("NO"));
  Serial.println(F("\n1 - Sensor diagnostics"));
  Serial.println(F("2 - Set zero position (press 'Z' when prompted)"));
  Serial.println(F("3 - Enter jog calibration (A/D/L/R/Q)"));
  Serial.println(F("4 - Automated sensor correlation experiment"));
  Serial.println(F("T - Show current status"));
  Serial.println(F("════════════════════════════════════════════"));
  Serial.print(F("Choice: "));
}

void printStatus() {
  Serial.println(F("\n--- Current Status ---"));
  Serial.print(F("Zero set: "));
  Serial.println(motorZeroSet ? F("YES") : F("NO"));
  Serial.print(F("Deg/step scale: "));
  Serial.println(getDegPerStep(), 4);
  Serial.print(F("Steps position: "));
  Serial.println(stepper.currentPosition());
  Serial.print(F("Jog increment: "));
  Serial.print(jogIncrementSteps);
  Serial.print(F(" steps (~"));
  Serial.print(jogIncrementSteps * getDegPerStep(), 2);
  Serial.println(F("°)"));
  if (motorZeroSet) {
    float rel = readMotorRelativeDegrees();
    Serial.print(F("Sensor relative angle: "));
    if (isnan(rel)) {
      Serial.println(F("(read error)"));
    } else {
      Serial.print(rel, 2);
      Serial.println(F("°"));
    }
  } else {
    float absDeg = readMotorAbsoluteDegrees();
    Serial.print(F("Sensor absolute angle: "));
    if (isnan(absDeg)) Serial.println(F("(read error)"));
    else {
      Serial.print(absDeg, 2);
      Serial.println(F("°"));
    }
  }
  if (leftLimitSet || rightLimitSet) {
    Serial.print(F("Left limit steps: "));
    Serial.print(leftLimitSet ? stepsAtLeftLimit : 0);
    Serial.print(F(" | Right limit steps: "));
    Serial.println(rightLimitSet ? stepsAtRightLimit : 0);
  }
  Serial.println(F("-----------------------"));
}

void sensorDiagnostics() {
  Serial.println(F("\nRunning sensor diagnostics..."));
  if (detectAS5600Magnet()) {
    Serial.println(F("Magnet detected ✅"));
  } else {
    Serial.println(F("Magnet NOT detected ⚠ - check gap/pull-ups."));
  }
  float absDeg = readMotorAbsoluteDegrees();
  if (isnan(absDeg)) {
    Serial.println(F("Failed to read angle - I2C issues suspected."));
  } else {
    Serial.print(F("Current absolute angle: "));
    Serial.print(absDeg, 2);
    Serial.println(F("°"));
  }
  printMenu();
}

void startZeroing() {
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║      ZERO POSITION SETUP                   ║"));
  Serial.println(F("╚════════════════════════════════════════════╝"));
  Serial.println(F("1. Position motor arm at CENTER"));
  Serial.println(F("2. Ensure magnet is detected"));
  Serial.println(F("3. Press 'Z' to record zero reference"));
  currentState = STATE_ZEROING;
}

void finishZeroing() {
  float absDeg = readMotorAbsoluteDegrees();
  if (isnan(absDeg)) {
    Serial.println(F("Cannot set zero - sensor read failed."));
    return;
  }
  motorZeroAngle = absDeg;
  motorZeroSet = true;
  stepper.setCurrentPosition(0);
  Serial.println(F("\n✅ Zero recorded."));
  printPositionReport("Zero snapshot");
  currentState = STATE_IDLE;
  printMenu();
}

void startCalibration() {
  if (!motorZeroSet) {
    Serial.println(F("\nZero not set. Run option 2 first."));
    printMenu();
    return;
  }
  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║      JOG CALIBRATION MODE                  ║"));
  Serial.println(F("╚════════════════════════════════════════════╝"));
  Serial.println(F("Use 'A'/'D' to jog -/+"));
  Serial.println(F("Use 'L' to record LEFT limit, 'R' for RIGHT"));
  Serial.println(F("Use 'Q' to exit calibration"));
  Serial.println(F("Use '+' / '-' to grow/shrink jog size, '0' to target ~10°"));
  Serial.println(F("Each jog completion prints steps vs sensor data."));
  leftLimitSet = rightLimitSet = false;
  sensorLeftAbsDeg = sensorRightAbsDeg = 0.0f;
  enableMotor();
  stepper.setMaxSpeed(JOG_SPEED);
  stepper.setAcceleration(JOG_ACCEL);
  announceJogIncrement();
  currentState = STATE_CALIBRATION;
}

void summarizeCalibration() {
  if (leftLimitSet && rightLimitSet) {
    Serial.println(F("\nCalibration summary:"));
    long stepSpanCounts = stepsAtRightLimit - stepsAtLeftLimit;
    long stepSpanAbs = labs(stepSpanCounts);
    if (stepSpanAbs == 0) {
      Serial.println(F("Span is zero. Jog farther before setting limits."));
      return;
    }

    float sensorSpanSigned = signedAngleDelta(sensorLeftAbsDeg, sensorRightAbsDeg);
    float sensorSpanAbs = fabs(sensorSpanSigned);

    float simpleExpectedSpan = stepSpanCounts * EXPECTED_DEG_PER_STEP;
    Serial.print(F("Steps span: "));
    Serial.print(stepSpanCounts);
    Serial.print(F(" | Sensor° span: "));
    Serial.print(sensorSpanSigned, 2);
    Serial.print(F(" | Fallback expected° span: "));
    Serial.print(simpleExpectedSpan, 2);
    Serial.print(F(" | Deg/step (measured): "));
    float measuredScale = sensorSpanAbs / stepSpanAbs;
    Serial.println(measuredScale, 4);

    calibratedDegPerStep = measuredScale;
    degPerStepDerived = true;
    autoSetJogIncrementForDegrees(TARGET_JOG_DEGREES);

    float midAngleAbs = normalizeAngle(sensorLeftAbsDeg + sensorSpanSigned * 0.5f);
    motorZeroAngle = midAngleAbs;
    motorZeroSet = true;
    sensorLeftDeg = normalizeAngle(sensorLeftAbsDeg - motorZeroAngle);
    sensorRightDeg = normalizeAngle(sensorRightAbsDeg - motorZeroAngle);

    Serial.print(F("Auto-zero set to midpoint angle: "));
    Serial.print(motorZeroAngle, 2);
    Serial.println(F("° (relative sensor values updated)."));

    Serial.print(F("Left limit relative: "));
    Serial.print(sensorLeftDeg, 2);
    Serial.print(F("° | Right limit relative: "));
    Serial.print(sensorRightDeg, 2);
    Serial.println(F("°"));
    Serial.println(F("(Tip: run option 2 -> Z to realign stepper counts using this midpoint.)"));
  } else {
    Serial.println(F("\nCalibration summary unavailable - set both limits."));
  }
}

long computeExperimentSpanSteps() {
  long defaultSpan = 600;
  if (leftLimitSet && rightLimitSet) {
    long leftSpan = labs(stepsAtLeftLimit) - EXPERIMENT_MARGIN_STEPS;
    long rightSpan = labs(stepsAtRightLimit) - EXPERIMENT_MARGIN_STEPS;
    if (leftSpan < MIN_JOG_STEPS || rightSpan < MIN_JOG_STEPS) {
      return defaultSpan;
    }
    long safeSpan = min(leftSpan, rightSpan);
    if (safeSpan < MIN_JOG_STEPS) {
      return defaultSpan;
    }
    return safeSpan;
  }
  return defaultSpan;
}

void runSensorCorrelationExperiment() {
  if (!motorZeroSet) {
    Serial.println(F("\nZero not set. Run option 2 and press 'Z' before starting the experiment."));
    printMenu();
    return;
  }

  const long spanSteps = computeExperimentSpanSteps();
  if (spanSteps < MIN_JOG_STEPS) {
    Serial.println(F("\nSpan too small for experiment. Increase jog range and set limits first."));
    printMenu();
    return;
  }

  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║  SENSOR vs STEPPER CORRELATION EXPERIMENT  ║"));
  Serial.println(F("╚════════════════════════════════════════════╝"));
  Serial.print(F("Target span: ±"));
  Serial.print(spanSteps);
  Serial.println(F(" steps"));
  Serial.print(F("Speed: "));
  Serial.print(EXPERIMENT_SPEED);
  Serial.print(F(" steps/s | Accel: "));
  Serial.print(EXPERIMENT_ACCEL);
  Serial.println(F(" steps/s²"));
  Serial.println(F("Sampling evenly across sweep (sensor vs expected deg)."));
  Serial.println(F("Press reset if you need to abort."));

  enableMotor();
  stepper.setMaxSpeed(EXPERIMENT_SPEED);
  stepper.setAcceleration(EXPERIMENT_ACCEL);

  const int samples = max(1, EXPERIMENT_SAMPLES);
  float sumAbsErr = 0.0f;
  float sumSqErr = 0.0f;
  float maxAbsErr = 0.0f;
  int sampleCount = 0;

  Serial.println(F("Idx | Steps | Expected° | Sensor° | Δ (sensor-expected)"));
  Serial.println(F("----------------------------------------------------------"));

  for (int i = 0; i <= samples; ++i) {
    float frac = (float)i / (float)samples;
    long targetSteps = lroundf(-spanSteps + (2.0f * spanSteps * frac));
    if (targetSteps < -spanSteps) targetSteps = -spanSteps;
    if (targetSteps > spanSteps) targetSteps = spanSteps;

    stepper.moveTo(targetSteps);
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
    delay(5); // allow sensor to settle slightly

    float expectedDeg = stepper.currentPosition() * getDegPerStep();
    float sensorDeg = readMotorRelativeDegrees();
    if (isnan(sensorDeg)) {
      Serial.println(F("Sensor read failed during experiment. Aborting."));
      break;
    }

    float error = sensorDeg - expectedDeg;
    float absErr = fabs(error);
    sumAbsErr += absErr;
    sumSqErr += error * error;
    if (absErr > maxAbsErr) maxAbsErr = absErr;
    ++sampleCount;

    Serial.print(i);
    Serial.print(F(" | "));
    Serial.print(stepper.currentPosition());
    Serial.print(F(" | "));
    Serial.print(expectedDeg, 2);
    Serial.print(F(" | "));
    Serial.print(sensorDeg, 2);
    Serial.print(F(" | "));
    Serial.println(error, 3);
  }

  stepper.moveTo(0);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  disableMotor();

  if (sampleCount == 0) {
    Serial.println(F("No samples captured. Ensure sensor is connected and try again."));
    printMenu();
    return;
  }

  float meanAbsErr = sumAbsErr / sampleCount;
  float rmsErr = sqrtf(sumSqErr / sampleCount);

  Serial.println(F("\nExperiment summary:"));
  Serial.print(F("Samples: "));
  Serial.println(sampleCount);
  Serial.print(F("Span used: ±"));
  Serial.print(spanSteps);
  Serial.println(F(" steps"));
  Serial.print(F("Mean |error|: "));
  Serial.print(meanAbsErr, 3);
  Serial.println(F("°"));
  Serial.print(F("RMS error: "));
  Serial.print(rmsErr, 3);
  Serial.println(F("°"));
  Serial.print(F("Max |error|: "));
  Serial.print(maxAbsErr, 3);
  Serial.println(F("°"));

  Serial.println(F("════════════════════════════════════════════"));
  printMenu();
}

void exitCalibration() {
  disableMotor();
  jogReportPending = false;
  pendingReportLabel = nullptr;
  summarizeCalibration();
  currentState = STATE_IDLE;
  printMenu();
}

bool queueJog(long deltaSteps, const char* label) {
  if (stepper.distanceToGo() != 0) {
    Serial.println(F("Motor busy - wait for current jog to finish."));
    return false;
  }
  stepper.move(deltaSteps);
  jogReportPending = true;
  pendingReportLabel = label;
  Serial.print(label);
  Serial.print(F(" jog queued ("));
  Serial.print(deltaSteps);
  Serial.println(F(" steps)."));
  return true;
}

void recordLimit(bool isLeft) {
  float absDeg = readMotorAbsoluteDegrees();
  if (isnan(absDeg)) {
    Serial.println(F("Cannot record limit - sensor read failed."));
    return;
  }
  float relDeg = motorZeroSet ? normalizeAngle(absDeg - motorZeroAngle) : NAN;
  if (isLeft) {
    stepsAtLeftLimit = stepper.currentPosition();
    sensorLeftDeg = relDeg;
    sensorLeftAbsDeg = absDeg;
    leftLimitSet = true;
    printPositionReport("LEFT LIMIT SET");
  } else {
    stepsAtRightLimit = stepper.currentPosition();
    sensorRightDeg = relDeg;
    sensorRightAbsDeg = absDeg;
    rightLimitSet = true;
    printPositionReport("RIGHT LIMIT SET");
  }
}

void handleCalibrationCommand(char cmd) {
  switch (cmd) {
    case 'a': case 'A':
      queueJog(-jogIncrementSteps, "← Jogging LEFT");
      break;
    case 'd': case 'D':
      queueJog(+jogIncrementSteps, "→ Jogging RIGHT");
      break;
    case 'l': case 'L':
      recordLimit(true);
      break;
    case 'r': case 'R':
      recordLimit(false);
      break;
    case '+':
      adjustJogIncrementByFactor(1.5f);
      break;
    case '-':
      adjustJogIncrementByFactor(0.5f);
      break;
    case '0':
      if (!autoSetJogIncrementForDegrees(TARGET_JOG_DEGREES)) {
        Serial.println(F("Capture both limits first so the degree scale is known."));
      }
      break;
    case 'q': case 'Q':
      exitCalibration();
      break;
    default:
      Serial.println(F("Unknown calibration command. Use A/D/L/R/Q."));
      break;
  }
}

void handleCommand(char cmd) {
  if (cmd == '\n' || cmd == '\r') return;

  if (currentState == STATE_ZEROING) {
    if (cmd == 'z' || cmd == 'Z') {
      finishZeroing();
    } else {
      Serial.println(F("Press 'Z' to record zero."));
    }
    return;
  }

  if (currentState == STATE_CALIBRATION) {
    handleCalibrationCommand(cmd);
    return;
  }

  // IDLE / STARTUP commands
  switch (cmd) {
    case '1':
      sensorDiagnostics();
      break;
    case '2':
      startZeroing();
      break;
    case '3':
      startCalibration();
      break;
    case '4':
      runSensorCorrelationExperiment();
      break;
    case 't': case 'T':
      printStatus();
      break;
    default:
      Serial.println(F("Unknown option"));
      printMenu();
      break;
  }
}

void checkSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') continue;
    Serial.println();
    handleCommand(c);
  }
}

void setup() {
  pinMode(EN_PIN, OUTPUT);
  disableMotor();
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  Serial.begin(115200);
  while (!Serial && millis() < 2000) { ; }

  motorWire.begin();
  Serial.println(F("Software I2C sensor bus initialized (SDA=22, SCL=24)."));

  stepper.setMaxSpeed(JOG_SPEED);
  stepper.setAcceleration(JOG_ACCEL);
  stepper.setCurrentPosition(0);

  Serial.println(F("\n\n╔════════════════════════════════════════════╗"));
  Serial.println(F("║  MOTOR SENSOR vs STEPS COMPARISON TOOL     ║"));
  Serial.println(F("╚════════════════════════════════════════════╝"));
  currentState = STATE_IDLE;
  printMenu();
}

void loop() {
  checkSerialInput();

  if (currentState == STATE_CALIBRATION) {
    stepper.run();
    if (jogReportPending && stepper.distanceToGo() == 0) {
      jogReportPending = false;
      const char* label = pendingReportLabel ? pendingReportLabel : "Jog complete";
      printPositionReport(label);
    }
  }
}
