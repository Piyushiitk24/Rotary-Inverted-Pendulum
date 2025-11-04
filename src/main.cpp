#include <Arduino.h>
#include <Wire.h>
#include <SoftwareWire.h>
#include <AS5600.h>
#include <AccelStepper.h>

// Stepper Motor
#define STEP_PIN 5
#define DIR_PIN  6
#define EN_PIN   7
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Pendulum Sensor on Hardware I2C
AS5600 as5600_pendulum(&Wire);

// Motor Sensor on Software I2C
#define MOTOR_SDA_PIN 22
#define MOTOR_SCL_PIN 24
#define AS5600_ADDRESS 0x36
#define AS5600_RAW_ANGLE_REG 0x0C
#define AS5600_STATUS_REG 0x0B
SoftwareWire motorWire(MOTOR_SDA_PIN, MOTOR_SCL_PIN);

// Motor settings
#define MOTOR_SPEED 1000
#define MOTOR_ACCEL 2000

// CALIBRATION DATA - STEP-BASED (much more reliable!)
long maxStepsLeft = -10;   // Default: 10 steps to the left
long maxStepsRight = 10;   // Default: 10 steps to the right
float pendulumZeroAngle = 0.0;
bool isCalibrated = false;

// Current readings
float pendulumAngle = 0.0;
float motorAngle = 0.0;
long motorSteps = 0;

// Sensor I2C helpers
uint16_t readAS5600Angle(SoftwareWire &wire) {
  wire.beginTransmission(AS5600_ADDRESS);
  wire.write(AS5600_RAW_ANGLE_REG);
  if (wire.endTransmission() != 0) return 0;
  wire.requestFrom(AS5600_ADDRESS, 2);
  if (wire.available() >= 2) {
    uint8_t high = wire.read();
    uint8_t low = wire.read();
    return (high << 8) | low;
  }
  return 0;
}

bool detectAS5600Magnet(SoftwareWire &wire) {
  wire.beginTransmission(AS5600_ADDRESS);
  wire.write(AS5600_STATUS_REG);
  if (wire.endTransmission() != 0) return false;
  wire.requestFrom(AS5600_ADDRESS, 1);
  if (wire.available()) {
    uint8_t status = wire.read();
    return (status & 0x20) != 0;
  }
  return false;
}

void updateSensorReadings() {
  // Read pendulum angle (Hardware I2C)
  uint16_t pendulum_raw = as5600_pendulum.readAngle();
  float pendulum_deg = (pendulum_raw * 360.0) / 4096.0;
  
  // Read motor angle (Software I2C) - for display only
  uint16_t motor_raw = readAS5600Angle(motorWire);
  motorAngle = (motor_raw * 360.0) / 4096.0;
  
  // Calculate pendulum angle relative to zero
  pendulumAngle = pendulum_deg - pendulumZeroAngle;
  
  // Normalize pendulum angle to -180 to +180
  while (pendulumAngle > 180.0) pendulumAngle -= 360.0;
  while (pendulumAngle < -180.0) pendulumAngle += 360.0;
  
  // Get stepper position
  motorSteps = stepper.currentPosition();
}

void printSensorReadings() {
  updateSensorReadings();
  Serial.print("Pendulum: ");
  Serial.print(pendulumAngle, 2);
  Serial.print("° \t Motor Sensor: ");
  Serial.print(motorAngle, 2);
  Serial.print("° \t Steps: ");
  Serial.println(motorSteps);
}

void printCalibrationData() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║         CALIBRATION DATA                   ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.print("Pendulum Zero: ");
  Serial.print(pendulumZeroAngle, 2);
  Serial.println("°");
  Serial.println("\n--- MOTOR STEP LIMITS ---");
  Serial.print("LEFT Limit:  ");
  Serial.print(maxStepsLeft);
  Serial.println(" steps");
  Serial.print("RIGHT Limit: ");
  Serial.print(maxStepsRight);
  Serial.println(" steps");
  Serial.print("Total Range: ");
  Serial.print(maxStepsRight - maxStepsLeft);
  Serial.println(" steps");
  
  if (isCalibrated) {
    Serial.println("\n✓ System is CALIBRATED and ready!");
  } else {
    Serial.println("\n⚠ Please complete calibration steps 1-3");
  }
  Serial.println("════════════════════════════════════════════\n");
}

void printMainMenu() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║  STEP-BASED CALIBRATION (RELIABLE!)       ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\n--- STEP 1: SET ZERO POSITION ---");
  Serial.println("1 - Set Zero (center arm, pendulum up, press Z)");
  Serial.println("\n--- STEP 2: VERIFY SENSORS ---");
  Serial.println("2 - Live Sensor Test");
  Serial.println("\n--- STEP 3: SET STEP LIMITS ---");
  Serial.println("3 - Set LEFT Limit (manually jog to left edge)");
  Serial.println("4 - Set RIGHT Limit (manually jog to right edge)");
  Serial.println("5 - Show Calibration Data");
  Serial.println("\n--- STEP 4: TEST & MOVE ---");
  Serial.println("6 - Test Full Range (move between limits)");
  Serial.println("7 - Jog Left (CW, -1 step)");
  Serial.println("8 - Jog Right (CCW, +1 step)");
  Serial.println("9 - Return to Zero");
  Serial.println("\n--- QUICK LIMITS (if you know them) ---");
  Serial.println("L - Set limits to ±10 steps (default)");
  Serial.println("W - Set limits to ±15 steps");
  Serial.println("M - Set limits to ±20 steps");
  Serial.println("\n--- DIAGNOSTICS ---");
  Serial.println("D - EMI Test (disable motor, check sensor stability)");
  Serial.println("\n--- UTILITIES ---");
  Serial.println("0 - I2C Bus Scan");
  Serial.println("C - Clear calibration");
  Serial.println("════════════════════════════════════════════");
  Serial.print("Enter choice: ");
}

void setZeroPosition() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║        STEP 1: SET ZERO POSITION           ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\nINSTRUCTIONS:");
  Serial.println("1. Position the motor arm at CENTER");
  Serial.println("2. Position the pendulum STRAIGHT UP (0°)");
  Serial.println("3. Press 'Z' to set zero");
  Serial.println("4. Press 'X' to cancel\n");
  
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'z' || c == 'Z') {
        stepper.setCurrentPosition(0);
        uint16_t pendulum_raw = as5600_pendulum.readAngle();
        pendulumZeroAngle = (pendulum_raw * 360.0) / 4096.0;
        
        Serial.println("\n✓ ZERO POSITION SET!");
        Serial.print("  Pendulum zero: ");
        Serial.print(pendulumZeroAngle, 2);
        Serial.println("°");
        Serial.println("  Stepper position: 0 steps");
        delay(2000);
        return;
      } else if (c == 'x' || c == 'X') {
        Serial.println("\nCancelled.");
        delay(1000);
        return;
      }
    }
    delay(50);
  }
}

void liveSensorTest() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║         LIVE SENSOR READINGS               ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\n⚠ Testing with MOTOR DRIVER DISABLED");
  Serial.println("This tests if motor driver causes interference.\n");
  Serial.println("Rotate pendulum and move motor arm.");
  Serial.println("Press 'S' to stop.\n");
  
  // DISABLE motor driver to test for EMI
  digitalWrite(EN_PIN, HIGH);
  delay(100);
  
  Serial.println("────────────────────────────────────────────");
  Serial.println("Pendulum(°) \tMotor(°) \tΔPend \tΔMotor");
  Serial.println("────────────────────────────────────────────");
  
  unsigned long lastPrint = millis();
  float lastPendulum = 0;
  float lastMotor = 0;
  int stableCount = 0;
  int jumpCount = 0;
  
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's' || c == 'S' || c == 'x' || c == 'X') {
        Serial.println("────────────────────────────────────────────");
        Serial.print("Stable readings: ");
        Serial.println(stableCount);
        Serial.print("Large jumps (>10°): ");
        Serial.println(jumpCount);
        Serial.println("\nStopped.\n");
        
        // Re-enable motor driver
        digitalWrite(EN_PIN, LOW);
        delay(1000);
        return;
      }
    }
    
    if (millis() - lastPrint >= 100) {
      lastPrint = millis();
      
      updateSensorReadings();
      
      // Calculate deltas
      float deltaPend = abs(pendulumAngle - lastPendulum);
      float deltaMotor = abs(motorAngle - lastMotor);
      
      // Track stability
      if (deltaPend < 2.0 && deltaMotor < 2.0) {
        stableCount++;
      }
      if (deltaPend > 10.0 || deltaMotor > 10.0) {
        jumpCount++;
        Serial.print("⚠ ");
      } else {
        Serial.print("  ");
      }
      
      Serial.print(pendulumAngle, 2);
      Serial.print("° \t");
      Serial.print(motorAngle, 2);
      Serial.print("° \t");
      Serial.print(deltaPend, 1);
      Serial.print("° \t");
      Serial.print(deltaMotor, 1);
      Serial.println("°");
      
      lastPendulum = pendulumAngle;
      lastMotor = motorAngle;
    }
  }
}

void manualStep(int direction) {
  // Force enable the motor
  digitalWrite(EN_PIN, LOW);
  delay(1);
  
  // Set direction (LOW = CW/left, HIGH = CCW/right)
  digitalWrite(DIR_PIN, direction > 0 ? HIGH : LOW);
  delayMicroseconds(5);
  
  // Generate ONE step pulse
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(2);  // 2us pulse width
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(2);
  
  // Update AccelStepper's internal position to match
  stepper.setCurrentPosition(stepper.currentPosition() + direction);
}

void setLeftLimit() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║       SET LEFT LIMIT (STEP-BASED)          ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\nINSTRUCTIONS:");
  Serial.println("Use keys to jog motor to LEFT edge:");
  Serial.println("  'A' = Move left 1 step");
  Serial.println("  'D' = Move right 1 step");
  Serial.println("  'L' = Set current position as LEFT limit");
  Serial.println("  'X' = Cancel\n");
  
  // Check enable pin state
  Serial.print("Enable pin: ");
  Serial.println(digitalRead(EN_PIN) == LOW ? "ENABLED" : "DISABLED!");
  
  updateSensorReadings();
  Serial.print("Starting position: ");
  Serial.print(motorSteps);
  Serial.println(" steps\n");
  
  while (true) {
    // Check serial immediately (no delay)
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'a' || c == 'A') {
        Serial.println("Stepping LEFT (CW)...");
        manualStep(-1);  // Move left
        delay(50);  // Wait for motor to settle
        
        updateSensorReadings();
        Serial.print("← Left | Position: ");
        Serial.print(motorSteps);
        Serial.println(" steps");
      } else if (c == 'd' || c == 'D') {
        Serial.println("Stepping RIGHT (CCW)...");
        manualStep(+1);  // Move right
        delay(50);  // Wait for motor to settle
        
        updateSensorReadings();
        Serial.print("→ Right | Position: ");
        Serial.print(motorSteps);
        Serial.println(" steps");
      } else if (c == 'l' || c == 'L') {
        maxStepsLeft = stepper.currentPosition();
        Serial.println("\n✓ LEFT LIMIT SET!");
        Serial.print("  Position: ");
        Serial.print(maxStepsLeft);
        Serial.println(" steps");
        delay(2000);
        return;
      } else if (c == 'x' || c == 'X') {
        Serial.println("\nCancelled.");
        delay(1000);
        return;
      }
    }
    delay(10); // Small delay to prevent CPU spinning
  }
}

void setRightLimit() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║      SET RIGHT LIMIT (STEP-BASED)          ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\nINSTRUCTIONS:");
  Serial.println("From LEFT edge, jog motor to RIGHT edge:");
  Serial.println("  'A' = Move left 1 step");
  Serial.println("  'D' = Move right 1 step");
  Serial.println("  'R' = Set current position as RIGHT limit");
  Serial.println("  'X' = Cancel\n");
  Serial.println("NOTE: Start from left edge position, jog right");
  
  // Check enable pin state
  Serial.print("Enable pin: ");
  Serial.println(digitalRead(EN_PIN) == LOW ? "ENABLED" : "DISABLED!");
  
  updateSensorReadings();
  Serial.print("Starting position: ");
  Serial.print(motorSteps);
  Serial.println(" steps\n");
  
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'a' || c == 'A') {
        Serial.println("Stepping LEFT (CW)...");
        manualStep(-1);  // Move left
        delay(50);
        
        updateSensorReadings();
        Serial.print("← Left | Position: ");
        Serial.print(motorSteps);
        Serial.println(" steps");
      } else if (c == 'd' || c == 'D') {
        Serial.println("Stepping RIGHT (CCW)...");
        manualStep(+1);  // Move right
        delay(50);
        
        updateSensorReadings();
        Serial.print("→ Right | Position: ");
        Serial.print(motorSteps);
        Serial.println(" steps");
      } else if (c == 'r' || c == 'R') {
        maxStepsRight = stepper.currentPosition();
        isCalibrated = true;
        Serial.println("\n✓ RIGHT LIMIT SET!");
        Serial.print("  Position: ");
        Serial.print(maxStepsRight);
        Serial.println(" steps");
        Serial.println("\n✓ CALIBRATION COMPLETE!");
        printCalibrationData();
        delay(3000);
        return;
      } else if (c == 'x' || c == 'X') {
        Serial.println("\nCancelled.");
        delay(1000);
        return;
      }
    }
    delay(10);
  }
}

void testFullRange() {
  if (!isCalibrated) {
    Serial.println("\nERROR: Please calibrate first!");
    delay(2000);
    return;
  }
  
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║         TEST FULL RANGE                    ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\n⚠ Motor will move between calibrated limits!");
  Serial.println("Press 'S' at any time to STOP.\n");
  delay(2000);
  
  Serial.println("Moving to LEFT limit...");
  stepper.moveTo(maxStepsLeft);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's' || c == 'S') {
        stepper.stop();
        digitalWrite(EN_PIN, HIGH); // Disable motor
        Serial.println("\n⚠ EMERGENCY STOP!");
        delay(2000);
        digitalWrite(EN_PIN, LOW); // Re-enable
        return;
      }
    }
  }
  Serial.print("At LEFT: ");
  Serial.print(stepper.currentPosition());
  Serial.println(" steps");
  delay(1000);
  
  Serial.println("Moving to RIGHT limit...");
  stepper.moveTo(maxStepsRight);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's' || c == 'S') {
        stepper.stop();
        digitalWrite(EN_PIN, HIGH); // Disable motor
        Serial.println("\n⚠ EMERGENCY STOP!");
        delay(2000);
        digitalWrite(EN_PIN, LOW); // Re-enable
        return;
      }
    }
  }
  Serial.print("At RIGHT: ");
  Serial.print(stepper.currentPosition());
  Serial.println(" steps");
  delay(1000);
  
  Serial.println("Returning to center...");
  stepper.moveTo(0);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's' || c == 'S') {
        stepper.stop();
        digitalWrite(EN_PIN, HIGH); // Disable motor
        Serial.println("\n⚠ EMERGENCY STOP!");
        delay(2000);
        digitalWrite(EN_PIN, LOW); // Re-enable
        return;
      }
    }
  }
  Serial.println("✓ Test complete!\n");
  delay(1000);
}

void jogLeft() {
  stepper.move(-1);
  stepper.runToPosition();
  updateSensorReadings();
  Serial.print("← Position: ");
  Serial.print(motorSteps);
  Serial.println(" steps");
  delay(300);
}

void jogRight() {
  stepper.move(1);
  stepper.runToPosition();
  updateSensorReadings();
  Serial.print("→ Position: ");
  Serial.print(motorSteps);
  Serial.println(" steps");
  delay(300);
}

void returnToZero() {
  Serial.println("Returning to zero...");
  stepper.moveTo(0);
  stepper.runToPosition();
  Serial.println("✓ At center");
  printSensorReadings();
  delay(500);
}

void setQuickLimits(long steps) {
  maxStepsLeft = -steps;
  maxStepsRight = steps;
  isCalibrated = true;
  Serial.print("\n✓ Limits set to ±");
  Serial.print(steps);
  Serial.println(" steps");
  printCalibrationData();
  delay(2000);
}

void runI2CScan() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║           I2C BUS SCANNER                  ║");
  Serial.println("╚════════════════════════════════════════════╝");
  
  Serial.println("\n--- Hardware I2C (pins 20/21) ---");
  byte hw_count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      if (addr == 0x36) Serial.print(" (AS5600)");
      Serial.println();
      hw_count++;
    }
  }
  if (hw_count == 0) Serial.println("  None");
  
  Serial.println("\n--- Software I2C (pins 22/24) ---");
  byte sw_count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    motorWire.beginTransmission(addr);
    if (motorWire.endTransmission() == 0) {
      Serial.print("  0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      if (addr == 0x36) Serial.print(" (AS5600)");
      Serial.println();
      sw_count++;
    }
  }
  if (sw_count == 0) Serial.println("  None");
  Serial.println();
  delay(2000);
}

void clearCalibration() {
  Serial.println("\nClearing calibration...");
  pendulumZeroAngle = 0.0;
  maxStepsLeft = -10;
  maxStepsRight = 10;
  isCalibrated = false;
  Serial.println("✓ Cleared\n");
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n╔════════════════════════════════════════════╗");
  Serial.println("║   ROTARY INVERTED PENDULUM - V2.0          ║");
  Serial.println("║   Step-Based Calibration (Reliable!)       ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
  
  Serial.println("⚠⚠⚠ SAFETY WARNING ⚠⚠⚠");
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║  1. CHECK MOTOR ARM IS FREE TO MOVE        ║");
  Serial.println("║  2. ENSURE WIRES HAVE SLACK                ║");
  Serial.println("║  3. BE READY TO DISCONNECT POWER           ║");
  Serial.println("║  4. LIMITS: ±280 steps = ±504° max         ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\nPress ANY KEY to continue...\n");
  
  while (!Serial.available()) {
    delay(100);
  }
  while (Serial.available()) Serial.read(); // Clear buffer
  
  Serial.println("\nInitializing...\n");
  
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  stepper.setMaxSpeed(MOTOR_SPEED);
  stepper.setAcceleration(MOTOR_ACCEL);
  stepper.setCurrentPosition(0);
  Serial.println("[OK] Motor Driver");
  
  Wire.begin();
  delay(100);
  as5600_pendulum.begin();
  if (!as5600_pendulum.detectMagnet()) {
    Serial.println("[ERROR] Pendulum Sensor");
  } else {
    Serial.println("[OK] Pendulum Sensor");
  }
  
  motorWire.begin();
  motorWire.setClock(100000);
  delay(100);
  if (!detectAS5600Magnet(motorWire)) {
    Serial.println("[ERROR] Motor Sensor");
  } else {
    Serial.println("[OK] Motor Sensor");
  }
  
  Serial.println("\n════════════════════════════════════════════");
  delay(1000);
  
  printMainMenu();
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') return;
    
    switch (c) {
      case '1': setZeroPosition(); printMainMenu(); break;
      case '2': liveSensorTest(); printMainMenu(); break;
      case '3': setLeftLimit(); printMainMenu(); break;
      case '4': setRightLimit(); printMainMenu(); break;
      case '5': printCalibrationData(); delay(2000); printMainMenu(); break;
      case '6': testFullRange(); printMainMenu(); break;
      case '7': jogLeft(); break;
      case '8': jogRight(); break;
      case '9': returnToZero(); printMainMenu(); break;
      case '0': runI2CScan(); printMainMenu(); break;
      case 'l': case 'L': setQuickLimits(10); printMainMenu(); break;
      case 'w': case 'W': setQuickLimits(15); printMainMenu(); break;
      case 'm': case 'M': setQuickLimits(20); printMainMenu(); break;
      case 'd': case 'D': liveSensorTest(); printMainMenu(); break;
      case 'c': case 'C': clearCalibration(); printMainMenu(); break;
      default: 
        Serial.println("Invalid."); 
        delay(300); 
        printMainMenu(); 
        break;
    }
  }
}
