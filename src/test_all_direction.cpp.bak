#include <Arduino.h>
#include <Wire.h>          // Hardware I2C (Bus 0)
#include <SoftwareWire.h>  // Software I2C (Bus 1)
#include <AS5600.h>
#include <AccelStepper.h>

// --- Stepper Motor (AccelStepper) ---
#define STEP_PIN 5
#define DIR_PIN  6
#define EN_PIN   7
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// --- Sensor 1 (Pendulum) on Hardware I2C ---
AS5600 as5600_pendulum(&Wire); // Pass the bus to the constructor

// --- Sensor 2 (Motor) - MANUAL I2C HANDLING ---
#define MOTOR_SDA_PIN 22
#define MOTOR_SCL_PIN 24
#define AS5600_ADDRESS 0x36
#define AS5600_RAW_ANGLE_REG 0x0C
#define AS5600_STATUS_REG 0x0B

SoftwareWire motorWire(MOTOR_SDA_PIN, MOTOR_SCL_PIN);

// Helper functions for manual AS5600 on SoftwareWire
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
    return (status & 0x20) != 0; // Magnet detected bit
  }
  return false;
}

// --- Other Globals ---
const long TEST_MOVE = 800; 
unsigned long sensorTimestamp = 0; // For non-blocking timer
bool sensorTestRunning = false;

// --- Function Prototypes ---
void printMenu();
void runMotorTest();
void startSensorTest();
void updateSensorTest();
bool checkSerialForExit();

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\nUnified Hardware Characterization Test");
  Serial.println("========================================");

  // --- Stepper Setup ---
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(2000);
  stepper.setCurrentPosition(0);
  Serial.println("Motor Driver... Enabled.");

  // --- Pendulum Sensor (Hardware I2C) ---
  Wire.begin();
  as5600_pendulum.begin();
  if (!as5600_pendulum.detectMagnet()) {
    Serial.println("ERROR: Pendulum Sensor (Hardware I2C on 20/21) not detected!");
  } else {
    Serial.println("Pendulum Sensor... Connected.");
  }

  // --- Motor Sensor (Software I2C) ---
  motorWire.begin();
  motorWire.setClock(100000); // <-- FIX #1: Set stable 100kHz clock
  if (!detectAS5600Magnet(motorWire)) {
    Serial.println("ERROR: Motor Sensor (Software I2C on 22/24) not detected!");
  } else {
    Serial.println("Motor Sensor... Connected.");
  }

  printMenu();
}

// ==========================================
// MAIN LOOP (Menu Handler)
// ==========================================
void loop() {
  // Check for menu commands only if the sensor test isn't running
  if (Serial.available() > 0 && !sensorTestRunning) {
    char c = Serial.read();

    if (c == '1') {
      runMotorTest();
      printMenu();
    } else if (c == '2') {
      startSensorTest();
      sensorTestRunning = true;
    } else if (c == '\n' || c == '\r') {
      // Ignore newlines
    } else {
      Serial.println("Invalid choice. Try again.");
      printMenu();
    }
  }

  // --- FIX #2: Run the sensor test as a non-blocking loop ---
  if (sensorTestRunning) {
    updateSensorTest();
  }
}

// ==========================================
// HELPER FUNCTIONS
// ==========================================

void printMenu() {
  Serial.println("\n--- Main Menu ---");
  Serial.println("Select a test to run:");
  Serial.println(" 1. Motor Direction Test (Moves 90 deg & returns)");
  Serial.println(" 2. Live Sensor Test (Streams both sensor angles)");
  Serial.print("Enter choice (1 or 2): ");
}

/**
 * Checks Serial for an 'x' or 'X' to exit a loop.
 * Returns true if exit command was received.
 */
bool checkSerialForExit() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'x' || c == 'X') {
      sensorTestRunning = false; // Stop the sensor loop
      return true;
    }
  }
  return false;
}

/**
 * Task 1: Runs the motor in its "forward" direction,
 * waits, and returns to zero.
 */
void runMotorTest() {
  Serial.println("\n--- Running Motor Test ---");
  Serial.println("Moving motor 'forward' (+800 steps)...");
  Serial.println("Observe: Did the arm move CW or CCW?");
  
  stepper.moveTo(TEST_MOVE);
  stepper.runToPosition(); // This is a blocking call, which is fine for this test

  Serial.println("...pausing for 3 seconds...");
  delay(3000); // Blocking delay is fine here, as it's not in the main sensor loop

  Serial.println("Returning to zero...");
  stepper.moveTo(0);
  stepper.runToPosition();

  Serial.println("--- Motor Test Complete ---");
}

/**
 * Task 2: Initializes the continuous sensor test.
 */
void startSensorTest() {
  Serial.println("\n--- Running Live Sensor Test ---");
  Serial.println("Streaming live data... Type 'X' to exit.");
  Serial.println("-------------------------------------------------");
  Serial.println("Pendulum Angle \t Motor Angle");
  Serial.println("-------------------------------------------------");

  while(Serial.available()) Serial.read(); // Clear serial buffer
  sensorTimestamp = millis(); // Set the timer
}

/**
 * Task 2: This is now called by loop() constantly.
 * It only prints data every 100ms.
 */
void updateSensorTest() {
  // First, check for exit command. Do this every loop.
  if (checkSerialForExit()) {
    Serial.println("--- Sensor Test Complete ---");
    printMenu(); // Show the menu again
    return;
  }

  // Second, check if 100ms have passed.
  if (millis() - sensorTimestamp >= 100) {
    sensorTimestamp = millis(); // Reset the timer

    // Read from Hardware Bus (using library)
    float pendulum_deg = (as5600_pendulum.readAngle() * 360.0) / 4096.0;

    // Read from Software Bus (manual I2C)
    uint16_t motor_raw = readAS5600Angle(motorWire);
    float motor_deg = (motor_raw * 360.0) / 4096.0;

    Serial.print(pendulum_deg, 2);
    Serial.print(" deg \t\t");
    Serial.print(motor_deg, 2);
    Serial.println(" deg");
  }
}