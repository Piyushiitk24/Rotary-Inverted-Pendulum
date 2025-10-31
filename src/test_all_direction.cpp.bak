#include <Arduino.h>
#include <Wire.h>          // Hardware I2C (Bus 0)
#include <SoftwareWire.h>  // Software I2C (Bus 1)
#include <AS5600.h>
#include <AccelStepper.h>

// --- Stepper Motor (AccelStepper) ---
#define STEP_PIN 5
#define DIR_PIN  6
#define EN_PIN   7
// Interface type 1 = DRIVER (STEP/DIR)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// --- Sensor 1 (Pendulum) on Hardware I2C ---
AS5600 as5600_pendulum;

// --- Sensor 2 (Motor) on Software I2C ---
#define MOTOR_SDA_PIN 22
#define MOTOR_SCL_PIN 24
SoftwareWire motorWire(MOTOR_SDA_PIN, MOTOR_SCL_PIN);
AS5600 as5600_motor(&motorWire); // <-- FIX #1: Pass the bus to the constructor

// Test move distance (800 steps = 90 degrees at 1/16 microstepping)
const long TEST_MOVE = 800; 

// --- Function Prototypes ---
void printMenu();
void runMotorTest();
void runSensorTest();
bool checkSerialForExit();

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin("\n\nUnified Hardware Characterization Test");
  Serial.println("========================================");

  // --- Stepper Setup ---
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable driver
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(2000);
  stepper.setCurrentPosition(0);
  Serial.println("Motor Driver... Enabled.");

  // --- Pendulum Sensor (Hardware I2C) ---
  Wire.begin();
  as5600_pendulum.begin(); // This one uses the default 'Wire'
  if (!as5600_pendulum.detectMagnet()) {
    Serial.println("ERROR: Pendulum Sensor (Hardware I2C on 20/21) not detected!");
  } else {
    Serial.println("Pendulum Sensor... Connected.");
  }

  // --- Motor Sensor (Software I2C) ---
  motorWire.begin();
  as5600_motor.begin(); // <-- FIX #2: Call begin() with no arguments
  if (!as5600_motor.detectMagnet()) {
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
  if (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '1') {
      runMotorTest();
      printMenu();
    } else if (c == '2') {
      runSensorTest();
      printMenu();
    } else if (c == '\n' || c == '\r') {
      // Ignore newlines
    } else {
      Serial.println("Invalid choice. Try again.");
      printMenu();
    }
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
  stepper.runToPosition(); // Blocks until move is done

  Serial.println("...pausing for 3 seconds...");
  delay(3000);

  Serial.println("Returning to zero...");
  stepper.moveTo(0);
  stepper.runToPosition(); // Blocks until move is done

  Serial.println("--- Motor Test Complete ---");
}

/**
 * Task 2: Runs a continuous loop printing both sensor values.
 * Exits when 'x' is sent.
 */
void runSensorTest() {
  Serial.println("\n--- Running Live Sensor Test ---");
  Serial.println("Streaming live data... Type 'X' to exit.");
  Serial.println("-------------------------------------------------");
  Serial.println("Pendulum Angle \t Motor Angle");
  Serial.println("-------------------------------------------------");

  // Clear serial buffer
  while(Serial.available()) Serial.read();
  
  while (true) {
    // Read from Hardware Bus
    float pendulum_deg = (as5600_pendulum.readAngle() * 360.0) / 4096.0;

    // Read from Software Bus
    float motor_deg = (as5600_motor.readAngle() * 360.0) / 4096.0;

    Serial.print(pendulum_deg, 2);
    Serial.print(" deg \t\t");
    Serial.print(motor_deg, 2);
    Serial.println(" deg");

    delay(100); // 10 Hz refresh rate

    // Check for exit command
    if (checkSerialForExit()) {
      Serial.println("--- Sensor Test Complete ---");
      break; // Exit the while(true) loop
    }
  }
}