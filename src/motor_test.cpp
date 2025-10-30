#include <Arduino.h>
#include <AccelStepper.h>
#include <SoftwareWire.h> // For the motor sensor
#include <AS5600.h>

// --- Stepper ---
#define STEP_PIN 5
#define DIR_PIN  6
#define EN_PIN   7
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// --- Motor Sensor (Software I2C) ---
#define MOTOR_SDA_PIN 22
#define MOTOR_SCL_PIN 24
SoftwareWire motorWire(MOTOR_SDA_PIN, MOTOR_SCL_PIN);
AS5600 as5600_motor;

// --- Safety Limit ---
// 75 deg * (3200 steps / 360 deg) = 666 steps
const long MAX_POSITION_STEPS = 666;

// Calibration
float MOTOR_ZERO_OFFSET = 0.0; // Will be set in setup()

void setup() {
  Serial.begin(115200);
  Serial.println("Interactive Motor Test with SENSOR FEEDBACK");
  Serial.println("--------------------------------------------");

  // --- Motor Sensor Setup ---
  motorWire.begin();
  as5600_motor.begin(&motorWire);
  if (!as5600_motor.detectMagnet()) {
    Serial.println("ERROR: Motor Sensor (Software I2C) not detected!");
    while(1);
  }
  Serial.println("Motor Sensor connected.");

  // --- Stepper Setup ---
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable driver
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(2000);
  
  // --- HOMING ---
  Serial.println("ACTION: Manually move the rotary arm to the CENTER position.");
  Serial.println("Press [ENTER] to confirm and zero both motor and sensor.");
  while (!Serial.available());
  Serial.read(); // Clear buffer
  
  stepper.setCurrentPosition(0);
  MOTOR_ZERO_OFFSET = (as5600_motor.readAngle() * 360.0) / 4096.0;
  Serial.print("Motor position zeroed. Sensor offset calibrated to: ");
  Serial.println(MOTOR_ZERO_OFFSET);

  Serial.println("\nCommands:");
  Serial.println(" 'R' -> Move to +75 degrees (RIGHT limit)");
  Serial.println(" 'L' -> Move to -75 degrees (LEFT limit)");
  Serial.println(" 'Z' -> Move to Zero (CENTER)");
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == 'R' || c == 'r') {
      Serial.println("Moving to RIGHT limit (+666 steps)...");
      stepper.moveTo(MAX_POSITION_STEPS);
    } 
    else if (c == 'L' || c == 'l') {
      Serial.println("Moving to LEFT limit (-666 steps)...");
      stepper.moveTo(-MAX_POSITION_STEPS);
    } 
    else if (c == 'Z' || c == 'z') {
      Serial.println("Moving to ZERO (center)...");
      stepper.moveTo(0);
    }
  }

  // Run the stepper motor
  stepper.run();

  // ----- Sensor Feedback -----
  // We only print feedback if the motor has stopped moving
  if (!stepper.isRunning()) {
    // Read the current angle
    float raw_angle = (as5600_motor.readAngle() * 360.0) / 4096.0;
    
    // Apply the zero offset
    float calibrated_angle = raw_angle - MOTOR_ZERO_OFFSET;
    
    // Handle wrap-around
    if (calibrated_angle > 180.0) calibrated_angle -= 360.0;
    else if (calibrated_angle < -180.0) calibrated_angle += 360.0;

    Serial.print("Motor at rest. Commanded Steps: ");
    Serial.print(stepper.currentPosition());
    Serial.print("\t Sensor Angle: ");
    Serial.print(calibrated_angle, 2);
    Serial.println(" deg");
    
    delay(200); // Prevent spamming
  }
}
