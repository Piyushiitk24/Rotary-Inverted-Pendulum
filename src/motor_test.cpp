#include <Arduino.h>
#include <AccelStepper.h>

// Stepper Motor Pins
#define STEP_PIN 5
#define DIR_PIN  6
#define EN_PIN   7

// AccelStepper Driver Setup
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Safety Limit: 75 Degrees
// 75 deg * (3200 steps / 360 deg) = 666 steps
const long MAX_POSITION_STEPS = 666;

void setup() {
	Serial.begin(115200);
	Serial.println("Interactive Motor Direction Test");
	Serial.println("---------------------------------");
  
	// Enable the driver (Active LOW)
	pinMode(EN_PIN, OUTPUT);
	digitalWrite(EN_PIN, LOW);

	// Configure stepper settings
	stepper.setMaxSpeed(1000);      // Safe speed in steps/sec
	stepper.setAcceleration(2000);  // Safe acceleration in steps/sec^2
  
	// --- HOMING ---
	Serial.println("ACTION: Manually move the rotary arm to the CENTER position.");
	Serial.println("Press [ENTER] to confirm and zero the motor.");
	while (!Serial.available());
	Serial.read(); // Clear buffer
  
	stepper.setCurrentPosition(0);
	Serial.println("Motor position zeroed.");
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

	// This is the most important part.
	// It is NON-BLOCKING and must be called in every loop.
	// It checks if a step is due and performs it.
	stepper.run();
}

