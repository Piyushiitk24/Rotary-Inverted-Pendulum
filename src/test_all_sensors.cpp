#include <Arduino.h>
#include <Wire.h>          // Hardware I2C (Bus 0)
#include <SoftwareWire.h>  // Software I2C (Bus 1)
#include <AS5600.h>

// --- Sensor 1 (Pendulum) on Hardware I2C ---
AS5600 as5600_pendulum;

// --- Sensor 2 (Motor) on Software I2C ---
#define MOTOR_SDA_PIN 22
#define MOTOR_SCL_PIN 24
SoftwareWire motorWire(MOTOR_SDA_PIN, MOTOR_SCL_PIN);
AS5600 as5600_motor;

void setup() {
  Serial.begin(115200);
  Serial.println("Dual AS5600 Sensor Test");
  Serial.println("-------------------------");

  // Initialize Hardware I2C (Bus 0)
  Wire.begin();
  as5600_pendulum.begin(); // Defaults to 'Wire'
  if (!as5600_pendulum.detectMagnet()) {
    Serial.println("ERROR: Pendulum Sensor (Hardware I2C on 20/21) not detected!");
  } else {
    Serial.println("Pendulum Sensor (Hardware I2C) connected!");
  }

  // Initialize Software I2C (Bus 1)
  motorWire.begin();
  as5600_motor.begin(&motorWire); // Pass the software wire object
  if (!as5600_motor.detectMagnet()) {
    Serial.println("ERROR: Motor Sensor (Software I2C on 22/24) not detected!");
  } else {
    Serial.println("Motor Sensor (Software I2C) connected!");
  }

  Serial.println("\nMove both parts by hand and observe the values.");
  Serial.println("Pendulum Angle \t Motor Angle");
}

void loop() {
  // Read from Hardware Bus
  float pendulum_deg = (as5600_pendulum.readAngle() * 360.0) / 4096.0;

  // Read from Software Bus
  float motor_deg = (as5600_motor.readAngle() * 360.0) / 4096.0;

  Serial.print(pendulum_deg, 2);
  Serial.print(" deg \t\t");
  Serial.print(motor_deg, 2);
  Serial.println(" deg");

  delay(100);
}
