#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

AS5600 as5600;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  Serial.println("AS5600 Sensor Direction Test");
  Serial.println("---------------------------------");
  
  if (!as5600.begin()) {
    Serial.println("ERROR: AS5600 not detected! Check wiring (SDA/SCL).");
    while (1);
  }
  Serial.println("AS5600 connected.");
  Serial.println("Move the pendulum by hand and observe the 'Raw Angle'.");
  Serial.println("Note which direction (clockwise/counter-clockwise) makes the numbers INCREASE.");
}

void loop() {
  // Read the raw angle (0-4095)
  uint16_t raw_angle = as5600.readAngle();
  
  // Convert to degrees
  float degrees = (raw_angle * 360.0) / 4096.0;

  Serial.print("Raw Angle: ");
  Serial.print(raw_angle);
  Serial.print("\t Degrees: ");
  Serial.println(degrees, 2);
  
  delay(100); // Print 10 times per second
}
