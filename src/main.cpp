/**
 * Pendulum Drop Test (Dynamic)
 * Purpose: High-speed logging (200Hz) to capture free-fall dynamics.
 * Hardware: AS5600 on I2C Mux Channel 0.
 */

#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

#define SERIAL_BAUD     500000
#define TCA_ADDRESS     0x70
#define PENDULUM_CH     0      // Ch 0

AS5600 pendSensor;
float zeroOffset = 0.0;
unsigned long lastLog = 0;

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000); 

  tcaSelect(PENDULUM_CH);
  if(pendSensor.detectMagnet() == 0) {
    // Retry once
    delay(100);
    if(pendSensor.detectMagnet() == 0) {
       Serial.println("ERROR: Magnet missing Ch0");
       while(1);
    }
  }

  Serial.println("# --- DROP TEST READY ---");
  Serial.println("# 1. Hang DOWN.");
  Serial.println("# 2. Send 'Z' or 'z'.");
  Serial.println("# 3. Lift to 90 and drop.");
}

void loop() {
  // 1. Zero Command (Case Insensitive)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'Z' || c == 'z') {
      tcaSelect(PENDULUM_CH);
      zeroOffset = pendSensor.rawAngle() * 0.08789;
      Serial.print("# ZERO SET! New Offset: ");
      Serial.println(zeroOffset);
    }
  }

  // 2. High Speed Logging (5ms = 200Hz)
  if (millis() - lastLog >= 5) {
    lastLog = millis();
    
    tcaSelect(PENDULUM_CH);
    float raw = pendSensor.rawAngle() * 0.08789;
    float angle = raw - zeroOffset;

    // Normalize
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;

    Serial.print(millis());
    Serial.print(",");
    Serial.println(angle, 2);
  }
}
