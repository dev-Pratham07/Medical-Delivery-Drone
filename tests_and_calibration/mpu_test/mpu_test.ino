/*
 * MPU6050 TEST — Arduino Uno
 * Pins:
 * SDA -> A4
 * SCL -> A5
 * VCC -> 3.3V or 5V (GY-521 module)
 * GND -> GND
 */

#include <Wire.h>

#define MPU_ADDR 0x68

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {

  Serial.begin(115200);

  Wire.begin();

  // Wake MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(100);

  // Check connection
  Wire.beginTransmission(MPU_ADDR);

  if (Wire.endTransmission() == 0) {
    Serial.println("MPU6050 DETECTED");
  } else {
    Serial.println("MPU6050 NOT FOUND");
    while (1);
  }
}

void loop() {

  // Start reading at accel register
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  // Request 14 bytes
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();

  Wire.read();
  Wire.read();

  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();

  Serial.print("ACCEL X:");
  Serial.print(ax);

  Serial.print(" Y:");
  Serial.print(ay);

  Serial.print(" Z:");
  Serial.print(az);

  Serial.print(" | GYRO X:");
  Serial.print(gx);

  Serial.print(" Y:");
  Serial.print(gy);

  Serial.print(" Z:");
  Serial.println(gz);

  delay(200);
}