#include <Wire.h>

const int MPU = 0x68;

// ================= RAW MPU VALUES =================
int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

// ================= GYRO OFFSETS =================
float gyroXoffset = 0;
float gyroYoffset = 0;
float gyroZoffset = 0;

// ================= ANGLES =================
float pitch = 0;
float roll  = 0;

// ================= FILTERED ACC =================
float accPitchFiltered = 0;
float accRollFiltered  = 0;

// ================= OFFSETS =================
float pitchOffset = 0;
float rollOffset  = 0;

// ================= PID =================
float pitchSetpoint = 0;
float rollSetpoint  = 0;

float pitchError;
float rollError;

float pitchIntegral = 0;
float rollIntegral  = 0;

float pitchPrevError = 0;
float rollPrevError  = 0;

float pitchPID = 0;
float rollPID  = 0;

// STARTING PID VALUES
float Kp = 1.2;
float Ki = 0.02;
float Kd = 3.0;

// ================= TIMING =================
unsigned long prevTime;

void setup() {

  Serial.begin(115200);

  // Arduino UNO I2C
  Wire.begin();

  // Wake MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("KEEP MPU PERFECTLY STILL");

  delay(3000);

  // ================= GYRO CALIBRATION =================
  for (int i = 0; i < 3000; i++) {

    readMPU();

    gyroXoffset += GyX;
    gyroYoffset += GyY;
    gyroZoffset += GyZ;

    delay(2);
  }

  gyroXoffset /= 3000.0;
  gyroYoffset /= 3000.0;
  gyroZoffset /= 3000.0;

  Serial.println("GYRO CALIBRATED");

  delay(2000);

  // ================= ANGLE OFFSET CALIBRATION =================
  float pitchSum = 0;
  float rollSum  = 0;

  for (int i = 0; i < 1000; i++) {

    readMPU();

    float ax = (float)AcX;
    float ay = (float)AcY;
    float az = (float)AcZ;

    float accPitch =
      atan2(
        ax,
        sqrt((ay * ay) + (az * az))
      ) * 57.2958;

    float accRoll =
      atan2(
        ay,
        sqrt((ax * ax) + (az * az))
      ) * 57.2958;

    pitchSum += accPitch;
    rollSum  += accRoll;

    delay(2);
  }

  pitchOffset = pitchSum / 1000.0;
  rollOffset  = rollSum / 1000.0;

  Serial.println("OFFSET CALIBRATED");

  prevTime = micros();
}

void loop() {

  readMPU();

  // ================= LOOP TIME =================
  float dt =
    (micros() - prevTime) / 1000000.0;

  prevTime = micros();

  if (dt <= 0 || dt > 0.1)
    return;

  // ================= GYRO =================
  float gyroX =
    (GyX - gyroXoffset) / 131.0;

  float gyroY =
    (GyY - gyroYoffset) / 131.0;

  float gyroZ =
    (GyZ - gyroZoffset) / 131.0;

  // ================= FLOAT SAFE VALUES =================
  float ax = (float)AcX;
  float ay = (float)AcY;
  float az = (float)AcZ;

  // ================= ACCELEROMETER ANGLES =================
  float accPitch =
    atan2(
      ax,
      sqrt((ay * ay) + (az * az))
    ) * 57.2958;

  float accRoll =
    atan2(
      ay,
      sqrt((ax * ax) + (az * az))
    ) * 57.2958;

  // Remove offsets
  accPitch -= pitchOffset;
  accRoll  -= rollOffset;

  // ================= LOW PASS FILTER =================
  accPitchFiltered =
    0.9 * accPitchFiltered +
    0.1 * accPitch;

  accRollFiltered =
    0.9 * accRollFiltered +
    0.1 * accRoll;

  // ================= COMPLEMENTARY FILTER =================
  pitch =
    0.98 * (pitch + gyroY * dt) +
    0.02 * accPitchFiltered;

  roll =
    0.98 * (roll + gyroX * dt) +
    0.02 * accRollFiltered;

  // ================= SAFETY LIMIT =================
  if (pitch > 90 || pitch < -90)
    pitch = accPitchFiltered;

  if (roll > 90 || roll < -90)
    roll = accRollFiltered;

  // ================= PID =================

  // Error
  pitchError = pitchSetpoint - pitch;
  rollError  = rollSetpoint - roll;

  // Integral
  pitchIntegral += pitchError * dt;
  rollIntegral  += rollError * dt;

  // Prevent integral windup
  pitchIntegral = constrain(pitchIntegral, -50, 50);
  rollIntegral  = constrain(rollIntegral, -50, 50);

  // Derivative
static float pitchDerivativeFiltered = 0;
static float rollDerivativeFiltered = 0;

float pitchDerivative =
  (pitchError - pitchPrevError) / dt;

float rollDerivative =
  (rollError - rollPrevError) / dt;

// Low pass derivative filter
pitchDerivativeFiltered =
  0.7 * pitchDerivativeFiltered +
  0.3 * pitchDerivative;

rollDerivativeFiltered =
  0.7 * rollDerivativeFiltered +
  0.3 * rollDerivative;
  // PID output
  pitchPID =
      Kp * pitchError +
      Ki * pitchIntegral +
      Kd * pitchDerivativeFiltered;

  rollPID =
      Kp * rollError +
      Ki * rollIntegral +
      Kd * rollDerivativeFiltered;

  // Save previous
  pitchPrevError = pitchError;
  rollPrevError  = rollError;

  // ================= OUTPUT =================
  Serial.print("Pitch: ");
  Serial.print(pitch);

  Serial.print("  Roll: ");
  Serial.print(roll);

  Serial.print("  YawRate: ");
  Serial.print(gyroZ);

  Serial.print("  PitchPID: ");
  Serial.print(pitchPID);

  Serial.print("  RollPID: ");
  Serial.println(rollPID);

  delay(4);
}

void readMPU() {

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU, 14, true);

  if (Wire.available() == 14) {

    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();

    // Skip temperature
    Wire.read();
    Wire.read();

    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();
  }
}