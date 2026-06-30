#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

// ================= RFID PINS =================
#define SS_PIN  A1
#define RST_PIN 8

MFRC522 rfid(SS_PIN, RST_PIN);

byte authorizedUID[4] = {0x9D, 0xA4, 0x5F, 0x06};

// ================= SERVO =================
Servo myServo;

bool          servoOpen     = false;
unsigned long servoOpenTime = 0;

const unsigned long SERVO_HOLD_MS = 10000;

// ================= RC RECEIVER =================
volatile uint32_t ch1Start, ch2Start, ch3Start, ch4Start, ch5Start;

volatile uint16_t ch1Value = 1500;
volatile uint16_t ch2Value = 1500;
volatile uint16_t ch3Value = 1000;
volatile uint16_t ch4Value = 1500;
volatile uint16_t ch5Value = 1000;

volatile uint8_t lastPortD;

// ================= MPU RAW =================
const int MPU = 0x68;

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

// ================= ANGLE OFFSETS =================
float pitchOffset = 0;
float rollOffset  = 0;

// ================= PID SETPOINTS =================
float pitchSetpoint = 0;
float rollSetpoint  = 0;

// ================= PID STATE =================
float pitchError, rollError;

float pitchIntegral = 0;
float rollIntegral  = 0;

float pitchPrevError = 0;
float rollPrevError  = 0;

float pitchPID = 0;
float rollPID  = 0;

// ================= PID GAINS =================
float Kp = 1.2;
float Ki = 0.02;
float Kd = 3.0;

// ================= TIMING =================
unsigned long prevTime;

// =====================================================================

void setup() {

  Serial.begin(115200);

  // ================= MPU6050 INIT =================
  Wire.begin();
  Wire.setClock(400000);

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

  // ================= ANGLE OFFSET CALIBRATION =================
  float pitchSum = 0;
  float rollSum  = 0;

  for (int i = 0; i < 1000; i++) {

    readMPU();

    float ax = (float)AcX;
    float ay = (float)AcY;
    float az = (float)AcZ;

    pitchSum += atan2(ax, sqrt((ay * ay) + (az * az))) * 57.2958;
    rollSum  += atan2(ay, sqrt((ax * ax) + (az * az))) * 57.2958;

    delay(2);
  }

  pitchOffset = pitchSum / 1000.0;
  rollOffset  = rollSum  / 1000.0;

  Serial.println("OFFSET CALIBRATED");

  // ================= RFID + SERVO INIT =================
  SPI.begin();
  rfid.PCD_Init();

  myServo.attach(7);
  myServo.write(0);

  Serial.println("RFID READY");

  // ================= RC RECEIVER INIT =================
  pinMode(2, INPUT); // CH1
  pinMode(3, INPUT); // CH2
  pinMode(4, INPUT); // CH3
  pinMode(5, INPUT); // CH4
  pinMode(6, INPUT); // CH5

  lastPortD = PIND;

  // External interrupts for CH1 CH2
  attachInterrupt(digitalPinToInterrupt(2), ch1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), ch2ISR, CHANGE);

  // Pin change interrupts for CH3 CH4 CH5
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20); // D4
  PCMSK2 |= (1 << PCINT21); // D5
  PCMSK2 |= (1 << PCINT22); // D6

  Serial.println("RECEIVER READY");

  prevTime = micros();
}

// =====================================================================

void loop() {

  readMPU();

  // ================= LOOP TIME =================
  float dt = (micros() - prevTime) / 1000000.0;

  prevTime = micros();

  if (dt <= 0 || dt > 0.1)
    return;

  // ================= GYRO =================
  float gyroX = (GyX - gyroXoffset) / 131.0;
  float gyroY = (GyY - gyroYoffset) / 131.0;
  float gyroZ = (GyZ - gyroZoffset) / 131.0;

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
  pitchError = pitchSetpoint - pitch;
  rollError  = rollSetpoint  - roll;

  pitchIntegral += pitchError * dt;
  rollIntegral  += rollError  * dt;

  pitchIntegral = constrain(pitchIntegral, -50, 50);
  rollIntegral  = constrain(rollIntegral,  -50, 50);

  static float pitchDerivativeFiltered = 0;
  static float rollDerivativeFiltered  = 0;

  float pitchDerivative = (pitchError - pitchPrevError) / dt;
  float rollDerivative  = (rollError  - rollPrevError)  / dt;

  pitchDerivativeFiltered =
    0.7 * pitchDerivativeFiltered +
    0.3 * pitchDerivative;

  rollDerivativeFiltered =
    0.7 * rollDerivativeFiltered +
    0.3 * rollDerivative;

  pitchPID =
    Kp * pitchError +
    Ki * pitchIntegral +
    Kd * pitchDerivativeFiltered;

  rollPID =
    Kp * rollError +
    Ki * rollIntegral +
    Kd * rollDerivativeFiltered;

  pitchPrevError = pitchError;
  rollPrevError  = rollError;

  // ================= SNAPSHOT RC VALUES =================
  noInterrupts();
  uint16_t ch1 = ch1Value;
  uint16_t ch2 = ch2Value;
  uint16_t ch3 = ch3Value;
  uint16_t ch4 = ch4Value;
  uint16_t ch5 = ch5Value;
  interrupts();

  // ================= OUTPUT (every 5 loops) =================
  static uint8_t printCount = 0;

  if (++printCount >= 5) {

    printCount = 0;

    char p[8], r[8], y[8], pp[8], rp[8];

    dtostrf(pitch,    6, 2, p);
    dtostrf(roll,     6, 2, r);
    dtostrf(gyroZ,    6, 2, y);
    dtostrf(pitchPID, 6, 2, pp);
    dtostrf(rollPID,  6, 2, rp);

    char buf[120];
    sprintf(buf,
      "P:%s R:%s Y:%s PP:%s RP:%s C1:%u C2:%u C3:%u C4:%u C5:%u SW:%s",
      p, r, y, pp, rp,
      ch1, ch2, ch3, ch4, ch5,
      ch5 > 1600 ? "ON" : "OFF"
    );
    Serial.println(buf);
  }
  // ================= SWITCH PRIORITY =================
  if (ch5 > 1600) {

    // Switch ON — force servo open, RFID completely paused
    if (!servoOpen) {

      myServo.write(90);
      servoOpen     = true;
      servoOpenTime = millis();

      Serial.println("SWITCH ON: SERVO OPEN");
    }
  }
  else {

    // Switch OFF — RFID has full control

    // ================= SERVO HOLD TIMER =================
    if (servoOpen && (millis() - servoOpenTime >= SERVO_HOLD_MS)) {

      myServo.write(0);
      servoOpen = false;

      Serial.println("SERVO RESET");
    }

    // ================= RFID CHECK =================
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {

      Serial.print("UID: ");

      for (byte i = 0; i < rfid.uid.size; i++) {

        Serial.print("0x");

        if (rfid.uid.uidByte[i] < 0x10)
          Serial.print("0");

        Serial.print(rfid.uid.uidByte[i], HEX);
        Serial.print(" ");
      }

      Serial.println();

      bool authorized = true;

      for (byte i = 0; i < 4; i++) {

        if (rfid.uid.uidByte[i] != authorizedUID[i]) {
          authorized = false;
          break;
        }
      }

      if (authorized) {

        Serial.println("ACCESS GRANTED");

        myServo.write(90);
        servoOpen     = true;
        servoOpenTime = millis();
      }
      else {
        Serial.println("ACCESS DENIED");
      }

      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }
  }

  delay(4);
}

// =====================================================================

void readMPU() {

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU, 14, true);

  if (Wire.available() == 14) {

    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();

    Wire.read();
    Wire.read();

    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();
  }
}

// =====================================================================

// ================= CH1 ISR =================
void ch1ISR() {

  uint32_t now = micros();

  if (PIND & (1 << PIND2))
    ch1Start = now;
  else
    ch1Value = now - ch1Start;
}

// ================= CH2 ISR =================
void ch2ISR() {

  uint32_t now = micros();

  if (PIND & (1 << PIND3))
    ch2Start = now;
  else
    ch2Value = now - ch2Start;
}

// ================= CH3 CH4 CH5 ISR =================
ISR(PCINT2_vect) {

  uint32_t now = micros();

  uint8_t currentPortD = PIND;
  uint8_t changed      = currentPortD ^ lastPortD;

  // ================= CH3 D4 =================
  if (changed & (1 << PIND4)) {

    if (currentPortD & (1 << PIND4))
      ch3Start = now;
    else
      ch3Value = now - ch3Start;
  }

  // ================= CH4 D5 =================
  if (changed & (1 << PIND5)) {

    if (currentPortD & (1 << PIND5))
      ch4Start = now;
    else
      ch4Value = now - ch4Start;
  }

  // ================= CH5 D6 =================
  if (changed & (1 << PIND6)) {

    if (currentPortD & (1 << PIND6))
      ch5Start = now;
    else
      ch5Value = now - ch5Start;
  }

  lastPortD = currentPortD;
}