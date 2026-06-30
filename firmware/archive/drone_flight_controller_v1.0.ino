/*
 * ============================================================
 *  F450 QUADCOPTER FLIGHT CONTROLLER
 *  Platform  : Arduino Uno / Nano (ATmega328P)
 *  Author    : Generated Flight Controller
 *  Version   : 1.0.0
 *
 *  PIN MAPPING
 *  ───────────────────────────────────────────────────────────
 *  MPU-6050   SDA → A4  |  SCL → A5
 *  RFID RC522 SS  → A1  |  SCK → D13 | MOSI → D11
 *                 MISO → D12 | RST → D8
 *  Servo (MG90S)      → D7
 *  ESC Front-Left     → D9   (Timer1 – hardware PWM)
 *  ESC Front-Right    → A2   (Software PWM)
 *  ESC Back-Right     → A3   (Software PWM)
 *  ESC Back-Left      → A0   (Software PWM)
 *  Receiver CH1 Roll  → D2   (interrupt-capable)
 *  Receiver CH2 Pitch → D3   (interrupt-capable)
 *  Receiver CH3 Throt → D4
 *  Receiver CH4 Yaw   → D5
 *  Receiver CH5 RFID  → D6
 *
 *  MOTOR LAYOUT  (top view)
 *        FL(D9) [CW]   FR(A2) [CCW]
 *              \       /
 *               [     ]
 *              /       \
 *        BL(A0) [CCW]  BR(A3) [CW]
 *
 *  LIBRARIES REQUIRED  (install via Library Manager)
 *   • Wire        (built-in)
 *   • Servo       (built-in)
 *   • MFRC522     by GithubCommunity
 * ============================================================
 */

// ──────────────────────────────────────────────────────────
//  INCLUDES
// ──────────────────────────────────────────────────────────
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <MFRC522.h>

// ──────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ──────────────────────────────────────────────────────────
// ESCs
#define PIN_ESC_FL    9     // Front-Left  – OC1A (hardware PWM 50 Hz)
#define PIN_ESC_FR    A2    // Front-Right
#define PIN_ESC_BR    A3    // Back-Right
#define PIN_ESC_BL    A0    // Back-Left

// Receiver channels (PWM-in)
#define PIN_RX_ROLL   2     // CH1 – hardware INT0
#define PIN_RX_PITCH  3     // CH2 – hardware INT1
#define PIN_RX_THROT  4     // CH3
#define PIN_RX_YAW    5     // CH4
#define PIN_RX_SW     6     // CH5 (RFID arm switch)

// MPU-6050 on I²C (A4/A5 used automatically by Wire)

// RFID RC522
#define PIN_RFID_SS   A1
#define PIN_RFID_RST  8

// Servo
#define PIN_SERVO     7

// ──────────────────────────────────────────────────────────
//  FLIGHT PARAMETERS  (tune for your F450)
// ──────────────────────────────────────────────────────────
// Throttle limits (µs)
#define THR_MIN       1000
#define THR_ARM       1050  // below this = disarmed
#define THR_MAX       1900  // safety ceiling

// PID gains – Roll
#define KP_ROLL       1.8f
#define KI_ROLL       0.04f
#define KD_ROLL       12.0f

// PID gains – Pitch
#define KP_PITCH      1.8f
#define KI_PITCH      0.04f
#define KD_PITCH      12.0f

// PID gains – Yaw
#define KP_YAW        2.5f
#define KI_YAW        0.06f
#define KD_YAW        0.0f

// Integral windup guard (µs)
#define I_LIMIT       400.0f

// Target loop period (µs)
#define LOOP_US       4000   // 250 Hz

// Complementary filter coefficient
#define CF_ALPHA      0.9965f  // ~0.9965 → τ ≈ 4 s at 250 Hz

// MPU-6050 I²C address
#define MPU_ADDR      0x68

// Gyro / Accel scale factors
// FS_SEL=0  → 131 LSB/°/s
// AFS_SEL=0 → 16384 LSB/g
#define GYRO_SCALE    131.0f
#define ACCEL_SCALE   16384.0f

// Dead-band on receiver sticks (µs)
#define STICK_DB      10

// ──────────────────────────────────────────────────────────
//  RFID – authorised UID
// ──────────────────────────────────────────────────────────
const byte AUTH_UID[4]  = { 0x9D, 0xA4, 0x5F, 0x06 };
#define SERVO_UNLOCK_DEG   90
#define SERVO_LOCK_DEG     0
#define SERVO_HOLD_MS      10000UL

// ──────────────────────────────────────────────────────────
//  OBJECTS
// ──────────────────────────────────────────────────────────
MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);
Servo   servoLock;
Servo   escFL, escFR, escBR, escBL;

// ──────────────────────────────────────────────────────────
//  RECEIVER  (pulse-width capture via polling + micros)
// ──────────────────────────────────────────────────────────
volatile uint16_t rxRoll   = 1500;
volatile uint16_t rxPitch  = 1500;
volatile uint16_t rxThrot  = 1000;
volatile uint16_t rxYaw    = 1500;
volatile uint16_t rxSwitch = 1000;

// Internal edge-timing (interrupts for CH1 & CH2)
volatile uint32_t risingRoll  = 0;
volatile uint32_t risingPitch = 0;

void isrRoll() {
    if (digitalRead(PIN_RX_ROLL)) risingRoll = micros();
    else { uint16_t w = (uint16_t)(micros() - risingRoll); if (w>800&&w<2200) rxRoll=w; }
}
void isrPitch() {
    if (digitalRead(PIN_RX_PITCH)) risingPitch = micros();
    else { uint16_t w = (uint16_t)(micros() - risingPitch); if (w>800&&w<2200) rxPitch=w; }
}

// Polled pulse-width read for remaining channels
uint16_t readPWM(uint8_t pin) {
    uint32_t t = pulseIn(pin, HIGH, 25000);
    if (t < 800 || t > 2200) return (pin == PIN_RX_THROT) ? 1000 : 1500;
    return (uint16_t)t;
}

// ──────────────────────────────────────────────────────────
//  MPU-6050 RAW DATA
// ──────────────────────────────────────────────────────────
struct RawIMU {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
};

// Calibration offsets (computed at boot)
int16_t gxOff = 0, gyOff = 0, gzOff = 0;
int16_t axOff = 0, ayOff = 0; // az bias left for gravity

// ──────────────────────────────────────────────────────────
//  ATTITUDE  (degrees)
// ──────────────────────────────────────────────────────────
float angleRoll  = 0.0f;
float anglePitch = 0.0f;
float angleYaw   = 0.0f;

// ──────────────────────────────────────────────────────────
//  PID STATE
// ──────────────────────────────────────────────────────────
float errRollPrev = 0, iRoll = 0;
float errPitchPrev= 0, iPitch= 0;
float errYawPrev  = 0, iYaw  = 0;

// ──────────────────────────────────────────────────────────
//  ESC OUTPUT  (µs)
// ──────────────────────────────────────────────────────────
uint16_t pwmFL = 1000, pwmFR = 1000, pwmBR = 1000, pwmBL = 1000;

// ──────────────────────────────────────────────────────────
//  ARMING
// ──────────────────────────────────────────────────────────
bool armed = false;

// ──────────────────────────────────────────────────────────
//  SERVO / RFID STATE
// ──────────────────────────────────────────────────────────
bool     servoUnlocked  = false;
uint32_t servoUnlockAt  = 0;

// ══════════════════════════════════════════════════════════
//  MPU-6050 HELPERS
// ══════════════════════════════════════════════════════════
void mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
}

bool mpuRead(RawIMU &d) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);  // ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(MPU_ADDR, (uint8_t)14);
    if (Wire.available() < 14) return false;
    d.ax = (Wire.read()<<8)|Wire.read();
    d.ay = (Wire.read()<<8)|Wire.read();
    d.az = (Wire.read()<<8)|Wire.read();
    Wire.read(); Wire.read();  // temp – discard
    d.gx = (Wire.read()<<8)|Wire.read();
    d.gy = (Wire.read()<<8)|Wire.read();
    d.gz = (Wire.read()<<8)|Wire.read();
    return true;
}

void mpuInit() {
    Wire.begin();
    Wire.setClock(400000L);   // 400 kHz fast-mode

    // Wake up, set clock source to gyro X
    mpuWrite(0x6B, 0x01);    // PWR_MGMT_1: clk=GYRO_X, no sleep
    delay(100);

    // DLPF: Bandwidth 42 Hz → smooth gyro, decent latency
    mpuWrite(0x1A, 0x03);    // CONFIG: DLPF_CFG=3

    // Gyro full-scale ±250 °/s (FS_SEL=0)
    mpuWrite(0x1B, 0x00);

    // Accel full-scale ±2 g (AFS_SEL=0)
    mpuWrite(0x1C, 0x00);

    // Sample-rate divider: 0 → 1 kHz / (1+0) = 1 kHz
    mpuWrite(0x19, 0x00);
}

void mpuCalibrate() {
    Serial.println(F("Calibrating IMU – keep drone level & still..."));
    const int N = 2000;
    long sgx=0, sgy=0, sgz=0, sax=0, say=0;
    RawIMU d;
    for (int i = 0; i < N; i++) {
        mpuRead(d);
        sgx += d.gx; sgy += d.gy; sgz += d.gz;
        sax += d.ax; say += d.ay;
        delay(2);
    }
    gxOff = sgx / N;
    gyOff = sgy / N;
    gzOff = sgz / N;
    axOff = sax / N;
    ayOff = say / N;
    Serial.println(F("IMU calibration done."));
    Serial.print(F("  Gyro offsets  gx=")); Serial.print(gxOff);
    Serial.print(F(" gy=")); Serial.print(gyOff);
    Serial.print(F(" gz=")); Serial.println(gzOff);
    Serial.print(F("  Accel offsets ax=")); Serial.print(axOff);
    Serial.print(F(" ay=")); Serial.println(ayOff);
}

// ══════════════════════════════════════════════════════════
//  ATTITUDE ESTIMATION  – Complementary Filter
//  Runs at LOOP_US period; dt supplied in seconds.
// ══════════════════════════════════════════════════════════
void updateAttitude(float dt) {
    RawIMU d;
    if (!mpuRead(d)) return;

    // Apply calibration offsets
    d.gx -= gxOff;
    d.gy -= gyOff;
    d.gz -= gzOff;
    d.ax -= axOff;
    d.ay -= ayOff;

    // Convert gyro to °/s
    float gRoll  =  (float)d.gy / GYRO_SCALE;  // rotation around X
    float gPitch = -(float)d.gx / GYRO_SCALE;  // rotation around Y
    float gYaw   =  (float)d.gz / GYRO_SCALE;  // rotation around Z

    // Accel angles (degrees), valid when near-hover
    float aRoll  = atan2f((float)d.ay, (float)d.az) * 57.2957795f;
    float aPitch = atan2f(-(float)d.ax,
                          sqrtf((float)d.ay*(float)d.ay +
                                (float)d.az*(float)d.az)) * 57.2957795f;

    // Complementary filter
    angleRoll  = CF_ALPHA * (angleRoll  + gRoll  * dt) + (1.0f - CF_ALPHA) * aRoll;
    anglePitch = CF_ALPHA * (anglePitch + gPitch * dt) + (1.0f - CF_ALPHA) * aPitch;
    angleYaw   += gYaw * dt;   // yaw: gyro-only (no magnetometer)
    if (angleYaw >  180.0f) angleYaw -= 360.0f;
    if (angleYaw < -180.0f) angleYaw += 360.0f;
}

// ══════════════════════════════════════════════════════════
//  PID CONTROLLER
// ══════════════════════════════════════════════════════════
float pidCalc(float setpoint, float measured, float kp, float ki, float kd,
              float &errPrev, float &integral, float dt) {
    float err   = setpoint - measured;
    integral   += err * dt;
    integral    = constrain(integral, -I_LIMIT, I_LIMIT);
    float deriv = (err - errPrev) / dt;
    errPrev     = err;
    return kp * err + ki * integral + kd * deriv;
}

// Reset PID state (called on disarm)
void resetPID() {
    errRollPrev = 0; iRoll  = 0;
    errPitchPrev= 0; iPitch = 0;
    errYawPrev  = 0; iYaw   = 0;
}

// ══════════════════════════════════════════════════════════
//  ESC  –  write microseconds to all four motors
// ══════════════════════════════════════════════════════════
void writeMotors(uint16_t fl, uint16_t fr, uint16_t br, uint16_t bl) {
    escFL.writeMicroseconds(fl);
    escFR.writeMicroseconds(fr);
    escBR.writeMicroseconds(br);
    escBL.writeMicroseconds(bl);
}

void motorsOff() {
    writeMotors(1000, 1000, 1000, 1000);
}

// ══════════════════════════════════════════════════════════
//  RFID CHECK
// ══════════════════════════════════════════════════════════
void handleRFID() {
    // Timeout: lock servo after hold time
    if (servoUnlocked && (millis() - servoUnlockAt >= SERVO_HOLD_MS)) {
        servoLock.write(SERVO_LOCK_DEG);
        servoUnlocked = false;
        Serial.println(F("RFID: servo locked (timeout)."));
    }

    if (!rfid.PICC_IsNewCardPresent()) return;
    if (!rfid.PICC_ReadCardSerial())   return;

    Serial.print(F("RFID UID: "));
    bool match = (rfid.uid.size == 4);
    for (byte i = 0; i < rfid.uid.size; i++) {
        if (rfid.uid.uidByte[i] != AUTH_UID[i]) match = false;
        Serial.print(rfid.uid.uidByte[i], HEX);
        Serial.print(' ');
    }
    Serial.println();

    if (match) {
        Serial.println(F("RFID: authorised – unlocking servo."));
        servoLock.write(SERVO_UNLOCK_DEG);
        servoUnlocked = true;
        servoUnlockAt = millis();
    } else {
        Serial.println(F("RFID: unauthorised card – servo unchanged."));
    }

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
}

// ══════════════════════════════════════════════════════════
//  STICK HELPER  – apply dead-band, return setpoint in °
// ══════════════════════════════════════════════════════════
float stickToAngle(uint16_t pw, float maxAngle) {
    int16_t c = (int16_t)pw - 1500;
    if (abs(c) < STICK_DB) c = 0;
    return ((float)c / 500.0f) * maxAngle;
}

float stickToRate(uint16_t pw, float maxRate) {
    int16_t c = (int16_t)pw - 1500;
    if (abs(c) < STICK_DB) c = 0;
    return ((float)c / 500.0f) * maxRate;
}

// ══════════════════════════════════════════════════════════
//  BOOT SEQUENCE
// ══════════════════════════════════════════════════════════
void bootSequence() {
    Serial.println(F("========================================"));
    Serial.println(F("  F450 FLIGHT CONTROLLER  v1.0.0"));
    Serial.println(F("========================================"));

    // ── Step 1: MPU-6050
    Serial.println(F("[1/5] Initialising MPU-6050..."));
    mpuInit();

    // Verify device ID
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x75); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1);
    uint8_t whoami = Wire.read();
    if (whoami != 0x68) {
        Serial.print(F("  ERROR: MPU WHO_AM_I = 0x")); Serial.println(whoami, HEX);
        Serial.println(F("  Check wiring. Halting."));
        while (1);
    }
    Serial.println(F("  MPU-6050 found (WHO_AM_I=0x68)."));

    // ── Step 2: Calibrate IMU
    Serial.println(F("[2/5] Calibrating IMU..."));
    mpuCalibrate();

    // ── Step 3: RFID RC522
    Serial.println(F("[3/5] Initialising RFID RC522..."));
    SPI.begin();
    rfid.PCD_Init();
    rfid.PCD_DumpVersionToSerial();
    Serial.println(F("  RC522 ready."));

    // ── Step 4: Servo
    Serial.println(F("[4/5] Initialising servo..."));
    servoLock.attach(PIN_SERVO);
    servoLock.write(SERVO_LOCK_DEG);
    delay(500);
    Serial.println(F("  Servo at 0° (locked)."));

    // ── Step 5: ESC arming sequence
    Serial.println(F("[5/5] Arming ESCs (sending 1000µs)..."));
    escFL.attach(PIN_ESC_FL, 1000, 2000);
    escFR.attach(PIN_ESC_FR, 1000, 2000);
    escBR.attach(PIN_ESC_BR, 1000, 2000);
    escBL.attach(PIN_ESC_BL, 1000, 2000);
    motorsOff();
    delay(3000);  // hold low throttle for ESC beep sequence
    Serial.println(F("  ESCs armed."));

    // ── Receiver interrupts
    attachInterrupt(digitalPinToInterrupt(PIN_RX_ROLL),  isrRoll,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_RX_PITCH), isrPitch, CHANGE);

    Serial.println(F("========================================"));
    Serial.println(F("  SYSTEM READY – waiting to arm."));
    Serial.println(F("  Arm: Throttle LOW + Yaw RIGHT (>1800µs)"));
    Serial.println(F("========================================"));
}

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    // Receiver input pins
    pinMode(PIN_RX_ROLL,  INPUT);
    pinMode(PIN_RX_PITCH, INPUT);
    pinMode(PIN_RX_THROT, INPUT);
    pinMode(PIN_RX_YAW,   INPUT);
    pinMode(PIN_RX_SW,    INPUT);

    // RFID SS pin
    pinMode(PIN_RFID_SS, OUTPUT);
    digitalWrite(PIN_RFID_SS, HIGH);

    bootSequence();
}

// ══════════════════════════════════════════════════════════
//  MAIN LOOP  –  250 Hz
// ══════════════════════════════════════════════════════════
void loop() {
    static uint32_t loopTimer = 0;
    uint32_t now = micros();

    // ── Pace the loop
    if ((uint32_t)(now - loopTimer) < LOOP_US) return;
    float dt = (float)(now - loopTimer) / 1000000.0f;
    loopTimer = now;

    // Guard against huge dt (e.g., first iteration or stall)
    if (dt > 0.02f) dt = 0.02f;

    // ── Read slow channels (polled, ~1.5 ms each but non-blocking here
    //    via rotating read to spread cost across loops)
    static uint8_t pollSlot = 0;
    switch (pollSlot++ % 3) {
        case 0: rxThrot  = readPWM(PIN_RX_THROT); break;
        case 1: rxYaw    = readPWM(PIN_RX_YAW);   break;
        case 2: rxSwitch = readPWM(PIN_RX_SW);    break;
    }

    // ── Attitude
    updateAttitude(dt);

    // ── Arming logic
    //    Arm:   Throttle < ARM, Yaw stick full RIGHT (>1800)
    //    Disarm: Throttle < ARM, Yaw stick full LEFT (<1200)
    if (!armed) {
        if (rxThrot < THR_ARM && rxYaw > 1800) {
            armed = true;
            resetPID();
            Serial.println(F(">>> ARMED <<<"));
        }
    } else {
        if (rxThrot < THR_ARM && rxYaw < 1200) {
            armed = false;
            motorsOff();
            resetPID();
            Serial.println(F(">>> DISARMED <<<"));
        }
    }

    // ── RFID / servo (run regardless of arm state)
    handleRFID();

    // ── Flight
    if (!armed) return;

    // Throttle from receiver
    uint16_t thr = constrain(rxThrot, THR_MIN, THR_MAX);

    // Setpoints from sticks
    float spRoll  = stickToAngle(rxRoll,  25.0f);  // ±25° max bank
    float spPitch = stickToAngle(rxPitch, 25.0f);  // ±25° max pitch
    float spYaw   = stickToRate (rxYaw,   90.0f);  // ±90°/s yaw rate

    // PID outputs
    float outRoll  = pidCalc(spRoll,  angleRoll,  KP_ROLL,  KI_ROLL,  KD_ROLL,
                             errRollPrev,  iRoll,  dt);
    float outPitch = pidCalc(spPitch, anglePitch, KP_PITCH, KI_PITCH, KD_PITCH,
                             errPitchPrev, iPitch, dt);
    // Yaw: rate control (setpoint=desired rate, measured=current gyro rate)
    // Re-read gz for yaw rate (already stored in attitude update – re-read here)
    // For simplicity, use filtered yaw angle rate from CF differentiation
    static float yawPrev = 0;
    float yawRate = (angleYaw - yawPrev) / dt;
    yawPrev = angleYaw;
    float outYaw = pidCalc(spYaw, yawRate, KP_YAW, KI_YAW, KD_YAW,
                           errYawPrev, iYaw, dt);

    /*
     *  Motor mixing  –  F450 X-configuration
     *
     *            FL(CW)   FR(CCW)
     *              ↑  ↓Roll  ↑
     *   Pitch↑ →  [  quad  ]  ← Pitch↓
     *              ↓         ↓
     *            BL(CCW)  BR(CW)
     *
     *  Roll  right → FL↑, BL↑, FR↓, BR↓
     *  Pitch fwd   → FL↑, FR↑, BL↓, BR↓
     *  Yaw   CW    → FL↑, BR↑, FR↓, BL↓  (reaction torque)
     */
    float fl = thr + outRoll + outPitch - outYaw;
    float fr = thr - outRoll + outPitch + outYaw;
    float br = thr - outRoll - outPitch - outYaw;
    float bl = thr + outRoll - outPitch + outYaw;

    // Clamp to valid ESC range
    pwmFL = (uint16_t)constrain(fl, 1100, THR_MAX);
    pwmFR = (uint16_t)constrain(fr, 1100, THR_MAX);
    pwmBR = (uint16_t)constrain(br, 1100, THR_MAX);
    pwmBL = (uint16_t)constrain(bl, 1100, THR_MAX);

    writeMotors(pwmFL, pwmFR, pwmBR, pwmBL);

    // ── Serial telemetry (every 100 ms) – comment out in flight for speed
    static uint32_t telTimer = 0;
    if (millis() - telTimer > 100) {
        telTimer = millis();
        Serial.print(F("R:")); Serial.print(angleRoll,1);
        Serial.print(F(" P:")); Serial.print(anglePitch,1);
        Serial.print(F(" Y:")); Serial.print(angleYaw,1);
        Serial.print(F(" | FL:")); Serial.print(pwmFL);
        Serial.print(F(" FR:")); Serial.print(pwmFR);
        Serial.print(F(" BR:")); Serial.print(pwmBR);
        Serial.print(F(" BL:")); Serial.print(pwmBL);
        Serial.print(F(" | Thr:")); Serial.println(rxThrot);
    }
}
/* ─── END OF FILE ─────────────────────────────────────── */
