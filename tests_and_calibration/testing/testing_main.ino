/*
 * ============================================================
 *  F450 QUADCOPTER FLIGHT CONTROLLER
 *  Platform : Arduino Uno / Nano (ATmega328P)
 *  Version  : 2.0.0
 *
 *  PIN MAPPING
 *  ─────────────────────────────────────────────────────────
 *  MPU-6050          SDA → A4  |  SCL → A5
 *  RFID RC522        SS  → A1  |  SCK → D13
 *                    MOSI→ D11 |  MISO→ D12 | RST → D8
 *  Servo (MG90S)         → D7
 *  ESC Front-Left        → D9   (hardware PWM Timer1)
 *  ESC Front-Right       → D10  (hardware PWM Timer1)
 *  ESC Back-Right        → D5
 *  ESC Back-Left         → D6
 *  Receiver CH1 Roll     → D2   (INT0 – hardware interrupt)
 *  Receiver CH2 Pitch    → D3   (INT1 – hardware interrupt)
 *  Receiver CH3 Throttle → D4   (polled)
 *  Receiver CH4 Yaw      → A2   (polled)
 *  Receiver CH5 Switch   → A3   (polled)
 *
 *  MOTOR LAYOUT (top view, X-config)
 *        FL(D9)[CW]    FR(D10)[CCW]
 *              \         /
 *               [  F450  ]
 *              /         \
 *        BL(D6)[CCW]   BR(D5)[CW]
 *
 *  LIBRARIES REQUIRED (Library Manager)
 *    Wire    – built-in
 *    Servo   – built-in
 *    SPI     – built-in
 *    MFRC522 – by GithubCommunity
 * ============================================================
 */

#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <MFRC522.h>

// ──────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ──────────────────────────────────────────────────────────
// ESCs
#define PIN_ESC_FL    A0     // Front-Left  – hardware PWM (Timer1)
#define PIN_ESC_FR    A3   // Front-Right – software PWM
#define PIN_ESC_BR    A2    // Back-Right  – software PWM
#define PIN_ESC_BL    9  // Back-Left   – software PWM

// Receiver channels
#define PIN_RX_ROLL   2     // CH1 – INT0
#define PIN_RX_PITCH  3     // CH2 – INT1
#define PIN_RX_THROT  4     // CH3
#define PIN_RX_YAW    5     // CH4
#define PIN_RX_SW     6     // CH5

#define PIN_RFID_SS   A1
#define PIN_RFID_RST  8
#define PIN_SERVO     7

// ──────────────────────────────────────────────────────────
//  FLIGHT PARAMETERS
// ──────────────────────────────────────────────────────────
#define THR_MIN         1000
#define THR_ARM         1050    // below → disarmed
#define THR_MAX         1900    // safety ceiling

#define MAX_BANK_DEG    25.0f   // max roll / pitch angle (°)
#define MAX_YAW_RATE    90.0f   // max yaw rate (°/s)

// Loop target: 250 Hz
#define LOOP_US         4000

// MPU-6050
#define MPU_ADDR        0x68
#define GYRO_SCALE      131.0f   // LSB / (°/s)  at ±250 °/s

// ──────────────────────────────────────────────────────────
//  PID GAINS  (tuned from Doc 1 values, scaled for full build)
// ──────────────────────────────────────────────────────────
//  Roll & Pitch
#define KP_RP   1.4f
#define KI_RP   0.02f
#define KD_RP   3.5f

//  Yaw (rate control)
#define KP_YAW  2.5f
#define KI_YAW  0.06f
#define KD_YAW  0.0f

// Integral windup limit (µs equivalent)
#define I_LIMIT 400.0f

// Dead-band on sticks (µs each side of 1500)
#define STICK_DB 10

// ──────────────────────────────────────────────────────────
//  COMPLEMENTARY FILTER
//  alpha = 0.98 → gyro trust; (1-alpha) = 0.02 → accel trust
//  Matches Doc 1 tuning exactly.
// ──────────────────────────────────────────────────────────
#define CF_ALPHA  0.98f

// ──────────────────────────────────────────────────────────
//  LOW-PASS FILTER ON ACCEL ANGLE (Doc 1 style)
// ──────────────────────────────────────────────────────────
#define LP_ACC    0.90f   // 90 % old / 10 % new

// ──────────────────────────────────────────────────────────
//  LOW-PASS FILTER ON DERIVATIVE (Doc 1 style)
// ──────────────────────────────────────────────────────────
#define LP_DERIV  0.70f   // 70 % old / 30 % new

// ──────────────────────────────────────────────────────────
//  RFID
// ──────────────────────────────────────────────────────────
const byte AUTH_UID[4]   = { 0x9D, 0xA4, 0x5F, 0x06 };
#define SERVO_LOCK_DEG    0
#define SERVO_UNLOCK_DEG  90
#define SERVO_HOLD_MS     10000UL

// ──────────────────────────────────────────────────────────
//  OBJECTS
// ──────────────────────────────────────────────────────────
MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);
Servo   servoLock;
Servo   escFL, escFR, escBR, escBL;

// ══════════════════════════════════════════════════════════
//  RECEIVER  (interrupt + polled)
// ══════════════════════════════════════════════════════════
// ══════════════════════════════════════════════════════════
//  RECEIVER  (ALL HARDWARE INTERRUPTS NOW)
// ══════════════════════════════════════════════════════════
volatile uint16_t rxRoll   = 1500;
volatile uint16_t rxPitch  = 1500;
volatile uint16_t rxThrot  = 1000;
volatile uint16_t rxYaw    = 1500;
volatile uint16_t rxSwitch = 1000;

volatile uint32_t riseRoll   = 0;
volatile uint32_t risePitch  = 0;
volatile uint32_t riseThrot  = 0;
volatile uint32_t riseYaw    = 0;
volatile uint32_t riseSwitch = 0;

uint8_t lastPortD = 0; // Tracks the previous state of pins D0-D7

void isrRoll() {
    if (digitalRead(PIN_RX_ROLL)) riseRoll = micros();
    else {
        uint16_t w = (uint16_t)(micros() - riseRoll);
        if (w > 800 && w < 2200) rxRoll = w;
    }
}

void isrPitch() {
    if (digitalRead(PIN_RX_PITCH)) risePitch = micros();
    else {
        uint16_t w = (uint16_t)(micros() - risePitch);
        if (w > 800 && w < 2200) rxPitch = w;
    }
}

// New Pin Change Interrupt for D4 (Throt), D5 (Yaw), and D6 (Switch)
ISR(PCINT2_vect) {
    uint32_t now = micros();
    uint8_t currentPortD = PIND; // Read state of all pins on Port D
    uint8_t changed = currentPortD ^ lastPortD; // Find which pins changed
    lastPortD = currentPortD;

    // Check D4 (Throttle)
    if (changed & (1 << 4)) {
        if (currentPortD & (1 << 4)) riseThrot = now;
        else {
            uint16_t w = (uint16_t)(now - riseThrot);
            if (w > 800 && w < 2200) rxThrot = w;
        }
    }
    // Check D5 (Yaw)
    if (changed & (1 << 5)) {
        if (currentPortD & (1 << 5)) riseYaw = now;
        else {
            uint16_t w = (uint16_t)(now - riseYaw);
            if (w > 800 && w < 2200) rxYaw = w;
        }
    }
    // Check D6 (Switch)
    if (changed & (1 << 6)) {
        if (currentPortD & (1 << 6)) riseSwitch = now;
        else {
            uint16_t w = (uint16_t)(now - riseSwitch);
            if (w > 800 && w < 2200) rxSwitch = w;
        }
    }
}
uint16_t readPWM(uint8_t pin) {
    uint32_t t = pulseIn(pin, HIGH, 25000);
    
    // ── TROUBLESHOOTING LOGIC ──
    if (t < 800 || t > 2200) {
        if (pin == PIN_RX_THROT) {
            Serial.print(F("!!! ERROR: Throttle Signal Lost / Timeout !!! Raw pulse duration: "));
            Serial.println(t);
            return 1000;
        } else if (pin == PIN_RX_YAW) {
            Serial.println(F("!!! ERROR: Yaw Signal Lost / Timeout !!!"));
            return 1500;
        } else if (pin == PIN_RX_SW) {
            Serial.println(F("!!! ERROR: Switch Signal Lost / Timeout !!!"));
            return 1500;
        }
    }
    return (uint16_t)t;
}

// ══════════════════════════════════════════════════════════
//  IMU RAW + CALIBRATION OFFSETS
// ══════════════════════════════════════════════════════════
int16_t rawAcX, rawAcY, rawAcZ;
int16_t rawGyX, rawGyY, rawGyZ;

float gyroXoff = 0, gyroYoff = 0, gyroZoff = 0;
float pitchOffset = 0, rollOffset = 0;

// ══════════════════════════════════════════════════════════
//  ATTITUDE  (degrees)
// ══════════════════════════════════════════════════════════
float pitch = 0, roll = 0, yaw = 0;
float accPitchFilt = 0, accRollFilt = 0;

// ══════════════════════════════════════════════════════════
//  PID STATE
// ══════════════════════════════════════════════════════════
float pitchErr = 0,  rollErr = 0,  yawErr = 0;
float pitchInt = 0,  rollInt = 0,  yawInt = 0;
float pitchPrev= 0,  rollPrev= 0,  yawPrev= 0;
float pitchDFilt=0,  rollDFilt=0,  yawDFilt=0;
float pitchPID = 0,  rollPID = 0,  yawPID = 0;

// ══════════════════════════════════════════════════════════
//  ESC OUTPUT  (µs)
// ══════════════════════════════════════════════════════════
uint16_t pwmFL = 1000, pwmFR = 1000, pwmBR = 1000, pwmBL = 1000;

// ══════════════════════════════════════════════════════════
//  ARM / RFID STATE
// ══════════════════════════════════════════════════════════
bool armed          = false;
bool servoUnlocked  = false;
uint32_t servoAt    = 0;

// ══════════════════════════════════════════════════════════
//  MPU-6050 HELPERS
// ══════════════════════════════════════════════════════════
void mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void readMPU() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    if (Wire.available() == 14) {
        rawAcX = Wire.read() << 8 | Wire.read();
        rawAcY = Wire.read() << 8 | Wire.read();
        rawAcZ = Wire.read() << 8 | Wire.read();
        Wire.read(); Wire.read();            // skip temperature
        rawGyX = Wire.read() << 8 | Wire.read();
        rawGyY = Wire.read() << 8 | Wire.read();
        rawGyZ = Wire.read() << 8 | Wire.read();
    }
}

void mpuInit() {
    Wire.begin();
    Wire.setClock(400000L);
    mpuWrite(0x6B, 0x01);   // wake, use gyro X as clock source
    delay(100);
    mpuWrite(0x1A, 0x03);   // DLPF 42 Hz – smooths gyro nicely
    mpuWrite(0x1B, 0x00);   // gyro  ±250 °/s
    mpuWrite(0x1C, 0x00);   // accel ±2 g
    mpuWrite(0x19, 0x00);   // sample rate divider = 0 → 1 kHz
}

// ══════════════════════════════════════════════════════════
//  CALIBRATION
// ══════════════════════════════════════════════════════════
void calibrate() {

    // ── STEP 1: Gyro bias (3000 samples × 2 ms = 6 s)
    Serial.println(F("KEEP DRONE PERFECTLY STILL..."));
    delay(2000);

    const int GYRO_N = 3000;
    long gxSum = 0, gySum = 0, gzSum = 0;
    for (int i = 0; i < GYRO_N; i++) {
        readMPU();
        gxSum += rawGyX;
        gySum += rawGyY;
        gzSum += rawGyZ;
        delay(2);
    }
    gyroXoff = (float)gxSum / GYRO_N;
    gyroYoff = (float)gySum / GYRO_N;
    gyroZoff = (float)gzSum / GYRO_N;

    Serial.print(F("Gyro offsets: X="));
    Serial.print(gyroXoff, 2);
    Serial.print(F("  Y="));
    Serial.print(gyroYoff, 2);
    Serial.print(F("  Z="));
    Serial.println(gyroZoff, 2);

    // ── STEP 2: Accelerometer angle offsets (1000 samples)
    float pSum = 0, rSum = 0;
    const int ACC_N = 1000;
    for (int i = 0; i < ACC_N; i++) {
        readMPU();
        float ax = (float)rawAcX;
        float ay = (float)rawAcY;
        float az = (float)rawAcZ;

        pSum += atan2(ax, sqrt(ay*ay + az*az)) * 57.2958f;
        rSum += atan2(ay, sqrt(ax*ax + az*az)) * 57.2958f;
        delay(2);
    }
    pitchOffset = pSum / ACC_N;
    rollOffset  = rSum / ACC_N;

    Serial.print(F("Angle offsets: pitch="));
    Serial.print(pitchOffset, 3);
    Serial.print(F("  roll="));
    Serial.println(rollOffset, 3);
    Serial.println(F("CALIBRATION DONE"));
}

// ══════════════════════════════════════════════════════════
//  ATTITUDE UPDATE  (complementary filter – Doc 1 approach)
// ══════════════════════════════════════════════════════════
void updateAttitude(float dt) {
    readMPU();

    // Calibrated gyro rates (°/s)
    float gyroX = (rawGyX - gyroXoff) / GYRO_SCALE;
    float gyroY = (rawGyY - gyroYoff) / GYRO_SCALE;
    float gyroZ = (rawGyZ - gyroZoff) / GYRO_SCALE;

    // Raw accel values as float
    float ax = (float)rawAcX;
    float ay = (float)rawAcY;
    float az = (float)rawAcZ;

    // Accelerometer angles (degrees) – subtract mount offsets
    float accPitch = atan2(ax, sqrt(ay*ay + az*az)) * 57.2958f - pitchOffset;
    float accRoll  = atan2(ay, sqrt(ax*ax + az*az)) * 57.2958f - rollOffset;

    // Low-pass filter on accel angle (reduces vibration noise)
    accPitchFilt = LP_ACC * accPitchFilt + (1.0f - LP_ACC) * accPitch;
    accRollFilt  = LP_ACC * accRollFilt  + (1.0f - LP_ACC) * accRoll;

    // Complementary filter: blend gyro integration with accel
    pitch = CF_ALPHA * (pitch + gyroY * dt) + (1.0f - CF_ALPHA) * accPitchFilt;
    roll  = CF_ALPHA * (roll  + gyroX * dt) + (1.0f - CF_ALPHA) * accRollFilt;

    // Safety fallback: if angle exceeds physical limit, trust accel only
    if (pitch >  90.0f || pitch < -90.0f) pitch = accPitchFilt;
    if (roll  >  90.0f || roll  < -90.0f) roll  = accRollFilt;

    // Yaw: gyro-only integration (no magnetometer)
    yaw += gyroZ * dt;
    if (yaw >  180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;
}

// ══════════════════════════════════════════════════════════
//  PID CALCULATOR  (with derivative low-pass – Doc 1 style)
// ══════════════════════════════════════════════════════════
float computePID(float setpoint, float measured,
                 float kp, float ki, float kd,
                 float &errPrev, float &integral, float &derivFilt,
                 float dt) {

    float err = setpoint - measured;

    // Integral with windup guard
    integral += err * dt;
    integral  = constrain(integral, -I_LIMIT, I_LIMIT);

    // Derivative with low-pass filter
    float deriv = (err - errPrev) / dt;
    derivFilt   = LP_DERIV * derivFilt + (1.0f - LP_DERIV) * deriv;

    errPrev = err;

    return kp * err + ki * integral + kd * derivFilt;
}

void resetPID() {
    pitchInt  = 0; rollInt  = 0; yawInt  = 0;
    pitchPrev = 0; rollPrev = 0; yawPrev = 0;
    pitchDFilt= 0; rollDFilt= 0; yawDFilt= 0;
}

// ══════════════════════════════════════════════════════════
//  MOTOR OUTPUT
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
//  RFID / SERVO
// ══════════════════════════════════════════════════════════
void handleRFID() {
    // Auto-lock after hold time
    if (servoUnlocked && (millis() - servoAt >= SERVO_HOLD_MS)) {
        servoLock.write(SERVO_LOCK_DEG);
        servoUnlocked = false;
        Serial.println(F("RFID: locked (timeout)"));
    }

    if (!rfid.PICC_IsNewCardPresent()) return;
    if (!rfid.PICC_ReadCardSerial())   return;

    bool match = (rfid.uid.size == 4);
    Serial.print(F("RFID UID: "));
    for (byte i = 0; i < rfid.uid.size; i++) {
        if (rfid.uid.uidByte[i] != AUTH_UID[i]) match = false;
        Serial.print(rfid.uid.uidByte[i], HEX);
        Serial.print(' ');
    }
    Serial.println();

    if (match) {
        Serial.println(F("RFID: AUTHORISED – servo unlocked"));
        servoLock.write(SERVO_UNLOCK_DEG);
        servoUnlocked = true;
        servoAt       = millis();
    } else {
        Serial.println(F("RFID: DENIED"));
    }

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
}

// ══════════════════════════════════════════════════════════
//  STICK HELPERS
// ══════════════════════════════════════════════════════════
float stickToAngle(uint16_t pw, float maxDeg) {
    int16_t c = (int16_t)pw - 1500;
    if (abs(c) < STICK_DB) c = 0;
    return ((float)c / 500.0f) * maxDeg;
}

float stickToRate(uint16_t pw, float maxRate) {
    int16_t c = (int16_t)pw - 1500;
    if (abs(c) < STICK_DB) c = 0;
    return ((float)c / 500.0f) * maxRate;
}

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    Serial.println(F("============================================"));
    Serial.println(F("   F450 FLIGHT CONTROLLER  v2.0.0"));
    Serial.println(F("============================================"));

    // Receiver input pins
    pinMode(PIN_RX_ROLL,  INPUT);
    pinMode(PIN_RX_PITCH, INPUT);
    pinMode(PIN_RX_THROT, INPUT);
    pinMode(PIN_RX_YAW,   INPUT);
    pinMode(PIN_RX_SW,    INPUT);

    // RFID SS
    pinMode(PIN_RFID_SS, OUTPUT);
    digitalWrite(PIN_RFID_SS, HIGH);

    // ── MPU-6050
    Serial.println(F("[1/5] Initialising MPU-6050..."));
    mpuInit();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x75);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
    uint8_t whoami = Wire.read();
  Serial.print(F("  WHO_AM_I = 0x"));
Serial.println(whoami, HEX);

if (whoami == 0x00 || whoami == 0xFF) {
    // 0x00 or 0xFF means no I2C response at all — real wiring problem
    Serial.println(F("  ERROR: No response on I2C. Check wiring. HALTED."));
    while (1);
}
Serial.println(F("  MPU-6050 (or compatible) found."));
    Serial.println(F("  MPU-6050 OK."));

    // ── IMU calibration
    Serial.println(F("[2/5] Calibrating IMU..."));
    calibrate();

    // ── RFID
    Serial.println(F("[3/5] Initialising RFID RC522..."));
    SPI.begin();
    rfid.PCD_Init();
    rfid.PCD_DumpVersionToSerial();
    Serial.println(F("  RC522 ready."));

    // ── Servo
    Serial.println(F("[4/5] Initialising servo..."));
    servoLock.attach(PIN_SERVO);
    servoLock.write(SERVO_LOCK_DEG);
    delay(500);
    Serial.println(F("  Servo at 0° (locked)."));

    // ── ESC arming
    Serial.println(F("[5/5] Arming ESCs (3 s at 1000 µs)..."));
    escFL.attach(PIN_ESC_FL, 1000, 2000);
    escFR.attach(PIN_ESC_FR, 1000, 2000);
    escBR.attach(PIN_ESC_BR, 1000, 2000);
    escBL.attach(PIN_ESC_BL, 1000, 2000);
    motorsOff();
    delay(3000);
    Serial.println(F("  ESCs armed."));

// ── Interrupts
    attachInterrupt(digitalPinToInterrupt(PIN_RX_ROLL),  isrRoll,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_RX_PITCH), isrPitch, CHANGE);

    // Enable Pin Change Interrupts for Throttle, Yaw, and Switch
    PCICR  |= (1 << PCIE2);    // Enable PCINT group 2 (Port D)
    PCMSK2 |= (1 << PCINT20);  // Unmask D4
    PCMSK2 |= (1 << PCINT21);  // Unmask D5
    PCMSK2 |= (1 << PCINT22);  // Unmask D6

    Serial.println(F("============================================"));
    Serial.println(F("  SYSTEM READY"));
    Serial.println(F("  ARM  : Throttle LOW + Yaw RIGHT (>1800µs)"));
    Serial.println(F("  DISARM: Throttle LOW + Yaw LEFT (<1200µs)"));
    Serial.println(F("============================================"));
}

// ══════════════════════════════════════════════════════════
//  MAIN LOOP  –  250 Hz
// ══════════════════════════════════════════════════════════
void loop() {
    static uint32_t loopTimer = 0;
    uint32_t now = micros();

    if ((uint32_t)(now - loopTimer) < LOOP_US) return;

    float dt = (float)(now - loopTimer) / 1000000.0f;
    loopTimer = now;

    // Guard against stale dt (startup or stall)
    if (dt <= 0.0f || dt > 0.1f) return;

    // Receiver switch directly drives the servo position.
    if (rxSwitch < 1600) {
        servoLock.write(SERVO_LOCK_DEG);
        servoUnlocked = false;
    } else {
        servoLock.write(SERVO_UNLOCK_DEG);
        servoUnlocked = true;
        servoAt = millis();
    }

    // ── Attitude
    updateAttitude(dt);

    // ── Arm / disarm
    //    ARM   : Throttle < THR_ARM  AND  Yaw > 1800 µs
    //    DISARM: Throttle < THR_ARM  AND  Yaw < 1200 µs
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

    // ── RFID (always active, regardless of arm state)
    handleRFID();

    if (!armed) return;

    // ── Throttle
    uint16_t thr = constrain(rxThrot, THR_MIN, THR_MAX);

    // ── Stick setpoints
    float spRoll  = stickToAngle(rxRoll,  MAX_BANK_DEG);
    float spPitch = stickToAngle(rxPitch, MAX_BANK_DEG);
    float spYaw   = stickToRate (rxYaw,   MAX_YAW_RATE);

    // ── PID: Roll
    rollPID = computePID(spRoll, roll,
                         KP_RP, KI_RP, KD_RP,
                         rollPrev, rollInt, rollDFilt, dt);

    // ── PID: Pitch
    pitchPID = computePID(spPitch, pitch,
                          KP_RP, KI_RP, KD_RP,
                          pitchPrev, pitchInt, pitchDFilt, dt);

    // ── PID: Yaw rate  (gyro-derived rate vs desired rate)
    static float yawAnglePrev = 0;
    float yawRate = (yaw - yawAnglePrev) / dt;
    yawAnglePrev  = yaw;
    yawPID = computePID(spYaw, yawRate,
                        KP_YAW, KI_YAW, KD_YAW,
                        yawPrev, yawInt, yawDFilt, dt);

    // ──────────────────────────────────────────────────────
    //  MOTOR MIXING  –  F450 X-configuration
    //
    //   Roll  right → FL↑  BL↑  FR↓  BR↓
    //   Pitch fwd   → FL↑  FR↑  BL↓  BR↓
    //   Yaw   CW    → FL↑  BR↑  FR↓  BL↓  (reaction torque)
    //
    //   FL(CW)  = thr + roll + pitch - yaw
    //   FR(CCW) = thr - roll + pitch + yaw
    //   BR(CW)  = thr - roll - pitch - yaw
    //   BL(CCW) = thr + roll - pitch + yaw
    // ──────────────────────────────────────────────────────
    float fl = thr + rollPID + pitchPID - yawPID;
    float fr = thr - rollPID + pitchPID + yawPID;
    float br = thr - rollPID - pitchPID - yawPID;
    float bl = thr + rollPID - pitchPID + yawPID;

    // Clamp to valid ESC range (1100 µs minimum keeps props spinning)
    pwmFL = (uint16_t)constrain(fl, 1100, THR_MAX);
    pwmFR = (uint16_t)constrain(fr, 1100, THR_MAX);
    pwmBR = (uint16_t)constrain(br, 1100, THR_MAX);
    pwmBL = (uint16_t)constrain(bl, 1100, THR_MAX);

    writeMotors(pwmFL, pwmFR, pwmBR, pwmBL);

    // ── Serial telemetry (every 100 ms – comment out for final flight)
    static uint32_t telTimer = 0;
    if (millis() - telTimer > 100) {
        telTimer = millis();
        Serial.print(F("Pitch:"));   Serial.print(pitch,   1);
        Serial.print(F(" Roll:"));   Serial.print(roll,    1);
        Serial.print(F(" Yaw:"));    Serial.print(yaw,     1);
        Serial.print(F(" | PitchPID:")); Serial.print(pitchPID, 1);
        Serial.print(F(" RollPID:")); Serial.print(rollPID,  1);
        Serial.print(F(" YawPID:")); Serial.print(yawPID,   1);
        Serial.print(F(" | FL:"));   Serial.print(pwmFL);
        Serial.print(F(" FR:"));     Serial.print(pwmFR);
        Serial.print(F(" BR:"));     Serial.print(pwmBR);
        Serial.print(F(" BL:"));     Serial.print(pwmBL);
        Serial.print(F(" Thr:"));    Serial.println(rxThrot);
        Serial.print(F(" Switch:")); Serial.println(rxSwitch);
    }
}

/* ─── END OF FILE ──────────────────────────────────────── */
