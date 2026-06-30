/*
 * ============================================================
 *  F450 QUADCOPTER FLIGHT CONTROLLER
 *  Platform : Arduino Uno / Nano (ATmega328P)
 *  Version  : 2.3.0
 *
 *  Changes from v2.2.0:
 *   - Passive buzzer on A0 with distinct beep patterns
 *     for every system event
 * ============================================================
 */

#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <MFRC522.h>

// ──────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ──────────────────────────────────────────────────────────
#define PIN_ESC_FL    9
#define PIN_ESC_FR    10
#define PIN_ESC_BR    5
#define PIN_ESC_BL    6

#define PIN_RX_ROLL   2
#define PIN_RX_PITCH  3
#define PIN_RX_THROT  4
#define PIN_RX_YAW    A3
#define PIN_RX_SW     A2

#define PIN_RFID_SS   A1
#define PIN_RFID_RST  8
#define PIN_SERVO     7
#define PIN_BUZZER    A0

// ──────────────────────────────────────────────────────────
//  FLIGHT PARAMETERS
// ──────────────────────────────────────────────────────────
#define THR_MIN         1000
#define THR_ARM         1050
#define THR_MAX         1900
#define THR_MOTOR_MIN   1100

#define MAX_BANK_DEG    25.0f
#define MAX_YAW_RATE    90.0f
#define LOOP_US         4000
#define MPU_ADDR        0x68
#define INV_GYRO_SCALE  0.007633588f
#define INV_STICK       0.002f

// ──────────────────────────────────────────────────────────
//  PID GAINS
// ──────────────────────────────────────────────────────────
#define KP_RP   1.4f
#define KI_RP   0.02f
#define KD_RP   3.5f
#define KP_YAW  2.5f
#define KI_YAW  0.06f
#define KD_YAW  0.0f
#define I_LIMIT 400.0f
#define STICK_DB 10

// ──────────────────────────────────────────────────────────
//  FILTER CONSTANTS
// ──────────────────────────────────────────────────────────
#define CF_ALPHA        0.98f
#define CF_ALPHA_INV    0.02f
#define LP_ACC          0.90f
#define LP_ACC_INV      0.10f
#define LP_DERIV        0.70f
#define LP_DERIV_INV    0.30f

// ──────────────────────────────────────────────────────────
//  RFID / SERVO
// ──────────────────────────────────────────────────────────
const byte AUTH_UID[4]   = { 0x0A, 0x77, 0xF7, 0x05 };
#define SERVO_LOCK_DEG    0
#define SERVO_UNLOCK_DEG  90
#define SERVO_HOLD_MS     10000UL

struct RxSnapshot { uint16_t roll, pitch, throt, yaw, sw; };  
//  BUZZER
// ──────────────────────────────────────────────────────────
#define BUZZ_FREQ   2500

void beep(uint16_t freq, uint16_t durationMs) {
    uint32_t period = 1000000UL / freq;
    uint32_t halfP  = period / 2;
    uint32_t cycles = (uint32_t)durationMs * 1000UL / period;
    for (uint32_t i = 0; i < cycles; i++) {
        digitalWrite(PIN_BUZZER, HIGH);
        delayMicroseconds(halfP);
        digitalWrite(PIN_BUZZER, LOW);
        delayMicroseconds(halfP);
    }
}

void beepShort()  { beep(BUZZ_FREQ, 80);  }
void beepLong()   { beep(BUZZ_FREQ, 400); }
void beepGap()    { delay(100); }

void beepPattern(uint8_t count, bool longBeep = false) {
    for (uint8_t i = 0; i < count; i++) {
        if (longBeep) beepLong(); else beepShort();
        if (i < count - 1) beepGap();
    }
}

void beepAuthorised() {     // ascending — RFID granted
    beep(2000, 80); delay(80);
    beep(2500, 80); delay(80);
    beep(3000, 80);
}

void beepDenied() {         // low double — RFID rejected
    beep(800, 150); delay(100);
    beep(800, 150);
}

void beepError() {          // rapid loop — fatal halt
    while (1) {
        beep(1500, 50);
        delay(50);
    }
}

// ──────────────────────────────────────────────────────────
//  OBJECTS
// ──────────────────────────────────────────────────────────
MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);
Servo   servoLock;
Servo   escFL, escFR, escBR, escBL;

// ══════════════════════════════════════════════════════════
//  RECEIVER
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


inline RxSnapshot readRx() {
    RxSnapshot s;
    uint8_t sreg = SREG;
    cli();
    s.roll  = rxRoll;
    s.pitch = rxPitch;
    s.throt = rxThrot;
    s.yaw   = rxYaw;
    s.sw    = rxSwitch;
    SREG = sreg;
    return s;
}

void isrRoll() {
    uint32_t now = micros();
    if (PIND & (1 << 2)) { riseRoll = now; }
    else {
        uint16_t w = (uint16_t)(now - riseRoll);
        if (w > 800 && w < 2200) rxRoll = w;
    }
}

void isrPitch() {
    uint32_t now = micros();
    if (PIND & (1 << 3)) { risePitch = now; }
    else {
        uint16_t w = (uint16_t)(now - risePitch);
        if (w > 800 && w < 2200) rxPitch = w;
    }
}

ISR(PCINT2_vect) {
    uint32_t now = micros();
    uint8_t cur = PIND;
    static uint8_t prev = 0;
    uint8_t chg = cur ^ prev;
    prev = cur;
    if (chg & (1 << 4)) {
        if (cur & (1 << 4)) riseThrot = now;
        else {
            uint16_t w = (uint16_t)(now - riseThrot);
            if (w > 800 && w < 2200) rxThrot = w;
        }
    }
}

ISR(PCINT1_vect) {
    uint32_t now = micros();
    uint8_t cur = PINC;
    static uint8_t prev = 0;
    uint8_t chg = cur ^ prev;
    prev = cur;
    if (chg & (1 << 3)) {
        if (cur & (1 << 3)) riseYaw = now;
        else {
            uint16_t w = (uint16_t)(now - riseYaw);
            if (w > 800 && w < 2200) rxYaw = w;
        }
    }
    if (chg & (1 << 2)) {
        if (cur & (1 << 2)) riseSwitch = now;
        else {
            uint16_t w = (uint16_t)(now - riseSwitch);
            if (w > 800 && w < 2200) rxSwitch = w;
        }
    }
}

uint16_t readPWM(uint8_t pin) {
    uint8_t sreg = SREG; cli();
    uint16_t val;
    if      (pin == PIN_RX_THROT) val = rxThrot;
    else if (pin == PIN_RX_YAW)   val = rxYaw;
    else if (pin == PIN_RX_SW)    val = rxSwitch;
    else                           val = 1500;
    SREG = sreg;
    if (val < 800 || val > 2200) {
        if (pin == PIN_RX_THROT) { Serial.print(F("!!! ERROR: Throttle Signal Lost. val=")); Serial.println(val); return 1000; }
        if (pin == PIN_RX_YAW)   { Serial.println(F("!!! ERROR: Yaw Signal Lost !!!")); return 1500; }
        if (pin == PIN_RX_SW)    { Serial.println(F("!!! ERROR: Switch Signal Lost !!!")); return 1500; }
    }
    return val;
}

// ══════════════════════════════════════════════════════════
//  IMU
// ══════════════════════════════════════════════════════════
int16_t rawAcX, rawAcY, rawAcZ;
int16_t rawGyX, rawGyY, rawGyZ;

float gyroXoff = 0, gyroYoff = 0, gyroZoff = 0;
float pitchOffset = 0, rollOffset = 0;
float pitch = 0, roll = 0, yaw = 0;
float accPitchFilt = 0, accRollFilt = 0;

// ══════════════════════════════════════════════════════════
//  PID STATE
// ══════════════════════════════════════════════════════════
float pitchInt = 0,  rollInt = 0,  yawInt = 0;
float pitchPrev= 0,  rollPrev= 0,  yawPrev= 0;
float pitchDFilt=0,  rollDFilt=0,  yawDFilt=0;
float pitchPID = 0,  rollPID = 0,  yawPID = 0;

uint16_t pwmFL = 1000, pwmFR = 1000, pwmBR = 1000, pwmBL = 1000;

bool armed         = false;
bool servoUnlocked = false;
uint32_t servoAt   = 0;

// ══════════════════════════════════════════════════════════
//  MPU-6050
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
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);
    rawAcX = Wire.read() << 8 | Wire.read();
    rawAcY = Wire.read() << 8 | Wire.read();
    rawAcZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    rawGyX = Wire.read() << 8 | Wire.read();
    rawGyY = Wire.read() << 8 | Wire.read();
    rawGyZ = Wire.read() << 8 | Wire.read();
}

void mpuInit() {
    Wire.begin();
    Wire.setClock(400000L);
    mpuWrite(0x6B, 0x01);
    delay(100);
    mpuWrite(0x1A, 0x03);
    mpuWrite(0x1B, 0x00);
    mpuWrite(0x1C, 0x00);
    mpuWrite(0x19, 0x00);
}

// ══════════════════════════════════════════════════════════
//  CALIBRATION
// ══════════════════════════════════════════════════════════
void calibrate() {
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

    Serial.print(F("Gyro offsets: X=")); Serial.print(gyroXoff, 2);
    Serial.print(F("  Y="));             Serial.print(gyroYoff, 2);
    Serial.print(F("  Z="));             Serial.println(gyroZoff, 2);

    float pSum = 0, rSum = 0;
    const int ACC_N = 1000;
    for (int i = 0; i < ACC_N; i++) {
        readMPU();
        float ax = (float)rawAcX, ay = (float)rawAcY, az = (float)rawAcZ;
        pSum += atan2(ax, sqrt(ay*ay + az*az)) * 57.2958f;
        rSum += atan2(ay, sqrt(ax*ax + az*az)) * 57.2958f;
        delay(2);
    }
    pitchOffset = pSum / ACC_N;
    rollOffset  = rSum / ACC_N;

    Serial.print(F("Angle offsets: pitch=")); Serial.print(pitchOffset, 3);
    Serial.print(F("  roll="));               Serial.println(rollOffset, 3);
    Serial.println(F("CALIBRATION DONE"));

    beepPattern(3);    // 3 short beeps — calibration complete
}

// ══════════════════════════════════════════════════════════
//  ATTITUDE UPDATE
// ══════════════════════════════════════════════════════════
void updateAttitude(float dt) {
    readMPU();

    float gyroX = (rawGyX - gyroXoff) * INV_GYRO_SCALE;
    float gyroY = (rawGyY - gyroYoff) * INV_GYRO_SCALE;
    float gyroZ = (rawGyZ - gyroZoff) * INV_GYRO_SCALE;

    float ax = (float)rawAcX;
    float ay = (float)rawAcY;
    float az = (float)rawAcZ;

    float ax2 = ax * ax;
    float ay2 = ay * ay;
    float az2 = az * az;

    float accPitch = atan2(ax, sqrt(ay2 + az2)) * 57.2958f - pitchOffset;
    float accRoll  = atan2(ay, sqrt(ax2 + az2)) * 57.2958f - rollOffset;

    accPitchFilt = LP_ACC * accPitchFilt + LP_ACC_INV * accPitch;
    accRollFilt  = LP_ACC * accRollFilt  + LP_ACC_INV * accRoll;

    pitch = CF_ALPHA * (pitch + gyroY * dt) + CF_ALPHA_INV * accPitchFilt;
    roll  = CF_ALPHA * (roll  + gyroX * dt) + CF_ALPHA_INV * accRollFilt;

    if (pitch >  90.0f || pitch < -90.0f) pitch = accPitchFilt;
    if (roll  >  90.0f || roll  < -90.0f) roll  = accRollFilt;

    yaw += gyroZ * dt;
    if (yaw >  180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;
}

// ══════════════════════════════════════════════════════════
//  PID
// ══════════════════════════════════════════════════════════
float computePID(float setpoint, float measured,
                 float kp, float ki, float kd,
                 float &errPrev, float &integral, float &derivFilt,
                 float dt, float invDt) {
    float err = setpoint - measured;
    integral += err * dt;
    integral  = constrain(integral, -I_LIMIT, I_LIMIT);
    float deriv = (err - errPrev) * invDt;
    derivFilt   = LP_DERIV * derivFilt + LP_DERIV_INV * deriv;
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
inline uint16_t clampPWM(float v) {
    if (v < THR_MOTOR_MIN) return THR_MOTOR_MIN;
    if (v > THR_MAX)       return THR_MAX;
    return (uint16_t)v;
}

void writeMotors(uint16_t fl, uint16_t fr, uint16_t br, uint16_t bl) {
    escFL.writeMicroseconds(fl);
    escFR.writeMicroseconds(fr);
    escBR.writeMicroseconds(br);
    escBL.writeMicroseconds(bl);
}

void motorsOff() { writeMotors(1000, 1000, 1000, 1000); }

// ══════════════════════════════════════════════════════════
//  RFID / SERVO
// ══════════════════════════════════════════════════════════
void handleRFID(uint32_t now) {
    if (servoUnlocked && (now - servoAt >= SERVO_HOLD_MS)) {
        servoLock.write(SERVO_LOCK_DEG);
        servoUnlocked = false;
        Serial.println(F("RFID: locked (timeout)"));
        beepShort();                       // 1 short — auto locked
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
        Serial.println(F("RFID: AUTHORISED - servo unlocked"));
        servoLock.write(SERVO_UNLOCK_DEG);
        servoUnlocked = true;
        servoAt = now;
        beepAuthorised();                  // ascending 3 tones
    } else {
        Serial.println(F("RFID: DENIED"));
        beepDenied();                      // low double beep
    }

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
}

// ══════════════════════════════════════════════════════════
//  STICK HELPERS
// ══════════════════════════════════════════════════════════
inline float stickToAngle(uint16_t pw, float maxDeg) {
    int16_t c = (int16_t)pw - 1500;
    if (c > -STICK_DB && c < STICK_DB) return 0.0f;
    return (float)c * INV_STICK * maxDeg;
}

inline float stickToRate(uint16_t pw, float maxRate) {
    int16_t c = (int16_t)pw - 1500;
    if (c > -STICK_DB && c < STICK_DB) return 0.0f;
    return (float)c * INV_STICK * maxRate;
}

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
    beepShort();                           // power-on beep

    Serial.println(F("============================================"));
    Serial.println(F("   F450 FLIGHT CONTROLLER  v2.3.0"));
    Serial.println(F("============================================"));

    pinMode(PIN_RX_ROLL,  INPUT);
    pinMode(PIN_RX_PITCH, INPUT);
    pinMode(PIN_RX_THROT, INPUT);
    pinMode(PIN_RX_YAW,   INPUT);
    pinMode(PIN_RX_SW,    INPUT);
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
    Serial.print(F("  WHO_AM_I = 0x")); Serial.println(whoami, HEX);
    if (whoami == 0x00 || whoami == 0xFF) {
        Serial.println(F("  ERROR: No response on I2C. Check wiring. HALTED."));
        beepError();                       // rapid beep loop — never returns
    }
    Serial.println(F("  MPU-6050 OK."));
    beepShort();                           // step 1 done

    // ── Calibration
    Serial.println(F("[2/5] Calibrating IMU..."));
    calibrate();                           // beepPattern(3) called inside

    // ── RFID
    Serial.println(F("[3/5] Initialising RFID RC522..."));
    SPI.begin();
    rfid.PCD_Init();
    rfid.PCD_DumpVersionToSerial();
    Serial.println(F("  RC522 ready."));
    beepShort();                           // step 3 done

    // ── Servo
    Serial.println(F("[4/5] Initialising servo..."));
    servoLock.attach(PIN_SERVO);
    servoLock.write(SERVO_LOCK_DEG);
    delay(500);
    Serial.println(F("  Servo at 0 deg (locked)."));
    beepShort();                           // step 4 done

    // ── ESC arming
    Serial.println(F("[5/5] Arming ESCs (3 s at 1000 us)..."));
    escFL.attach(PIN_ESC_FL, 1000, 2000);
    escFR.attach(PIN_ESC_FR, 1000, 2000);
    escBR.attach(PIN_ESC_BR, 1000, 2000);
    escBL.attach(PIN_ESC_BL, 1000, 2000);
    motorsOff();
    delay(3000);
    Serial.println(F("  ESCs armed."));
    beepShort();                           // step 5 done

    // ── Interrupts
    attachInterrupt(digitalPinToInterrupt(PIN_RX_ROLL),  isrRoll,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_RX_PITCH), isrPitch, CHANGE);
    PCICR  |= (1 << PCIE2);
    PCMSK2  = (1 << PCINT20);
    PCICR  |= (1 << PCIE1);
    PCMSK1  = (1 << PCINT10) | (1 << PCINT11);

    Serial.println(F("============================================"));
    Serial.println(F("  SYSTEM READY"));
    Serial.println(F("  ARM  : Throttle LOW + Yaw RIGHT (>1800us)"));
    Serial.println(F("  DISARM: Throttle LOW + Yaw LEFT (<1200us)"));
    Serial.println(F("============================================"));

    beepPattern(2, true);                  // 2 long beeps — system ready
}

// ══════════════════════════════════════════════════════════
//  MAIN LOOP  –  250 Hz
// ══════════════════════════════════════════════════════════
void loop() {
    static uint32_t loopTimer = 0;
    uint32_t now = micros();
    if ((uint32_t)(now - loopTimer) < LOOP_US) return;

    float dt = (float)(now - loopTimer) * 1e-6f;
    loopTimer = now;
    if (dt <= 0.0f || dt > 0.1f) return;

    float invDt = 1.0f / dt;

    RxSnapshot rx = readRx();
    uint32_t ms = millis();

    // ── Servo state change
    static uint8_t lastServoState = 2;
    uint8_t wantServo = (rx.sw < 1300) ? 0 : 1;
    if (wantServo != lastServoState) {
        if (wantServo == 0) {
            servoLock.write(SERVO_LOCK_DEG);
            servoUnlocked = false;
        } else {
            servoLock.write(SERVO_UNLOCK_DEG);
            servoUnlocked = true;
            servoAt = ms;
        }
        lastServoState = wantServo;
    }

    updateAttitude(dt);

    // ── Arm / disarm
    if (!armed) {
        if (rx.throt < THR_ARM && rx.yaw > 1800) {
            armed = true;
            resetPID();
            Serial.println(F(">>> ARMED <<<"));
            beepPattern(2);                // 2 short beeps — armed
        }
    } else {
        if (rx.throt < THR_ARM && rx.yaw < 1200) {
            armed = false;
            motorsOff();
            resetPID();
            Serial.println(F(">>> DISARMED <<<"));
            beepLong();                    // 1 long beep — disarmed
        }
    }

    handleRFID(ms);

    if (!armed) return;

    uint16_t thr = constrain(rx.throt, THR_MIN, THR_MAX);

    float spRoll  = stickToAngle(rx.roll,  MAX_BANK_DEG);
    float spPitch = stickToAngle(rx.pitch, MAX_BANK_DEG);
    float spYaw   = stickToRate (rx.yaw,   MAX_YAW_RATE);

    rollPID  = computePID(spRoll,  roll,  KP_RP,  KI_RP,  KD_RP,
                          rollPrev,  rollInt,  rollDFilt,  dt, invDt);
    pitchPID = computePID(spPitch, pitch, KP_RP,  KI_RP,  KD_RP,
                          pitchPrev, pitchInt, pitchDFilt, dt, invDt);

    static float yawAnglePrev = 0;
    float yawRate    = (yaw - yawAnglePrev) * invDt;
    yawAnglePrev     = yaw;
    yawPID = computePID(spYaw, yawRate, KP_YAW, KI_YAW, KD_YAW,
                        yawPrev, yawInt, yawDFilt, dt, invDt);

    pwmFL = clampPWM(thr + rollPID + pitchPID - yawPID);
    pwmFR = clampPWM(thr - rollPID + pitchPID + yawPID);
    pwmBR = clampPWM(thr - rollPID - pitchPID - yawPID);
    pwmBL = clampPWM(thr + rollPID - pitchPID + yawPID);

    writeMotors(pwmFL, pwmFR, pwmBR, pwmBL);
}

/* ─── END OF FILE ──────────────────────────────────────── */