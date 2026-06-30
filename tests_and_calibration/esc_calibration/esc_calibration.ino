/*
 * ============================================================
 *  ESC CALIBRATION SKETCH
 *  F450 Quadcopter — Arduino Uno/Nano
 *
 *  INSTRUCTIONS:
 *  1. REMOVE ALL PROPS before doing anything
 *  2. Disconnect battery
 *  3. Upload this sketch
 *  4. Open Serial Monitor at 115200 baud
 *  5. Follow the on-screen instructions
 *
 *  PIN MAPPING (must match your flight controller)
 *  ESC Front-Left  → D9
 *  ESC Front-Right → D10
 *  ESC Back-Right  → D5
 *  ESC Back-Left   → D6
 * ============================================================
 */

#include <Servo.h>

#define PIN_ESC_FL  9
#define PIN_ESC_FR  10
#define PIN_ESC_BR  5
#define PIN_ESC_BL  6

#define PWM_MAX     2000
#define PWM_MIN     1000
#define PWM_MID     1500

Servo escFL, escFR, escBR, escBL;

void writeAll(uint16_t val) {
    escFL.writeMicroseconds(val);
    escFR.writeMicroseconds(val);
    escBR.writeMicroseconds(val);
    escBL.writeMicroseconds(val);
}

void waitEnter(const char* msg) {
    Serial.println(msg);
    Serial.println(F("  --> Press ENTER when ready..."));
    while (true) {
        if (Serial.available()) {
            Serial.read();
            break;
        }
    }
    delay(500);
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    Serial.println(F("============================================"));
    Serial.println(F("       ESC CALIBRATION TOOL"));
    Serial.println(F("============================================"));
    Serial.println(F("!! MAKE SURE ALL PROPS ARE REMOVED !!"));
    Serial.println();

    // Attach ESCs — no signal yet
    escFL.attach(PIN_ESC_FL, 1000, 2000);
    escFR.attach(PIN_ESC_FR, 1000, 2000);
    escBR.attach(PIN_ESC_BR, 1000, 2000);
    escBL.attach(PIN_ESC_BL, 1000, 2000);

    // ── STEP 1: Send MAX signal before battery connect
    Serial.println(F("STEP 1: Sending MAX throttle (2000us)"));
    Serial.println(F("        Disconnect battery if connected."));
    writeAll(PWM_MAX);
    waitEnter("        Now connect the battery and wait for ESC startup beeps.");

    // ── STEP 2: ESC in calibration mode — send MIN
    Serial.println(F("STEP 2: Sending MIN throttle (1000us)"));
    Serial.println(F("        Wait for confirmation beeps from all ESCs..."));
    writeAll(PWM_MIN);
    delay(4000);   // give ESCs time to register min and beep

    Serial.println(F("        ESCs should have beeped to confirm calibration."));
    Serial.println();

    // ── STEP 3: Throttle sweep test
    waitEnter("STEP 3: Sweep test — will slowly ramp all motors up then down.");

    Serial.println(F("        Ramping UP..."));
    for (uint16_t v = PWM_MIN; v <= PWM_MAX; v += 5) {
        writeAll(v);
        Serial.print(F("  PWM = ")); Serial.println(v);
        delay(50);
    }

    Serial.println(F("        Ramping DOWN..."));
    for (uint16_t v = PWM_MAX; v >= PWM_MIN; v -= 5) {
        writeAll(v);
        Serial.print(F("  PWM = ")); Serial.println(v);
        delay(50);
    }

    writeAll(PWM_MIN);

    // ── STEP 4: Individual motor test
    waitEnter("STEP 4: Individual motor test at 1200us. Watch each motor.");

    Serial.println(F("  Testing FL (D9)..."));
    escFL.writeMicroseconds(1200);
    delay(3000);
    escFL.writeMicroseconds(PWM_MIN);
    delay(500);

    Serial.println(F("  Testing FR (D10)..."));
    escFR.writeMicroseconds(1200);
    delay(3000);
    escFR.writeMicroseconds(PWM_MIN);
    delay(500);

    Serial.println(F("  Testing BR (D5)..."));
    escBR.writeMicroseconds(1200);
    delay(3000);
    escBR.writeMicroseconds(PWM_MIN);
    delay(500);

    Serial.println(F("  Testing BL (D6)..."));
    escBL.writeMicroseconds(1200);
    delay(3000);
    escBL.writeMicroseconds(PWM_MIN);
    delay(500);

    Serial.println();
    Serial.println(F("============================================"));
    Serial.println(F("  CALIBRATION COMPLETE"));
    Serial.println(F("  All ESCs are now calibrated to the same"));
    Serial.println(F("  min/max range."));
    Serial.println(F(""));
    Serial.println(F("  Next steps:"));
    Serial.println(F("  1. Disconnect battery"));
    Serial.println(F("  2. Upload your flight controller sketch"));
    Serial.println(F("  3. Reattach props (correct direction!)"));
    Serial.println(F("============================================"));
}

void loop() {
    // Nothing — calibration is one-shot in setup()
    // Send idle signal to keep ESCs happy
    writeAll(PWM_MIN);
    delay(100);
}