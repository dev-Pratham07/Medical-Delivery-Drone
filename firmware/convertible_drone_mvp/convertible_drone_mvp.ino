#include <Servo.h>
#include <Wire.h>

// ===== Pin Map =====
// ESC signal pins (front-left, front-right, back-left, back-right)
const uint8_t PIN_ESC_FL = 3;
const uint8_t PIN_ESC_FR = 5;
const uint8_t PIN_ESC_BL = 6;
const uint8_t PIN_ESC_BR = 9;

// Servo pins (left tilt, right tilt)
const uint8_t PIN_SERVO_LEFT = 10;
const uint8_t PIN_SERVO_RIGHT = 11;

// Receiver pins (use PWM/PPM channels as simple pulse inputs for MVP)
const uint8_t PIN_RX_THROTTLE = 2;
const uint8_t PIN_RX_STEER = 4;
const uint8_t PIN_RX_MODE = 7;
const uint8_t PIN_RX_KILL = 8;

// Position confirmation (limit switches) for 0 deg (flight) and 90 deg (rolling)
const uint8_t PIN_LEFT_FLIGHT_SW = A0;
const uint8_t PIN_LEFT_ROLL_SW = A1;
const uint8_t PIN_RIGHT_FLIGHT_SW = A2;
const uint8_t PIN_RIGHT_ROLL_SW = A3;

// Battery monitor and buzzer
const uint8_t PIN_BATTERY = A4;
const uint8_t PIN_BUZZER = 12;

// ===== Safety/Control Constants =====
const uint16_t PWM_MIN = 1000;
const uint16_t PWM_ARM = 1000;
const uint16_t PWM_MAX = 2000;
const uint16_t PWM_ROLL_MIN = 1100;
const uint16_t PWM_ROLL_MAX = 1300;

const uint16_t THROTTLE_ZERO_MAX = 1040;
const uint16_t RX_VALID_MIN = 900;
const uint16_t RX_VALID_MAX = 2100;

const uint32_t RX_TIMEOUT_MS = 200;
const uint32_t TRANSITION_TIMEOUT_MS = 5000;
const uint32_t LOOP_PERIOD_MS = 10;  // 100 Hz loop target

const uint16_t SERVO_FLIGHT_DEG = 0;
const uint16_t SERVO_ROLL_DEG = 90;

// Set after voltage divider calibration. Example assumes divider ratio = 3.2.
const float ADC_REF_V = 5.0f;
const float BAT_DIVIDER_RATIO = 3.2f;
const float LOW_BATTERY_V = 10.2f;
const float CRITICAL_BATTERY_V = 9.6f;

enum ModeState {
  IDLE = 0,
  TRANSITIONING,
  FLYING,
  ROLLING,
  FAILSAFE
};

enum TargetMode {
  TARGET_FLY = 0,
  TARGET_ROLL
};

struct RxInput {
  uint16_t throttle;
  uint16_t steer;
  uint16_t mode;
  uint16_t kill;
  bool linkAlive;
};

Servo escFL;
Servo escFR;
Servo escBL;
Servo escBR;
Servo tiltLeft;
Servo tiltRight;

ModeState g_state = IDLE;
TargetMode g_targetMode = TARGET_FLY;
RxInput g_rx = {PWM_MIN, 1500, 1000, 1000, false};

uint32_t g_lastRxOkMs = 0;
uint32_t g_lastLoopMs = 0;
uint32_t g_transitionStartMs = 0;

uint16_t g_outFL = PWM_MIN;
uint16_t g_outFR = PWM_MIN;
uint16_t g_outBL = PWM_MIN;
uint16_t g_outBR = PWM_MIN;

static inline bool inRange(uint16_t v, uint16_t lo, uint16_t hi) {
  return (v >= lo && v <= hi);
}

uint16_t readPulseSafe(uint8_t pin, uint16_t fallbackUs) {
  unsigned long us = pulseIn(pin, HIGH, 25000UL);
  if (us == 0UL) {
    return fallbackUs;
  }
  if (!inRange((uint16_t)us, RX_VALID_MIN, RX_VALID_MAX)) {
    return fallbackUs;
  }
  return (uint16_t)us;
}

void readReceiver() {
  uint16_t t = readPulseSafe(PIN_RX_THROTTLE, g_rx.throttle);
  uint16_t s = readPulseSafe(PIN_RX_STEER, g_rx.steer);
  uint16_t m = readPulseSafe(PIN_RX_MODE, g_rx.mode);
  uint16_t k = readPulseSafe(PIN_RX_KILL, g_rx.kill);

  bool valid = inRange(t, RX_VALID_MIN, RX_VALID_MAX) &&
               inRange(s, RX_VALID_MIN, RX_VALID_MAX) &&
               inRange(m, RX_VALID_MIN, RX_VALID_MAX) &&
               inRange(k, RX_VALID_MIN, RX_VALID_MAX);

  if (valid) {
    g_rx.throttle = t;
    g_rx.steer = s;
    g_rx.mode = m;
    g_rx.kill = k;
    g_rx.linkAlive = true;
    g_lastRxOkMs = millis();
  } else {
    g_rx.linkAlive = (millis() - g_lastRxOkMs) < RX_TIMEOUT_MS;
  }
}

void writeEscAll(uint16_t fl, uint16_t fr, uint16_t bl, uint16_t br) {
  g_outFL = constrain(fl, PWM_MIN, PWM_MAX);
  g_outFR = constrain(fr, PWM_MIN, PWM_MAX);
  g_outBL = constrain(bl, PWM_MIN, PWM_MAX);
  g_outBR = constrain(br, PWM_MIN, PWM_MAX);

  escFL.writeMicroseconds(g_outFL);
  escFR.writeMicroseconds(g_outFR);
  escBL.writeMicroseconds(g_outBL);
  escBR.writeMicroseconds(g_outBR);
}

void disarmMotors() {
  writeEscAll(PWM_ARM, PWM_ARM, PWM_ARM, PWM_ARM);
}

bool motorsDisarmed() {
  return g_outFL <= PWM_ARM && g_outFR <= PWM_ARM && g_outBL <= PWM_ARM && g_outBR <= PWM_ARM;
}

bool throttleIsZero() {
  return g_rx.throttle <= THROTTLE_ZERO_MAX;
}

bool killRequested() {
  return g_rx.kill > 1600;
}

float readBatteryVoltage() {
  int raw = analogRead(PIN_BATTERY);
  float vAdc = (raw * ADC_REF_V) / 1023.0f;
  return vAdc * BAT_DIVIDER_RATIO;
}

void buzzerTick(bool on) {
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
}

void setTiltFlight() {
  tiltLeft.write(SERVO_FLIGHT_DEG);
  tiltRight.write(SERVO_FLIGHT_DEG);
}

void setTiltRoll() {
  tiltLeft.write(SERVO_ROLL_DEG);
  tiltRight.write(SERVO_ROLL_DEG);
}

bool isFlightSwitchConfirmed() {
  return digitalRead(PIN_LEFT_FLIGHT_SW) == LOW && digitalRead(PIN_RIGHT_FLIGHT_SW) == LOW;
}

bool isRollSwitchConfirmed() {
  return digitalRead(PIN_LEFT_ROLL_SW) == LOW && digitalRead(PIN_RIGHT_ROLL_SW) == LOW;
}

bool transitionComplete(TargetMode target) {
  return (target == TARGET_FLY) ? isFlightSwitchConfirmed() : isRollSwitchConfirmed();
}

void startTransition(TargetMode target) {
  g_targetMode = target;
  g_state = TRANSITIONING;
  g_transitionStartMs = millis();
  disarmMotors();

  if (target == TARGET_FLY) {
    setTiltFlight();
  } else {
    setTiltRoll();
  }
}

void updateTransition() {
  if (!throttleIsZero() || !motorsDisarmed()) {
    g_state = FAILSAFE;
    return;
  }

  if (transitionComplete(g_targetMode)) {
    g_state = (g_targetMode == TARGET_FLY) ? FLYING : ROLLING;
    return;
  }

  if ((millis() - g_transitionStartMs) > TRANSITION_TIMEOUT_MS) {
    g_state = FAILSAFE;
  }
}

void updateFlying() {
  // MVP: pass-through throttle equally. Stabilization loop goes here later.
  uint16_t t = constrain(g_rx.throttle, PWM_MIN, PWM_MAX);
  writeEscAll(t, t, t, t);

  // Interlock: switching only allowed when throttle is zero and motors disarmed.
  if (g_rx.mode > 1600 && throttleIsZero()) {
    disarmMotors();
    if (motorsDisarmed()) {
      startTransition(TARGET_ROLL);
    }
  }
}

void updateRolling() {
  // Tank-style steering in rolling mode, with bounded PWM range.
  int16_t steer = (int16_t)g_rx.steer - 1500;
  int16_t base = map(g_rx.throttle, PWM_MIN, PWM_MAX, PWM_ROLL_MIN, PWM_ROLL_MAX);

  int16_t left = base - (steer / 4);
  int16_t right = base + (steer / 4);

  uint16_t leftCmd = constrain(left, PWM_ROLL_MIN, PWM_ROLL_MAX);
  uint16_t rightCmd = constrain(right, PWM_ROLL_MIN, PWM_ROLL_MAX);

  // Left side motors: FL + BL, Right side: FR + BR
  writeEscAll(leftCmd, rightCmd, leftCmd, rightCmd);

  if (g_rx.mode < 1400 && throttleIsZero()) {
    disarmMotors();
    if (motorsDisarmed()) {
      startTransition(TARGET_FLY);
    }
  }
}

void updateFailsafe() {
  disarmMotors();
  buzzerTick((millis() / 200) % 2 == 0);

  // Manual recovery path: kill low, link alive, throttle low.
  if (!killRequested() && g_rx.linkAlive && throttleIsZero()) {
    buzzerTick(false);
    g_state = IDLE;
  }
}

void updateStateMachine() {
  if (killRequested() || !g_rx.linkAlive) {
    g_state = FAILSAFE;
  }

  float vBat = readBatteryVoltage();
  if (vBat < CRITICAL_BATTERY_V) {
    g_state = FAILSAFE;
  }

  switch (g_state) {
    case IDLE:
      disarmMotors();
      if (!killRequested() && g_rx.linkAlive && throttleIsZero()) {
        // Choose initial mode by receiver switch (low = fly, high = roll).
        if (g_rx.mode > 1600) {
          startTransition(TARGET_ROLL);
        } else {
          startTransition(TARGET_FLY);
        }
      }
      break;

    case TRANSITIONING:
      updateTransition();
      break;

    case FLYING:
      updateFlying();
      break;

    case ROLLING:
      updateRolling();
      break;

    case FAILSAFE:
      updateFailsafe();
      break;
  }

  // Non-critical warning beep for low battery.
  if (vBat < LOW_BATTERY_V && g_state != FAILSAFE) {
    buzzerTick((millis() / 800) % 2 == 0);
  }
}

void printDebug() {
  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs < 250) {
    return;
  }
  lastPrintMs = millis();

  float vBat = readBatteryVoltage();
  Serial.print("state=");
  Serial.print((int)g_state);
  Serial.print(" thr=");
  Serial.print(g_rx.throttle);
  Serial.print(" steer=");
  Serial.print(g_rx.steer);
  Serial.print(" mode=");
  Serial.print(g_rx.mode);
  Serial.print(" kill=");
  Serial.print(g_rx.kill);
  Serial.print(" link=");
  Serial.print(g_rx.linkAlive ? 1 : 0);
  Serial.print(" vbat=");
  Serial.print(vBat, 2);
  Serial.print(" out=");
  Serial.print(g_outFL);
  Serial.print(",");
  Serial.print(g_outFR);
  Serial.print(",");
  Serial.print(g_outBL);
  Serial.print(",");
  Serial.println(g_outBR);
}

void setupPins() {
  pinMode(PIN_RX_THROTTLE, INPUT);
  pinMode(PIN_RX_STEER, INPUT);
  pinMode(PIN_RX_MODE, INPUT);
  pinMode(PIN_RX_KILL, INPUT);

  pinMode(PIN_LEFT_FLIGHT_SW, INPUT_PULLUP);
  pinMode(PIN_LEFT_ROLL_SW, INPUT_PULLUP);
  pinMode(PIN_RIGHT_FLIGHT_SW, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ROLL_SW, INPUT_PULLUP);

  pinMode(PIN_BATTERY, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
}

void setupActuators() {
  escFL.attach(PIN_ESC_FL);
  escFR.attach(PIN_ESC_FR);
  escBL.attach(PIN_ESC_BL);
  escBR.attach(PIN_ESC_BR);

  tiltLeft.attach(PIN_SERVO_LEFT);
  tiltRight.attach(PIN_SERVO_RIGHT);

  setTiltFlight();
  disarmMotors();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  setupPins();
  setupActuators();

  g_lastRxOkMs = millis();
  g_lastLoopMs = millis();
  g_state = IDLE;
}

void loop() {
  uint32_t now = millis();
  if (now - g_lastLoopMs < LOOP_PERIOD_MS) {
    return;
  }
  g_lastLoopMs = now;

  readReceiver();
  updateStateMachine();
  printDebug();
}
