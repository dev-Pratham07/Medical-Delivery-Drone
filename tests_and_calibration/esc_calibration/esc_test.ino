#include <Servo.h>

const int PWM_MIN = 1000;
const int PWM_MAX = 2000;
const int PWM_MID = 1500;
const int THROTTLE_CUTOFF = 1050;
const unsigned long RC_TIMEOUT_US = 25000;

// ESC objects
Servo esc1, esc2, esc3, esc4;

// Receiver pins
int chThrottle = 2;
int chRoll = 4;
int chPitch = 7;
int chYaw = 8;

// ESC pins
int m1 = 3;  // Front Left
int m2 = 5;  // Front Right
int m3 = 6;  // Back Right
int m4 = 9;  // Back Left

// Raw inputs
int throttle, roll, pitch, yaw;

// Outputs
int m1_out, m2_out, m3_out, m4_out;

void stopAll();

void setup() {
  pinMode(chThrottle, INPUT);
  pinMode(chRoll, INPUT);
  pinMode(chPitch, INPUT);
  pinMode(chYaw, INPUT);

  esc1.attach(m1);
  esc2.attach(m2);
  esc3.attach(m3);
  esc4.attach(m4);

  Serial.begin(9600);

  // Safe startup
  for (int i = 0; i < 200; i++) {
    esc1.writeMicroseconds(PWM_MIN);
    esc2.writeMicroseconds(PWM_MIN);
    esc3.writeMicroseconds(PWM_MIN);
    esc4.writeMicroseconds(PWM_MIN);
    delay(10);
  }

  Serial.println("Quad Ready");
}

void loop() {

  // Read receiver signals
  throttle = pulseIn(chThrottle, HIGH, RC_TIMEOUT_US);
  roll     = pulseIn(chRoll, HIGH, RC_TIMEOUT_US);
  pitch    = pulseIn(chPitch, HIGH, RC_TIMEOUT_US);
  yaw      = pulseIn(chYaw, HIGH, RC_TIMEOUT_US);

  // Fail-safe: throttle loss must always stop motors.
  if (throttle == 0) {
    stopAll();
    Serial.println("No throttle signal");
    return;
  }

  // If attitude channels drop out, keep them neutral instead of forcing full deflection.
  if (roll == 0) {
    roll = PWM_MID;
  }
  if (pitch == 0) {
    pitch = PWM_MID;
  }
  if (yaw == 0) {
    yaw = PWM_MID;
  }

  // Constrain inputs
  throttle = constrain(throttle, PWM_MIN, PWM_MAX);
  roll     = constrain(roll, PWM_MIN, PWM_MAX);
  pitch    = constrain(pitch, PWM_MIN, PWM_MAX);
  yaw      = constrain(yaw, PWM_MIN, PWM_MAX);

  // Convert to centered values (-500 to +500)
  int r = roll - PWM_MID;
  int p = pitch - PWM_MID;
  int y = yaw - PWM_MID;

  // Basic mixing (Quad X)
  m1_out = throttle + p + r - y; // Front Left
  m2_out = throttle + p - r + y; // Front Right
  m3_out = throttle - p - r - y; // Back Right
  m4_out = throttle - p + r + y; // Back Left

  // Constrain outputs
  m1_out = constrain(m1_out, PWM_MIN, PWM_MAX);
  m2_out = constrain(m2_out, PWM_MIN, PWM_MAX);
  m3_out = constrain(m3_out, PWM_MIN, PWM_MAX);
  m4_out = constrain(m4_out, PWM_MIN, PWM_MAX);

  // Deadband for stop
  if (throttle < THROTTLE_CUTOFF) {
    stopAll();
  } else {
    esc1.writeMicroseconds(m1_out);
    esc2.writeMicroseconds(m2_out);
    esc3.writeMicroseconds(m3_out);
    esc4.writeMicroseconds(m4_out);
  }

  // Debug
  Serial.print("T:");
  Serial.print(throttle);
  Serial.print(" R:");
  Serial.print(roll);
  Serial.print(" P:");
  Serial.print(pitch);
  Serial.print(" Y:");
  Serial.print(yaw);
  Serial.println();

  delay(20);
}

// Stop all motors
void stopAll() {
  esc1.writeMicroseconds(PWM_MIN);
  esc2.writeMicroseconds(PWM_MIN);
  esc3.writeMicroseconds(PWM_MIN);
  esc4.writeMicroseconds(PWM_MIN);
}