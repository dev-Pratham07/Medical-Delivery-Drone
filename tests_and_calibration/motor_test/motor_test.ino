#include <Servo.h>

Servo esc1, esc2, esc3, esc4;

void setup() {
  esc1.attach(3);
  esc2.attach(5);
  esc3.attach(6);
  esc4.attach(9);

  delay(2000); // wait before ESC power
}

void loop() {
  // All motors same speed
  esc1.writeMicroseconds(1200);
  esc2.writeMicroseconds(1200);
  esc3.writeMicroseconds(1200);
  esc4.writeMicroseconds(1200);
}