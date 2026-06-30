#include <Servo.h>

Servo myServo;

void setup() {
  myServo.attach(7);   // Servo signal connected to D7
}

void loop() {
  myServo.write(60);    // Move to 60 degrees
  delay(5000);
  myServo.write(0);   // Move to 120 degrees
  delay(5000);
}