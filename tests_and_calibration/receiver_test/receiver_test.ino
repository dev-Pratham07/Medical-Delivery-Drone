/*
 * FlySky FS-i6 / FS-iA6B Receiver Test
 * Arduino Uno
 *
 * Reads PWM signals from receiver channels
 * and prints pulse widths to Serial Monitor.
 *
 * Connections:
 * ---------------------------------
 * Receiver CH1 -> D2   Roll
 * Receiver CH2 -> D3   Pitch
 * Receiver CH3 -> D4   Throttle
 * Receiver CH4 -> D5   Yaw
 * Receiver CH5 -> D6   Switch
 * Receiver GND -> Arduino GND
 * Receiver VCC -> 5V
 *
 * Serial Monitor:
 * 115200 baud
 */

#include <PinChangeInterrupt.h>

// ---------------------------------
// Receiver Pins
// ---------------------------------

#define CH1_PIN 2
#define CH2_PIN 3
#define CH3_PIN 4
#define CH4_PIN A3
#define CH5_PIN A2

// ---------------------------------
// Channel indexes
// ---------------------------------

#define IDX_CH1 0
#define IDX_CH2 1
#define IDX_CH3 2
#define IDX_CH4 3
#define IDX_CH5 4

volatile uint16_t rcValue[5] = {
  1500, 1500, 1500, 1500, 1500
};

volatile uint32_t riseTime[5];

// =================================
// INTERRUPTS
// =================================

void ch1_ISR() {

  if (digitalRead(CH1_PIN)) {

    riseTime[IDX_CH1] = micros();

  } else {

    rcValue[IDX_CH1] =
      micros() - riseTime[IDX_CH1];
  }
}

void ch2_ISR() {

  if (digitalRead(CH2_PIN)) {

    riseTime[IDX_CH2] = micros();

  } else {

    rcValue[IDX_CH2] =
      micros() - riseTime[IDX_CH2];
  }
}

void pinChange_ISR() {

  uint32_t now = micros();

  // CH3
  if (digitalRead(CH3_PIN)) {

    riseTime[IDX_CH3] = now;

  } else {

    rcValue[IDX_CH3] =
      now - riseTime[IDX_CH3];
  }

  // CH4
  if (digitalRead(CH4_PIN)) {

    riseTime[IDX_CH4] = now;

  } else {

    rcValue[IDX_CH4] =
      now - riseTime[IDX_CH4];
  }

  // CH5
  if (digitalRead(CH5_PIN)) {

    riseTime[IDX_CH5] = now;

  } else {

    rcValue[IDX_CH5] =
      now - riseTime[IDX_CH5];
  }
}

// =================================
// SETUP
// =================================

void setup() {

  Serial.begin(115200);

  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  pinMode(CH4_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);

  // External interrupts
  attachInterrupt(
    digitalPinToInterrupt(CH1_PIN),
    ch1_ISR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(CH2_PIN),
    ch2_ISR,
    CHANGE
  );

  // Pin change interrupts
  attachPinChangeInterrupt(
    digitalPinToPinChangeInterrupt(CH3_PIN),
    pinChange_ISR,
    CHANGE
  );

  attachPinChangeInterrupt(
    digitalPinToPinChangeInterrupt(CH4_PIN),
    pinChange_ISR,
    CHANGE
  );

  attachPinChangeInterrupt(
    digitalPinToPinChangeInterrupt(CH5_PIN),
    pinChange_ISR,
    CHANGE
  );

  Serial.println("FS-i6 Receiver Test Ready");
}

// =================================
// LOOP
// =================================

void loop() {

  noInterrupts();

  uint16_t ch1 = rcValue[IDX_CH1];
  uint16_t ch2 = rcValue[IDX_CH2];
  uint16_t ch3 = rcValue[IDX_CH3];
  uint16_t ch4 = rcValue[IDX_CH4];
  uint16_t ch5 = rcValue[IDX_CH5];

  interrupts();

  Serial.print("CH1: ");
  Serial.print(ch1);

  Serial.print(" | CH2: ");
  Serial.print(ch2);

  Serial.print(" | CH3: ");
  Serial.print(ch3);

  Serial.print(" | CH4: ");
  Serial.print(ch4);

  Serial.print(" | CH5: ");
  Serial.println(ch5);

  delay(100);
}