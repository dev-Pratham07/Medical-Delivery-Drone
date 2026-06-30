# Convertible Drone MVP Firmware

This folder contains a safe-start firmware baseline for Arduino UNO.

## What this implementation includes

- Dual-mode state machine: `IDLE`, `TRANSITIONING`, `FLYING`, `ROLLING`, `FAILSAFE`
- Hard interlocks for transition:
  - throttle must be low
  - motors must be disarmed
  - transition timeout protection
- Limit-switch confirmation for final servo position at 0 deg and 90 deg
- Receiver link timeout fail-safe
- Kill switch input fail-safe
- Battery voltage monitoring with low/critical thresholds
- Rolling-mode tank steering within constrained PWM range (1100 to 1300 us)
- Serial telemetry output for bring-up/debug

## File

- `convertible_drone_mvp.ino`

## Bring-up notes

1. Calibrate ESCs before first arming.
2. Verify limit switch logic with serial output before spinning motors.
3. Keep propellers removed during first bench tests.
4. Tune battery divider ratio (`BAT_DIVIDER_RATIO`) using a multimeter.
5. Confirm kill switch path before any tethered hover tests.

## Next firmware steps

1. Replace `pulseIn` receiver reads with interrupt-based capture for lower latency.
2. Add MPU6050 read/filter and stabilization loop for `FLYING` state.
3. Add motor current or RPM-based stall detection for `ROLLING` state.
4. Add transition-event logging for test report evidence.
