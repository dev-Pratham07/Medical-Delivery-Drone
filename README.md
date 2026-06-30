# Convertible Medical Delivery Drone
### Applied Electromechanics Course Project — Vishwakarma Institute of Technology (VIT)

An innovative hybrid locomotion system designed to address the challenges of remote medical cargo transport. This repository features firmware, calibration tools, and interactive slides for a **Convertible Medical Drone** that functions as both a ground-rolling vehicle and an aerial quadcopter. 

It also includes firmware for a secondary variant: an **RFID-Authenticated F450 Medical Cargo Quadcopter**.

---

## 🚁 Project Overview: The Convertible Drone

Standard quadcopters expend massive amounts of energy staying airborne. Over flat or rough ground terrains, ground rolling is significantly more power-efficient. 

This project implements a **Dual-Locomotion Convertible Drone** that transitions dynamically:
*   **Flying Mode:** Propellers face upward, operating as a standard quadcopter stabilized by an **MPU6050** Inertial Measurement Unit (IMU).
*   **Rolling Mode:** Motor arms rotate 90 degrees outward. The propellers spin slowly inside rigid 3D-printed outer rings (resembling a Mercedes logo) that function as protective wheels, allowing the vehicle to roll on the ground.
*   **Transition Mech:** High-torque **MG9966** servo motors actuate the pivot mechanisms, and micro limit-switches confirm position locks at both endpoints.

```
                  [Flight Mode]
             (Propellers facing UP)
                       │
                       ▼   [Interlock Verification: Throttle = 0, Disarmed, Stable]
             [Transition State] 
           (MG9966 Servos rotate 90°)
                       │
                       ▼   [Limit Switch Confirm: A1/A3 Low]
             [Rolling Mode]
           (Propellers face SIDEWAYS)
```

---

## 🛠️ System Components & Electronics

*   **Logic Controller:** Arduino UNO (Atmega328P).
*   **Attitude Stabilizer:** MPU6050 Gyroscope/Accelerometer (mounted on vibration-damping sponge pads with a software complementary filter).
*   **Radio Control Link:** Flysky FS-CT6B 6-Channel 2.4GHz Transmitter & Receiver.
*   **Tilt Actuators:** Dual MG9966 high-torque servo motors.
*   **Propulsion:** Four 1000kv Brushless DC (BLDC) motors managed by 30A Electronic Speed Controllers (ESCs).
*   **End-stop Position Verification:** Four micro limit switches checking structural alignments.
*   **Power Bus:** 3S LiPo battery (11.1V, 2200mAh) isolated through a 5V BEC (Battery Eliminator Circuit) to prevent motor noise from resetting the Arduino.
*   **Telemetry Alarms:** Resistor divider (A4) and active piezo buzzer (D12) for voltage and connection alarms.

### 🔧 Flight Controller Wiring
Below is the physical wiring layout of the Arduino UNO flight controller core:

(images/img 3.jpeg)

---

## 📋 Pin Mapping (Convertible Drone)

The firmware (`firmware/convertible_drone_mvp/`) configures the Arduino UNO pinout as follows:

| Arduino Pin | Connected Component | Signal Type | Description |
| :--- | :--- | :--- | :--- |
| **D3** | ESC Front-Left (FL) | PWM Output | Propulsion motor speed |
| **D5** | ESC Front-Right (FR) | PWM Output | Propulsion motor speed |
| **D6** | ESC Back-Left (BL) | PWM Output | Propulsion motor speed |
| **D9** | ESC Back-Right (BR) | PWM Output | Propulsion motor speed |
| **D10** | Left Tilt Servo | PWM Output | Rotates Left arm (0° - 90°) |
| **D11** | Right Tilt Servo | PWM Output | Rotates Right arm (0° - 90°) |
| **D2** | RC CH3 (Throttle) | Interrupt Input | Controls vertical thrust / rolling power |
| **D4** | RC CH1/CH4 (Steer) | Pulse Input | Roll in flight / Differential steering on ground |
| **D7** | RC CH5 (Mode Switch) | Pulse Input | Triggers flying ↔ rolling transition |
| **D8** | RC CH6 (Kill Switch) | Pulse Input | Hard emergency disarm lockout |
| **A0** | Left 0° Limit Switch | Input Pullup | Closed when Left arm is locked at 0° (Flight) |
| **A1** | Left 90° Limit Switch | Input Pullup | Closed when Left arm is locked at 90° (Rolling) |
| **A2** | Right 0° Limit Switch | Input Pullup | Closed when Right arm is locked at 0° (Flight) |
| **A3** | Right 90° Limit Switch | Input Pullup | Closed when Right arm is locked at 90° (Rolling) |
| **A4** | Battery Voltage Divider | Analog Input | Voltage telemetry reading |
| **D12** | Piezo Buzzer | Digital Output | Audio failsafe and warning alarms |

---

## 🔒 Firmware State Machine & Safety Interlocks

The flight controller firmware executes an event-driven state machine:
1.  **IDLE:** Motors disarmed, waiting for arming command from receiver.
2.  **FLYING:** PID stabilization loop active. Throttle commands mapped to motors.
3.  **ROLLING:** Ground drive active. Throttle is mapped to a constrained range (**1100 to 1300 us**) to prevent BLDC motor stalling under wheel load. **Differential tank steering** is enabled: roll/yaw stick adjustments apply differential speed to left (FL+BL) and right (FR+BR) motor wheels.
4.  **TRANSITIONING:** BLDC motors are disarmed. MG9966 servos actuate rotation. Code blocks transition checks until limit switches verify successful lockouts.
5.  **FAILSAFE:** Instantly shuts down and disarms all motors. Triggered by:
    *   Receiver connection loss (timeout > 200 ms).
    *   Emergency manual kill switch activation.
    *   Critical battery voltage detection (< 9.6V).
    *   Transition timeout exceeding 5 seconds.

### ⚠️ Transition Safety Interlocks
To prevent crashes, transition commands from the FS-CT6B mode switch are blocked unless:
*   RC throttle is completely at zero.
*   Motors are disarmed.
*   MPU6050 confirms that the drone is level (tilt < $\pm 5$ degrees).

---

## 🏷️ Secondary Variant: RFID-Authenticated Cargo Quadcopter
Located in `firmware/f450_rfid_delivery/`, this project represents a quadcopter with secure cargo box locking:
*   **RFID Lockout:** Uses an **MFRC522** RFID reader to scan keycards. The cargo container (locked via an SG90 servo) will unlock only when an authorized UID tag is read.
*   **Buzzer Cues:** Unique audio patterns indicate card validation success, failure, and lock timeouts.
*   **Pin Mapping (F450 RFID Variant):**
    *   **SPI (SCK/MISO/MOSI):** D13 / D12 / D11 (RFID connection)
    *   **RFID SS / RST:** A1 / D8
    *   **SG90 Servo:** D7
    *   **Piezo Buzzer:** A0
    *   **Propulsion ESCs:** D9, D10, D5, D6
    *   **Receiver Input Interrupts:** D2 (Roll), D3 (Pitch), D4 (Throttle)

### 📸 F450 Prototype Construction
Below are photos of the completed RFID-authenticated F450 quadcopter prototype:

#### 1. Secure Container Box Locking & RFID Swipe Antenna
![RFID Card Swiping Lock Check](images/img 1.jpeg)

#### 2. Cargo Box Container Mounted Underneath Frame
![Secure Container Box Mounting](images/img 2.jpeg)

#### 3. Full Prototype Assembly and RC Transmitter
![F450 Medical Cargo Drone Prototype](images/img 4.jpeg)

---

## 📂 Repository Structure

The code and assets are organized as follows:

*   **`firmware/`**
    *   `convertible_drone_mvp/` - Main source code for the Convertible flight + roll drone.
    *   `f450_rfid_delivery/` - Main source code for the RFID-authenticated cargo delivery quadcopter.
    *   `archive/` - Legacy versions of the F450 flight controllers.
*   **`tests_and_calibration/`** - Module calibration and bring-up scripts:
    *   `esc_calibration/` - ESC endpoint settings and throttle response testers.
    *   `mpu_test/` - MPU6050 reading and PID auto-calibration.
    *   `receiver_test/` - Captures and prints FS-CT6B receiver channel widths.
    *   `rfid_test/` - RFID scanner checks and basic card triggers.
    *   `servo_test/` - Actuates limit tests on transition servos.
    *   `motor_test/` - Spawns motor spin thresholds tests.
    *   `tuning/` - Calibration targets for the flight PID.
*   **`presentation/`** - Deliverables for the Applied Electromechanics presentation:
    *   `slides.html` - Premium interactive light-themed presentation deck containing a **live PID Pitch Simulator** and **FS-CT6B joystick channel mapper**.
    *   `generate_pptx.py` - Script to generate the native PowerPoint file.
    *   `medicine_drone_presentation.pptx` - PowerPoint file.
*   **`docs/`** - Extended text files explaining electromechanics calculations.
*   **`patent/`** - Drafts and generation scripts for the RFID cargo box patent application.

---
