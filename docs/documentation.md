# Medicine Delivery Quadcopter: Design & Avionics Documentation
This document outlines the mechanics, electronics, control logic, wiring layout, secure RFID locking system, and control firmware for the **Quadcopter Medicine Delivery Drone** developed as the final implementation of the **Applied Electromechanics Course Project**.

---

## 1. Project Overview & Architecture
The project delivers a secure, automated quadcopter designed to transport and release medical payloads. 

### Core Concept
*   **Aerial Platform:** A standard F450 quadcopter powered by an Arduino UNO running real-time flight stabilization and attitude control at 250 Hz.
*   **Security Delivery Box:** A cargo container attached to the bottom of the drone containing the medicine payload.
*   **Electronic Cargo Lock:** An electronic lock operated by an MG996R or MG90S high-torque servo. The box remains locked to secure the medicine during flight.
*   **RFID Authorization:** A bottom-mounted MFRC522 RFID reader scans passenger or doctor credentials. Scanning an authorized RFID tag triggers the cargo lock to open for exactly **10 seconds** before automatically relocking.
*   **Manual Overrides:** The pilot can override the lock remotely from the ground control station using a toggle switch on the transmitter.

---

## 2. Bill of Materials (BOM) & Components
The system is constructed with the following electromechanical components:

| Component | Description | Quantity | Purpose |
| :--- | :--- | :---: | :--- |
| **Arduino UNO** | ATmega328P Microcontroller | 1 | Flight Controller & System Brain. Executes the 250Hz stabilization loop, reads receiver inputs, processes RFID tags, and drives the ESCs/cargo servo. |
| **MPU6050** | 3-axis Accelerometer + Gyroscope | 1 | Attitude sensor. Provides real-time angular rates and acceleration for pitch/roll stabilization. |
| **MFRC522 RFID Module** | 13.56 MHz SPI RFID Transceiver | 1 | Security scanner. Reads UIDs of patient/doctor tag cards at the delivery location. |
| **MG9966 High-Torque Servo** | Lock Actuator Servo | 1 | Operates the medicine container latch (0° = Locked, 90° = Unlocked). |
| **1000kV BLDC Motors** | Brushless Outrunner Motors | 4 | Main quadcopter propulsion (X-configuration). |
| **1045 Propellers** | 10" diameter, 4.5" pitch props | 4 | Generates lift (2x CW, 2x CCW). |
| **30A ESCs** | Electronic Speed Controllers | 4 | Controls BLDC motor speed via high-frequency PWM commands from the Arduino. |
| **LiPo Battery (3S 11.1V)** | 2200mAh 30C Lithium Polymer | 1 | Main system power source. |
| **Battery Eliminator Circuit (BEC)**| 5V 3A Switching Regulator | 1 | Steps down battery voltage to a constant, clean 5V to power logic components (Arduino, MPU6050, RFID), filtering out high-frequency ESC EMF noise. |
| **Power Distribution Board (PDB)** | DIY Copper Bus PDB | 1 | Distributes raw battery power to the 4 ESCs and the BEC. |
| **FS-CT6B Tx/Rx** | 6-Channel 2.4GHz Transmitter/Receiver | 1 | Radio control link (Manual throttle, pitch, roll, yaw, and aux switches). |
| **Active Piezo Buzzer** | 5V Active Buzzer | 1 | Audio feedback for connectivity status, arming, disarming, and low battery alarms (optional). |

---

## 3. Electronics & Wiring Mappings
The final avionics configuration maps the Arduino UNO pins to flight control inputs, ESC outputs, SPI lines, and I2C lines:

| Arduino Pin | Connection / Channel | Direction | Function |
| :--- | :--- | :---: | :--- |
| **D2 (INT0)** | RX Channel 1 (Roll) | Input | Interrupt-driven pulse reading (External Interrupt 0) |
| **D3 (INT1)** | RX Channel 2 (Pitch) | Input | Interrupt-driven pulse reading (External Interrupt 1) |
| **D4** | RX Channel 3 (Throttle) | Input | Pin Change Interrupt (PCINT20 / Port D4) |
| **A3** | RX Channel 4 (Yaw) | Input | Pin Change Interrupt (PCINT11 / Port C3) |
| **A2** | RX Channel 5 (Lock Switch Override) | Input | Pin Change Interrupt (PCINT10 / Port C2) |
| **D5** | ESC Back-Right (BR) | Output | PWM motor command output |
| **D6** | ESC Back-Left (BL) | Output | PWM motor command output |
| **D9** | ESC Front-Left (FL) | Output | PWM motor command output (Timer1 Hardware PWM) |
| **D10** | ESC Front-Right (FR) | Output | PWM motor command output (Timer1 Hardware PWM) |
| **D7** | Cargo Lock Servo | Output | Angle commands (0° = Locked, 90° = Unlocked) |
| **D8** | RFID RST Pin | Output | MFRC522 Reset control line |
| **A1** | RFID SS / SDA Pin | Output | MFRC522 SPI Slave Select line |
| **D11** | RFID MOSI | Output | SPI Master Out Slave In |
| **D12** | RFID MISO | Input | SPI Master In Slave Out |
| **D13** | RFID SCK | Output | SPI Serial Clock |
| **A4** | MPU6050 SDA | Bidirectional | I2C Serial Data (400 kHz clock rate) |
| **A5** | MPU6050 SCL | Output | I2C Serial Clock (400 kHz clock rate) |

---

## 4. Flight Stabilization & Control Algorithms
Stabilizing a quadcopter on a single 16MHz ATmega328P chip requires highly efficient filtering and PID control loops:

### 4.1 Sensor Filtering & Damping
*   **Vibration Isolator:** High-frequency motor vibrations corrupt the accelerometers. The MPU6050 is physically mounted on a soft sponge dampening pad.
*   **Low-Pass Filter on Accelerometer:** Reduces high-frequency sensor noise before calculating gravity vectors:
    $$\text{Angle}_{filt} = 0.90 \times \text{Angle}_{prev} + 0.10 \times \text{Angle}_{raw}$$
*   **Complementary Filter:** Combines high-speed gyroscope rate integration (responsive, but drifts over time) with gravity-referenced accelerometer tilt angles (stable, but noisy):
    $$\theta_{\text{pitch}} = 0.98 \times (\theta_{\text{pitch}} + \omega_y \cdot dt) + 0.02 \times \text{AccPitch}_{filt}$$
    $$\theta_{\text{roll}} = 0.98 \times (\theta_{\text{roll}} + \omega_x \cdot dt) + 0.02 \times \text{AccRoll}_{filt}$$
*   **Derivative Filter:** Smooths the D-term inputs in the PID loop to prevent high-frequency noise from causing motor chatter:
    $$\text{Deriv}_{filt} = 0.70 \times \text{Deriv}_{prev} + 0.30 \times \text{Deriv}_{raw}$$

### 4.2 Cascaded PID Stabilization
The flight controller computes separate correction values for Roll, Pitch, and Yaw:
$$\text{Output}_{\text{PID}} = K_p \cdot e(t) + K_i \cdot \int_{0}^{t} e(\tau)d\tau + K_d \cdot \frac{de(t)}{dt}$$

*   **Roll/Pitch Gains:** $K_p = 1.4$, $K_i = 0.02$, $K_d = 3.5$
*   **Yaw Gains:** $K_p = 2.5$, $K_i = 0.06$, $K_d = 0.0$ (Yaw is controlled on rate feedback).
*   **Windup Guard:** Integral accumulation is clamped to $\pm 400$ to prevent control lag during sudden corrections.

### 4.3 Motor Output Mixing
The PID outputs are mixed with the base throttle commands and mapped to the standard quadcopter **X-configuration**:
```
      FL (CW)      FR (CCW)
         \        /
          [ Drone ]
         /        \
      BL (CCW)     BR (CW)
```
*   **$\text{PWM}_{\text{FL}}$** $= \text{Throttle} + \text{Roll}_{\text{PID}} + \text{Pitch}_{\text{PID}} - \text{Yaw}_{\text{PID}}$
*   **$\text{PWM}_{\text{FR}}$** $= \text{Throttle} - \text{Roll}_{\text{PID}} + \text{Pitch}_{\text{PID}} + \text{Yaw}_{\text{PID}}$
*   **$\text{PWM}_{\text{BR}}$** $= \text{Throttle} - \text{Roll}_{\text{PID}} - \text{Pitch}_{\text{PID}} - \text{Yaw}_{\text{PID}}$
*   **$\text{PWM}_{\text{BL}}$** $= \text{Throttle} + \text{Roll}_{\text{PID}} - \text{Pitch}_{\text{PID}} + \text{Yaw}_{\text{PID}}$
*   Outputs are clamped between a minimum spin limit of `1100 µs` (to keep propellers spinning under load and avoid stuttering) and a maximum safety limit of `1900 µs`.

---

## 5. Medicine Cargo Locking Logic
The security container locking routine operates in the background of the stabilization loop:
*   **State Angles:** The servo is commanded to **0°** to engage the mechanical lock latch, and **90°** to release it.
*   **RFID Authorized Access:**
    *   The MFRC522 scans for 13.56 MHz passive tags.
    *   If the scanned card matches the pre-authorized UID: `{0x9D, 0xA4, 0x5F, 0x06}`, the cargo servo rotates to **90°** (Unlocked).
    *   The lock remains open for exactly **10 seconds (10,000 ms)**, tracked using non-blocking calls (`millis()`). Once the timer expires, the cargo servo automatically returns to **0°** (Locked).
*   **Transmitter Override:** The pilot can override the RFID card reader at any time by toggling Channel 5 (`A2` input). If the switch pulse is $> 1600 \text{ µs}$, the lock is forced open. Once toggled below $< 1600 \text{ µs}$, the locking system is locked and RFID card authority is restored.
*   **Safety Interlocking:** Unlocking servo commands are gated so they are only written upon a state change, preventing constant timer interrupts from introducing jitter into the ESC motor signals.

---

## 6. Firmware Optimizations (v2.2.0)
To run stabilization algorithms and RFID operations concurrently on an 8-bit AVR micro-architecture, the final code incorporates several low-level optimizations:
1.  **Register-Level Interrupt Capture:** Replaces blocking `pulseIn()` channel reads with Pin Change Interrupts (PCINT) and External Interrupts (INT0, INT1). Direct Port Register reading (`PIND & (1 << pin)`) replaces slow `digitalRead()` routines.
2.  **Inversion Multipliers:** Since division is computationally expensive on the Arduino ATmega328P, divisions are replaced with multiplication of pre-calculated floating-point constants:
    *   Gyro raw conversion: Multiplies by `INV_GYRO_SCALE` ($0.007633588$) instead of dividing by $131.0$.
    *   Stick normalizer: Multiplies by `INV_STICK` ($0.002$) instead of dividing by $500.0$.
3.  **Collapsed Redundant Equations:** Attitude calculation pre-calculates the squares of raw accelerometer data ($ax^2$, $ay^2$, $az^2$) once, collapsing both pitch and roll formulas into shared elements and saving 3 redundant multiplications per loop.
4.  **Inlined Output Constrain:** Constrain logic and float-to-int casting are inlined directly in the motor write routine, saving function overhead during the execution of `clampPWM()`.

---

## 7. Operating Instructions & Bring-up Procedures
Before launching the delivery drone, perform these safety checks:

### Bench Safe-Start Routine
1.  **Propellers OFF:** Remove all propellers before performing any indoor bench tests or connecting power.
2.  **Telemetry Review:** Open the Serial monitor at `115200` baud. Ensure raw sensor calculations read `0.0` when the drone is placed on a flat, level surface.
3.  **RFID Verification:** Scan the patient keycard. Confirm that the Serial monitor outputs `RFID: AUTHORISED - servo unlocked` and the servo latch swings to 90° for exactly 10 seconds.
4.  **Transmitter Override Test:** Toggle the AUX switch on the FS-CT6B transmitter. Confirm that it forces the cargo box open, and returns to locked when toggled off.

### Arming and Disarming Procedures
*   **Safe State:** Place the drone on the ground. Power on the transmitter first, then connect the 3S LiPo battery. The ESCs will emit a calibration chime.
*   **Arming:** Lower the throttle completely (CH3 < 1050 µs) and hold the Yaw stick to the far right (CH4 > 1800 µs) for 1 second. The flight controller will enable outputs and the monitor will output `>>> ARMED <<<`.
*   **Disarming:** Lower the throttle completely and hold the Yaw stick to the far left (CH4 < 1200 µs). Motor outputs will disarm immediately, and the monitor will output `>>> DISARMED <<<`.
*   **Safety Link Fail-safe:** If the receiver connection drops for more than **200 ms**, or if the pilot triggers the disarm switch, motor commands are immediately killed to prevent flyaways.
