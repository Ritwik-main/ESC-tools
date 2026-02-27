# ESC Tool - Technical Documentation

This document provides a deep dive into the technical implementation and features of the **Arduino ESC Debugger & Signal Tool**.

---

## 🚀 Core Technologies

### 1. High-Precision PWM (Timer 1)
Most Arduino projects use the `Servo` library, which relies on software-timed interrupts that can jitter. This tool uses **Hardware Timer 1** on the ATmega328P for professional-grade stability.
- **Resolution:** 0.5µs (16MHz clock with 1/8 prescaler).
- **Mechanism:** Uses CTC (Clear Timer on Compare) mode with dual compare registers:
  - `OCR1A`: Defines the total period (Frequency).
  - `OCR1B`: Defines the pulse width (Duty Cycle).
- **Result:** A rock-solid signal up to 1000Hz, essential for high-performance BLDC ESCs (DShot/OneShot capable hardware).

### 2. Signal Analyzer Logic
The **PWM Reader** mode functions like a mini logic analyzer.
- **Measurement:** Uses Pin 2 with a hardware interrupt (`CHANGE`) to capture microsecond timestamps of rising and falling edges.
- **Metrics:** 
  - **Jitter Tracking:** Calculates the delta between min/max observed pulses.
  - **Frame Drop Detection:** Detects missed pulses if the signal period exceeds 2x the expected duration.
- **Visualizer:** A real-time waveform "scope" is rendered on the OLED to visualize the duty cycle.

### 3. PPM Frame Generation
The **PPM Generator** outputs a standard 12-channel radio stream.
- **Logic:** A state-machine inside the Timer 1 interrupt toggles the signal and dynamically modifies the next "compare match" value.
- **Frame Sync:** Ensures a standard 27ms total frame length regardless of individual channel widths.

### 4. PPM Reader Logic
The **PPM Reader** allows reading individual channels from a Radio Receiver.
- **Mechanism:** Also uses Pin 2, triggering on `RISING` edges.
- **Sync:** Detects the long gap (>3ms) to reset the channel index for each frame.
- **Visuals:** Shows up to 8 channels simultaneously with numerical values and mini status bars.
- **Refresh Rate:** Tracks the signal refresh rate in Hz.

---

## 🛠 Feature Breakdown

### 🔹 Manual PWM Control
The primary mode for testing ESCs or servos manually.
- **Operation:** Uses the Up/Down buttons to adjust throttle.
- **Safety:** Always initializes at the user-defined `Min Pulse` (typically 1000us) to prevent accidental motor spin-up.

### 🔹 Step Throttle
Designed for quick response testing.
- **Increments:** 100µs steps.
- **Benefit:** Quickly verifies if an ESC has linear power delivery or "dead zones" in the throttle range.

### 🔹 Auto Sweep
Automated ramping for stress testing and burn-in.
- **Logic:** Smoothly cycles from `Min` to `Max` and back.
- **Use Case:** Checking for vibration issues at specific RPMs or thermal performance under varying loads.

### 🔹 Stress Test (Jump Mode)
The "Hammer" test for ESCs.
- **Logic:** Instantly jumps between 0% and 100% throttle every 1 second.
- **Use Case:** Testing the ESC's current limiting and timing advance handling during rapid acceleration.

### 🔹 ESC Calibration Wizard
Simplifies the often-confusing beep-sequence of ESC calibration.
- **Step 1:** Sends High signal -> User powers on ESC.
- **Step 2:** User hears "High-point" confirmed beeps.
- **Step 3:** Sends Low signal -> Calibration complete.

---

## 🔌 Hardware Configuration

| Component | Pin |
| :--- | :--- |
| **PWM Output** | Pin 3 |
| **PWM Input** | Pin 2 |

---

## 💾 Settings & Persistence
User settings are stored in **EEPROM** using a magic-byte verification (`0x53455431`). This ensures that your custom PWM frequency and pulse limits are remembered even after the device is turned off.

> [!IMPORTANT]
> **Safety First:** The tool includes a "Global Reset" feature. Pressing the **BACK** button in any active mode immediately cuts the signal to 0% and stops the PWM generator.


