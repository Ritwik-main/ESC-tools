# ESC Tool - Technical Documentation

This document provides a deep dive into the technical implementation and features of the **Arduino ESC Debugger & Signal Tool**.

---

## 🚀 Core Technologies

### 1. High-Precision PWM (Timer 1)
Most Arduino projects use the `Servo` library, which relies on software-timed interrupts that can jitter. This tool uses **Hardware Timer 1** on the ATmega328P for professional-grade stability.
- **Resolution:** 0.5µs (16MHz clock with 1/8 prescaler).
- **Mechanism:** Uses CTC (Clear Timer on Compare) mode with dual compare registers:
  - `OCR1A`: Defines the total period (Frequency), and doubles as TOP.
  - `OCR1B`: Defines the pulse width (Duty Cycle).
- **Edges:** Both ISRs drive PD3 with direct port writes (`PORTD |= (1 << PD3)`).
  `digitalWrite()` costs ~4–6 µs on AVR — a PROGMEM pin lookup, a timer-PWM
  disable and a `cli/sei` pair — which is a significant fraction of a 2 ms
  period and lands as jitter on both edges.

#### Glitch-free register updates
`OCR1A` and `OCR1B` are **not double-buffered in CTC mode**. Writing them from
the main loop mid-cycle is unsafe:

- Lowering `OCR1B` while the counter has already passed the new value makes the
  hardware miss the compare match. The pulse never terminates that cycle and
  merges into the next one.
- Lowering `OCR1A` below the current `TCNT1` makes the counter run all the way
  to `0xFFFF` before wrapping — a single **32.7 ms** dead period.

The probability of hitting the first case is roughly the duty cycle, so the
corruption grows with output rate and is largely invisible at 50 Hz.

`updatePWMParams()` therefore only **stages** values into `pwmTop`/`pwmCompare`
and raises `pwmParamsDirty`. The `TIMER1_COMPA_vect` ISR commits them at TOP,
where the counter has just reset to zero and no write can be missed.

#### Rate ceiling
A pulse must fit inside its period with the line returning low in between, so:

```
max rate = 1,000,000 / (max pulse + PWM_GUARD_US)
```

With `PWM_GUARD_US = 50` and a 2000 µs max pulse, that is **~487 Hz**. Rates
above the ceiling are skipped in the Settings cycle and clamped in
`updatePWMParams()`, so the tool cannot emit a period it can't fill.

> Genuinely high-rate protocols (OneShot125, DShot) do not use servo-width
> pulses — OneShot125 scales the whole range to 125–250 µs. Reaching 1 kHz
> requires implementing that scaling, not just raising the frequency.

### 2. Signal Analyzer Logic
The **PWM Reader** mode functions like a mini logic analyzer.
- **Measurement:** Uses Pin 2 with a hardware interrupt (`CHANGE`) to capture microsecond timestamps of rising and falling edges.
- **Edge sampling:** The ISR reads `PIND` as its *first* statement, before
  calling `micros()`. Interrupt latency means a narrow pulse can already have
  flipped back by the time the handler runs; sampling late misclassifies the
  edge and corrupts the reading.
- **Metrics:**
  - **Jitter Tracking:** Calculates the delta between min/max observed pulses.
  - **Frame Drop Detection:** Compares each period against a running average of
    healthy periods (`nominalPeriod`, an 8-tap exponential average). A gap
    exceeding 1.5x nominal is scored as `period / nominal - 1` missed frames.
- **Snapshotting:** Interrupt-owned values are 2- and 4-byte quantities that an
  edge interrupt can land in the middle of. The display path copies them under
  a brief `cli()` so a half-updated value is never rendered.
- **Visualizer:** A real-time waveform "scope" is rendered on the OLED to visualize the duty cycle.

> Both reader modes attach their interrupt on entry and are torn down by the
> global **BACK** handler, which clears the shared `readerAttached` flag so the
> mode re-arms correctly on every subsequent entry.

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

### 🔹 Noisy Signal
A specialized mode for testing the robustness of an ESC's signal processing.
- **Jitter:** Introduces a random ±10µs variation to each pulse width.
- **Drop Outs:** Randomly skips 5% of frames (signal held low) to simulate poor connection or electromagnetic interference.
- **Use Case:** Validating if your ESC handles "dirty" signals gracefully or if it enters failsafe unnecessarily.

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

On load, the stored rate is passed through `clampFrequencyToProtocol()`. A unit
that previously had 600 Hz or 1000 Hz saved in EEPROM is silently corrected down
to the highest valid option (400 Hz at a 2000 µs max pulse) rather than booting
into a rate it cannot emit.

## 📦 Resource Budget

| Target | Flash | RAM |
| :--- | :--- | :--- |
| Arduino Uno | 28,170 / 32,256 B (87%) | 815 / 2048 B (39%) |
| Arduino Nano | 28,170 / 30,720 B (91%) | 815 / 2048 B (39%) |

Flash is the binding constraint on a 328P. The menu and label strings are the
largest remaining reclaimable block — moving `menuItems[]` into PROGMEM would
free roughly a few hundred bytes if a future feature needs the room.

> [!IMPORTANT]
> **Safety First:** The tool includes a "Global Reset" feature. Pressing the **BACK** button in any active mode immediately cuts the signal to 0% and stops the PWM generator.


