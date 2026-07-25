# ESC Tool - Technical Documentation

This document provides a deep dive into the technical implementation and features of the **Arduino ESC Debugger & Signal Tool**.

---

## 🚀 Core Technologies

### 1. High-Precision PWM (Timer 1)
Most Arduino projects use the `Servo` library, which relies on software-timed interrupts that can jitter. This tool uses **Hardware Timer 1** on the ATmega328P for professional-grade stability.
- **Resolution:** 0.5µs (16MHz clock with 1/8 prescaler) for standard servo
  widths; 0.0625µs (1/1 prescaler) under OneShot125 — see section 2.
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
max rate = 1,000,000 / (protocol max pulse + PWM_GUARD_US)
```

With `PWM_GUARD_US = 50` and a 2000 µs max pulse, that is **~487 Hz**. Rates
above the ceiling are skipped in the Settings cycle and clamped in
`updatePWMParams()`, so the tool cannot emit a period it can't fill.

### 2. OneShot125 (scaled protocol)

Reaching kilohertz rates is not a matter of raising the frequency — a 2000 µs
pulse does not fit in a 1000 µs period at any prescaler. OneShot125 instead
rescales the whole throttle range into **125–250 µs**, so a full-throttle pulse
fits in a 300 µs period and the ceiling moves to **~3333 Hz**.

#### Time base switching
At the standard 2 MHz base, a 125 µs span is only 250 ticks — coarse throttle
resolution. Selecting OneShot125 therefore switches Timer 1 to **prescaler ÷1**
(16 MHz, 0.0625 µs per tick), giving 2000 steps across the range:

| | Standard | OneShot125 |
| :--- | :--- | :--- |
| Clock select | `CS11` (÷8) | `CS10` (÷1) |
| Tick | 0.5 µs | 0.0625 µs |
| Steps across range | 2000 (1000–2000 µs) | 2000 (125–250 µs) |
| TOP floor | 31 Hz | 245 Hz |

`applyTimerBase()` rewrites `TCCR1B` while preserving `WGM12`, and is only
called with the output disabled — from `setup()`, from Settings, and on entry to
and exit from the PPM generator — so there is never a live waveform to glitch.

The 245 Hz floor is not a protocol choice: `65535 / 16 MHz` is 4.096 ms, so any
slower rate would overflow the 16-bit TOP. It is enforced by `protocolMinHz()`
rather than allowed to wrap silently.

#### Single-point rescaling
Every mode continues to drive `currentThrottle` in the settings' own
microsecond domain (typically 1000–2000 µs). The conversion happens only in
`updatePWMParams()`, where a width becomes timer ticks:

```
compare = isOneShot() ? oneShotTicks(us) : us * ticksPerUs
```

`oneShotTicks()` interpolates **in ticks, not microseconds**, so the result
keeps the full 0.0625 µs resolution instead of being rounded to a whole
microsecond first. The endpoints land exactly: 0% → 2000 ticks (125.000 µs),
50% → 3000 ticks (187.500 µs), 100% → 4000 ticks (250.000 µs).

Keeping the rescale in one place means no mode needs to know which protocol is
active — Step Throttle's 100 µs increments, Auto Sweep's ramp, the Stress Test's
endpoints and the Calibration wizard all work unchanged.

#### Interactions
- **PPM Generator** pins the prescaler to ÷8 on entry. Its 300 µs sync pulse and
  27 ms frame are written as literal 0.5 µs tick counts, and PPM is a separate
  standard that OneShot scaling does not apply to. The global BACK handler
  restores the protocol's own base.
- **Noisy Signal** defines its jitter as a fixed *tick* count. Because the tick
  scale and the pulse scale both shrink by 8×, 20 ticks stays ~1% of the pulse
  width in either protocol (10 µs of 1000 µs, 1.25 µs of 125 µs). The on-screen
  label follows the active protocol.
- **EEPROM** gained a `protocol` field, taking the struct from 10 to 12 bytes.
  The magic moved from `SET1` to `SET2`, because the old magic would still have
  matched and whatever followed the struct would have been read as a protocol.

> DShot is a different problem again — it is a digital bitstream, not a pulse
> width, and would need its own encoder rather than a prescaler change.

### 3. Signal Analyzer Logic
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

### 4. PPM Frame Generation
The **PPM Generator** outputs a standard 12-channel radio stream.
- **Logic:** A state-machine inside the Timer 1 interrupt toggles the signal and dynamically modifies the next "compare match" value.
- **Frame Sync:** Ensures a standard 27ms total frame length regardless of individual channel widths.

### 5. PPM Reader Logic
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
User settings are stored in **EEPROM** using a magic-byte verification
(`0x53455432`, "SET2"). This ensures that your custom PWM frequency, pulse
limits and protocol are remembered even after the device is turned off.

On load, the stored rate is passed through `clampFrequencyToProtocol()`. A unit
that previously had 600 Hz or 1000 Hz saved in EEPROM is silently corrected down
to the highest valid option (400 Hz at a 2000 µs max pulse) rather than booting
into a rate it cannot emit. The same function runs when the protocol changes,
since the ceiling and floor both move with it. Coming *up* from a slower
protocol it picks the gentlest valid rate rather than the ceiling, so switching
to OneShot125 lands on 250 Hz rather than dropping the ESC straight onto 3 kHz.

The stored `protocol` is validated against the enum on load; anything else falls
back to defaults along with the rest of the struct.

## 📦 Resource Budget

| Target | Flash | RAM |
| :--- | :--- | :--- |
| Arduino Uno | 27,692 / 32,256 B (85%) | 529 / 2048 B (25%) |
| Arduino Nano | 27,692 / 30,720 B (90%) | 529 / 2048 B (25%) |

Flash is the binding constraint on a 328P. How the current figure was reached,
measured on `arduino:avr:nano`:

| Change | Flash | RAM |
| :--- | :--- | :--- |
| Baseline | 28,170 (91%) | 815 (39%) |
| Serial debug removed | 26,820 (87%) | 640 (31%) |
| OneShot125 added | 27,644 (89%) | 654 (31%) |
| `menuItems[]` → PROGMEM | 27,692 (90%) | 529 (25%) |

Two things worth recording, because both contradict what this document
previously claimed:

- **Removing `Serial` is where the flash actually is.** The seven debug prints
  pulled in the whole `HardwareSerial` machinery for **1,350 bytes** of flash and
  175 bytes of RAM. That is the single largest reclaimable block, and it is what
  paid for OneShot125.
- **Moving `menuItems[]` to PROGMEM does not reclaim flash.** It is a *RAM*
  optimisation: measured at **−125 bytes of SRAM but +48 bytes of flash**, since
  the strings already occupied flash as initialisers and the `strlcpy_P`
  accessor is new code. The earlier estimate of "a few hundred bytes" of flash
  was wrong in both magnitude and direction.

The cost of removing `Serial` is that a failed `display.begin()` has no way to
announce itself — the display is the only output channel, and pin 13 is SPI SCK,
so the built-in LED cannot be blinked without fighting the OLED bus. A dead
OLED therefore presents as a silent hang.

> [!IMPORTANT]
> **Safety First:** The tool includes a "Global Reset" feature. Pressing the **BACK** button in any active mode immediately cuts the signal to 0% and stops the PWM generator.


