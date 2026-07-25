# ESC Tools (v1.0) 🛠️

A comprehensive **ESC Debugger & Signal Tool** built using Arduino. This tool provides a powerful interface for testing, calibrating, and analyzing Electronic Speed Controllers (ESCs), Servo motors, Flight computers, Stepper motors, and all PWM Interfaces, all within a compact handheld device.

## ✨ Features

- **Manual PWM Control**: Fine-tune your ESC with precise pulse width control.
- **Step Throttle**: Quickly test range with 100µs increments.
- **Auto Sweep**: Automatically ramp throttle between min/max to find startup points or test durability.
- **Signal Analyzer**: Real-time PWM reader with:
  - Pulse width (µs) and Frequency (Hz) measurement.
  - Jitter analysis and Dropped Frame tracking.
  - **Mini Logic Analyzer**: Visual waveform representation on the OLED.
- **Noisy Signal**: Simulate real-world signal issues with programmable jitter (±10µs) and dropped frames (5%) to test ESC stability.
- **ESC Calibration**: Guided step-by-step routine to calibrate new ESCs.
- **Stress Test**: High-load cycling to test ESC/Motor cooling and stability.
- **PPM Generator**: 12-channel PPM signal generation for flight controller testing.
- **PPM Reader**: PPM channels reader to analyse and debug radio recievers.
- **Persistent Settings**: Save your min/max pulse and frequency preferences to EEPROM.

The major portion of this project was coded using Antigravity IDE. Feel free to comment, request edits/changes, pull requests, and give reviews.

## 📐 Signal Specifications

Output is generated from hardware Timer 1 at **0.5 µs resolution** (16 MHz / 8 prescaler).

| Parameter | Range |
|-----------|-------|
| Pulse width | 800 – 2200 µs (Min/Max configurable) |
| Output rate | 50, 60, 100, 200, 300, 400, 490 Hz |
| Resolution | 0.5 µs (1 timer tick) |
| Min low time | 50 µs guard between pulses |

### Why the rate tops out at 490 Hz

A pulse has to fit *inside* its own period, with the line returning low in
between. The maximum rate is therefore set by your **Max Pulse** setting:

```
max rate = 1,000,000 / (max pulse + 50 µs guard)
```

At the standard 2000 µs that works out to **~487 Hz**. Anything faster cannot
represent a full-throttle pulse at all — at 1000 Hz the entire period is
1000 µs, so a 2000 µs pulse simply does not exist.

The Settings screen shows the current ceiling, and rates above it are skipped
when cycling rather than being offered and then silently clipped. Raising Max
Pulse to 2200 µs lowers the ceiling to ~444 Hz and the rate is clamped down
automatically.

> [!NOTE]
> If you need rates above ~500 Hz, that requires a scaled protocol such as
> OneShot125 (125–250 µs), not a standard servo-width pulse. That is not
> implemented yet — see the roadmap.

## 🕹️ Hardware Setup

### Components
- **Microcontroller**: Arduino Nano (or compatible)
- **Display**: 128x64 SSD1306 SPI OLED
- **Controls**: 4x Push Buttons (Active-LOW)
- **Output**: ESC Signal Pin (Pin 3)
- **Input**: External PWM Source (Pin 2)

### Pin Mapping
| Component | Arduino Pin | Notes |
|-----------|-------------|-------|
| **OLED DC** | 9 | SPI Data/Command |
| **OLED CS** | 10 | SPI Chip Select |
| **OLED Reset**| 8 | OLED Reset Pin |
| **PWM Out**  | 3 | Timer 1 Driven (PD3) |
| **PWM In**   | 2 | Interrupt Driven (PD2) |
| **Button UP**| A0 | Pull-up enabled |
| **Button DN**| A1 | Pull-up enabled |
| **Button SEL**| A2 | Pull-up enabled |
| **Button BK** | A3 | Pull-up enabled |

**Note:** If you want to power the circuit with your BEC and not add a battery/USB power it, connect the 5V pin of the ESC to Vin of arduino.

<img width="3000" height="2367" alt="circuit_image" src="https://github.com/user-attachments/assets/c663d757-ee5a-47f5-ba49-9e20aff8d1c1" />
Designed with cirkitdesigner IDE

## 🌐 Hardware reference
1. Arduino Nano: https://robocraze.com/products/nano-development-board-compatible-with-arduino
2. Oled display: https://robu.in/product/0-96-oled-display-module
3. Buttons: https://robocraze.com/products/4-pins-dip-momentary-square-tactile-push-button-switch-10-pieces-6x6x5mm

## 🚀 Installation

1. Install the following libraries in the Arduino IDE:
   - `Adafruit GFX Library`
   - `Adafruit SSD1306`
2. Open `ESC_TOOL/ESC_TOOL.ino` in the Arduino IDE.
3. Select your board (e.g., Arduino Nano) and port.
4. Click **Upload**.

> [!WARNING]
> The sketch uses **~91% of flash on an Arduino Nano** (30,720 bytes) and ~87%
> on an Uno. It fits, but there is little room left for new features on a
> 328P — budget accordingly before adding modes.

> [!NOTE]
> ### Safety first
> Please ensure your actuators are firmly held or under mechanical control before you fire them up, and detach your propellers. 

## 📖 Usage

- **Navigation**: Use **UP** and **DOWN** buttons to scroll through the menu.
- **Select**: Press **SEL** to enter a mode or change a setting.
- **Back**: Press **BACK** to stop the current mode and return to the main menu.
- **Safety**: Throttle is automatically reset to 0% (Min Pulse) when exiting any live output mode.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

This project is licensed under the MIT License

---
## 🚀 Roadmap / To-Do
- [x] Initial project setup
- [x] Implement core API logic
- [ ] implement battery pack using tp4056 and mt3608
- [ ] Add hardware demo and pictures
- [ ] Add catchy banner maybe
- [x] Add hardware reference sites
- [x] Add PPM reader
- [x] Write documentation
- [x] Add Noisy Signal feature
- [x] Fix PWM timing glitches at high output rates
<<<<<<< HEAD
- [ ] Add OneShot125 / scaled-protocol support for rates above 500 Hz
- [ ] Move menu strings to PROGMEM to reclaim flash
=======
- [x] Add scope captures of the output waveforms
- [x] Remove Serial debug to reclaim flash (−1,350 B — this is where the flash was)
- [x] Add OneShot125 / scaled-protocol support for rates above 500 Hz *(arithmetic verified, awaiting scope confirmation)*
- [x] Move menu strings to PROGMEM — reclaims **RAM** (−125 B), not flash (+48 B)

## 🔧 Changelog

### OneShot125 and the flash budget

- **Added a Protocol setting** (Standard / OneShot125). OneShot switches Timer 1
  to prescaler ÷1 and rescales the throttle range to 125–250 µs, moving the rate
  ceiling from ~487 Hz to ~3333 Hz. The rescale happens at one point in
  `updatePWMParams()`, so every existing mode works unchanged.
- **Removed the `Serial` debug output**, reclaiming 1,350 bytes of flash. This is
  what made room for OneShot125; the sketch went from 91% to 87% before the new
  feature took it back to 90%.
- **Moved the menu strings to PROGMEM**, freeing 125 bytes of SRAM (39% → 25%
  overall). This was listed on the roadmap as a flash saving — it is not; it
  costs 48 bytes of flash and pays back in RAM.
- **Bumped the EEPROM magic to `SET2`.** The settings struct grew by the protocol
  field, so stored `SET1` data is now rejected and defaults restored.

### Signal integrity fixes

The output waveform was unstable above ~500 Hz. Root causes, all fixed:

- **600 Hz and 1000 Hz could not carry a servo pulse.** The period was shorter
  than the pulse, and a safety clamp quietly trimmed the width to
  `period − 5 µs`, producing a near-DC line with a 5 µs notch instead of PWM.
  Rates are now capped to what the configured Max Pulse can actually fit.
- **Compare registers were written mid-cycle.** `OCR1A`/`OCR1B` are not
  double-buffered in CTC mode, so an update landing after the counter had
  passed the new value missed the match — giving a merged double-width pulse,
  or a full 32.7 ms dead period on a frequency change. Values are now staged
  and committed inside the timer ISR at TOP, the only glitch-free point.
  The failure rate scaled with duty cycle, which is why it appeared at high
  rates first.
- **`digitalWrite()` in the timer ISRs** cost ~4–6 µs per edge. Replaced with
  direct port writes, removing that much jitter from every transition.
- **Signal Analyzer never counted dropped frames** (it compared a period
  against twice itself, which is never true) and could report garbage from
  torn multi-byte reads of interrupt-owned variables.
- **Both reader modes went dead after the first exit** — the interrupt was
  detached but the "attached" flag was never cleared, so re-entering showed
  frozen values.
- **PWM Reader force-drove the output** at a hard-coded 50 Hz sweep from a
  stray line of leftover debug code, ignoring your settings.
>>>>>>> parent of 9c02eea (Merge origin/main into oneshot125-and-flash-budget)
