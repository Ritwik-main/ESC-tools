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
- **ESC Calibration**: Guided step-by-step routine to calibrate new ESCs.
- **Stress Test**: High-load cycling to test ESC/Motor cooling and stability.
- **PPM Generator**: 12-channel PPM signal generation for flight controller testing.
- **Persistent Settings**: Save your min/max pulse and frequency preferences to EEPROM.

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
| **PWM Out**  | 3 | Timer 1 Driven |
| **PWM In**   | 2 | Interrupt Driven |
| **Button UP**| A1 | Pull-up enabled |
| **Button DN**| A0 | Pull-up enabled |
| **Button SEL**| A2 | Pull-up enabled |
| **Button BK** | A3 | Pull-up enabled |

**Note:** If you want to power the circuit with your BEC and not add a battery/USB power it, connect the 5V pin of the ESC to Vin of arduino.

<img width="3000" height="2367" alt="circuit_image" src="https://github.com/user-attachments/assets/c663d757-ee5a-47f5-ba49-9e20aff8d1c1" />
Designed with cirkitdesigner IDE

## 🚀 Installation

1. Install the following libraries in the Arduino IDE:
   - `Adafruit GFX Library`
   - `Adafruit SSD1306`
2. Open `ESC_TOOL.ino` in the Arduino IDE.
3. Select your board (e.g., Arduino Nano) and port.
4. Click **Upload**.

## 📖 Usage

- **Navigation**: Use **UP** and **DOWN** buttons to scroll through the menu.
- **Select**: Press **SEL** to enter a mode or change a setting.
- **Back**: Press **BACK** to stop the current mode and return to the main menu.
- **Safety**: Throttle is automatically reset to 0% (Min Pulse) when exiting any live output mode.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

This project is licensed under the MIT License


## Made with 🫶 using Google Antigravity

https://antigravity.google/


---
## 🚀 Roadmap / To-Do
- [x] Initial project setup
- [x] Implement core API logic
- [ ] implement battery pack using tp4056 and mt3608
- [ ] Add hardware demo and pictures
- [ ] Add catchy banner maybe
- [ ] Add hardware reference sites
- [x] Write documentation
