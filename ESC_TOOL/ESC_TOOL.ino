#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// Screen Dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Pin Definitions for SPI OLED
#define OLED_DC     9
#define OLED_CS     10
#define OLED_RESET  8

// Button Pins (Active-LOW, connected to GND)
#define BTN_UP      A0
#define BTN_DOWN    A1
#define BTN_SEL     A2
#define BTN_BACK    A3

// PWM Output Pin
#define PWM_OUT_PIN 3   // PD3
#define PWM_IN_PIN  2   // PD2

// Direct port access for the timer ISRs. digitalWrite() costs ~4-6us per call
// (PROGMEM pin lookup + timer disable + cli/sei), which is a large slice of a
// 1ms period and shows up as edge jitter on a scope. These are 1-2 cycles.
#define PWM_PIN_HIGH()  (PORTD |=  (1 << PD3))
#define PWM_PIN_LOW()   (PORTD &= ~(1 << PD3))
#define PWM_IN_LEVEL()  (PIND & (1 << PD2))

// Timer 1 runs at 2MHz (16MHz / 8), so 1 tick = 0.5us.
#define TICKS_PER_US 2

// The output must return low between pulses, so the period always has to be
// longer than the pulse. This guard is the minimum low time we insist on; it
// also sets the highest frequency a given Max Pulse can legally run at.
#define PWM_GUARD_US    50
#define PWM_GUARD_TICKS (PWM_GUARD_US * TICKS_PER_US)

// Timer 1 TOP is 16-bit, so 2000000/65536 is the slowest achievable rate.
#define PWM_MIN_HZ 31

// Noisy Signal mode parameters
#define NOISE_DROP_PERCENT 5
#define NOISE_JITTER_TICKS (10 * TICKS_PER_US)  // +/- 10us

// Display Object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS);

// Custom PWM Variables (Timer 1 driven)
volatile unsigned int pwmTop = 39999;     // TOP for freq (Default 50Hz)
volatile unsigned int pwmCompare = 2000;  // Pulse width in ticks
volatile bool pwmOutputEnabled = false;
// OCR1A/OCR1B are not double-buffered in CTC mode. Writing them mid-cycle can
// miss a compare match and produce a runt or a merged double-width pulse, so
// the main loop only stages values here and the COMPA ISR commits them at TOP.
volatile bool pwmParamsDirty = false;

int currentThrottle = 1000;

// PWM Reader Variables
volatile unsigned long pulseStart = 0;
volatile unsigned long lastRise = 0;
volatile unsigned long pulsePeriod = 0;
volatile unsigned long nominalPeriod = 0; // Running average, used to spot gaps
volatile int pulseInValue = 0;

int minObserved = 3000;
int maxObserved = 0;
float signalFreq = 0;

// Dropped Frame Tracking
volatile int droppedFrameCount = 0;
int droppedFPS = 0;

// Set while an edge interrupt is hooked to PWM_IN_PIN. File scope so the global
// BACK handler can tear the reader down and let it re-arm on the next entry.
bool readerAttached = false;

// PPM Reader Variables
volatile int ppmInValues[12];
volatile byte ppmInChannel = 0;
volatile unsigned long lastPpmTime = 0;
volatile unsigned int ppmFrameCount = 0;

// Sweep Variables
bool sweepDirection = true;
unsigned long lastSweepUpdate = 0;
unsigned long sweepInterval = 30; // ms

// System States
enum SystemState {
  SPLASH,
  MAIN_MENU,
  MANUAL_PWM,
  STEP_THROTTLE,
  AUTO_SWEEP,
  PWM_READER,
  PPM_READER,
  STRESS_TEST,
  CALIBRATION,
  SETTINGS,
  PPM_GENERATOR,
  NOISY_PWM
};

volatile SystemState currentState = SPLASH; // Read inside the Timer 1 ISRs
unsigned long stateTimer = 0;

// Menu Variables
int menuIndex = 0;
int menuScrollOffset = 0;
const int menuItemsCount = 10;
const int visibleItemsCount = 5; 
const char* menuItems[] = {
  "Manual PWM",
  "Step Throttle",
  "Auto Sweep",
  "Noisy Signal",
  "PWM Reader",
  "PPM Reader",
  "Stress Test",
  "ESC Calibration",
  "PPM Generator",
  "Settings"
};

// Button Logic
unsigned long lastButtonPress = 0;
const int debounceDelay = 200;

// EEPROM / Settings
struct Settings {
  uint32_t magic;
  int minPulse;
  int maxPulse;
  int frequency;
};

Settings sysSettings;
const uint32_t SETTINGS_MAGIC = 0x53455431; // "SET1"
int settingsIndex = 0;

// Selectable output rates. Entries that cannot fit the current Max Pulse plus
// the low-time guard are skipped, so the tool never offers a rate that would
// silently clip the pulse (1000Hz cannot carry a 2000us pulse at all).
const int freqOptions[] = {50, 60, 100, 200, 300, 400, 490};
const int freqOptionCount = sizeof(freqOptions) / sizeof(freqOptions[0]);

// Calibration States
enum CalibStep { CAL_HIGH, CAL_WAIT, CAL_LOW, CAL_DONE };
CalibStep currentCalStep = CAL_HIGH;

// PPM Generator Variables
int ppmValues[12] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
int currentPpmChan = 0;
volatile byte ppmPhase = 0;

void setup() {
  Serial.begin(115200);

  // Initialize PWM Output Pin
  pinMode(PWM_OUT_PIN, OUTPUT);
  digitalWrite(PWM_OUT_PIN, LOW);

  // Load Settings
  loadSettings();

  // Initialize display
  Serial.println(F("Initializing OLED..."));
  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  // Initialize Buttons (PULLUP = logic HIGH when open, LOW when pressed)
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SEL, INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);

  display.clearDisplay();
  // We'll let the loop handle the animated splash screen
  stateTimer = millis();
  
  // Setup Timer 1 for custom PWM (Last, to avoid interfering with boot/SPI)
  Serial.println(F("Initializing Timer1..."));
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 39999;           // 50Hz
  OCR1B = 2000;            // 1000us
  TCCR1B |= (1 << WGM12);  // CTC Mode
  TCCR1B |= (1 << CS11);   // Prescaler 8
  TIMSK1 |= (1 << OCIE1A);
  TIMSK1 |= (1 << OCIE1B);
  interrupts();
  
  stateTimer = millis();
  Serial.println(F("Setup Complete."));
}

void loop() {
  // Global Back Button Handling
  if (currentState != MAIN_MENU && currentState != SPLASH) {
    if (digitalRead(BTN_BACK) == LOW && (millis() - lastButtonPress > debounceDelay)) {
      if (currentState == PWM_READER || currentState == PPM_READER) {
        detachInterrupt(digitalPinToInterrupt(PWM_IN_PIN));
        readerAttached = false; // Let the reader re-arm on the next entry
      }
      if (currentState == SETTINGS) {
        clampFrequencyToProtocol();
        saveSettings();
      }
      pwmOutputEnabled = false; // Stop output
      currentThrottle = sysSettings.minPulse; // Reset throttle to 0%
      updatePWMParams(sysSettings.frequency, currentThrottle);
      
      currentState = MAIN_MENU;
      drawMenu();
      lastButtonPress = millis();
    }
  }

  switch (currentState) {
    case SPLASH: {
      unsigned long elapsed = millis() - stateTimer;
      if (elapsed > 2500) {
        Serial.println(F("Splash timeout - moving to Menu"));
        currentState = MAIN_MENU;
        drawMenu();
      } else {
        drawSplashScreen();
      }
      break;
    }

    case MAIN_MENU:
      handleMenuInput();
      break;

    case SETTINGS:
      handleSettings();
      break;
    
    case PPM_GENERATOR:
      handlePPMGenerator();
      break;

    case MANUAL_PWM:
      handleManualPWM();
      break;

    case STEP_THROTTLE:
      handleStepThrottle();
      break;

    case AUTO_SWEEP:
      handleAutoSweep();
      break;

    case NOISY_PWM:
      handleNoisyPWM();
      break;

    case PWM_READER:
      handlePWMReader();
      break;

    case PPM_READER:
      handlePPMReader();
      break;

    case STRESS_TEST:
      handleStressTest();
      break;

    case CALIBRATION:
      handleCalibration();
      break;

    default:
      // Basic placeholder for implemented modes
      if (millis() - stateTimer > 500) { // Slight delay to prevent immediate flickering
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.println(F("----  MODE  ----"));
        display.println(menuItems[menuIndex]);
        display.println(F("\n[BACK] to Exit"));
        display.display();
      }
      break;
  }
}

void drawSplashScreen() {
  display.clearDisplay();
  
  // --- Screen Border ---
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);

  // --- Title and PWM Wave ---
  display.setTextColor(SSD1306_WHITE);
  
  // "ESC Tools" in size 2 (108 pixels wide)
  int titleX = 5; 
  display.setTextSize(2);
  display.setCursor(titleX, 12);
  display.print(F("ESC Tools"));
  
  // PWM Wave icon next to title
  // Text ends at titleX + 108 = 113. Wave at 115.
  int waveX = 115;
  int waveY = 16;
  display.drawLine(waveX,     waveY+8, waveX+2, waveY+8, SSD1306_WHITE); // _
  display.drawLine(waveX+2,   waveY+8, waveX+2, waveY,   SSD1306_WHITE); // |
  display.drawLine(waveX+2,   waveY,   waveX+6, waveY,   SSD1306_WHITE); // -
  display.drawLine(waveX+6,   waveY,   waveX+6, waveY+8, SSD1306_WHITE); // |
  display.drawLine(waveX+6,   waveY+8, waveX+8, waveY+8, SSD1306_WHITE); // _

  // --- Version ---
  display.setTextSize(1);
  display.setCursor(31, 34);
  display.print(F("Version 1.0"));

  // --- Author ---
  display.setCursor(10, 48);
  display.print(F("By Ritwik Aggarwal"));

  display.display();
}

void drawMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  
  // Header
  display.setCursor(0, 0);
  display.println(F("----  MAIN MENU  ----"));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  // Items
  for (int i = 0; i < visibleItemsCount; i++) {
    int itemIdx = menuScrollOffset + i;
    if (itemIdx >= menuItemsCount) break;

    if (itemIdx == menuIndex) {
      display.fillRect(0, 12 + (i * 10), 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(5, 13 + (i * 10));
    display.print(menuItems[itemIdx]);
  }
  
  // Scroll Indicator (Subtle line if more items above/below)
  if (menuScrollOffset > 0) display.fillTriangle(124, 12, 122, 14, 126, 14, SSD1306_WHITE);
  if (menuScrollOffset + visibleItemsCount < menuItemsCount) display.fillTriangle(124, 62, 122, 60, 126, 60, SSD1306_WHITE);

  display.setTextColor(SSD1306_WHITE);
  display.display();
}

void handleMenuInput() {
  if (millis() - lastButtonPress < debounceDelay) return;

  if (digitalRead(BTN_UP) == LOW) {
    menuIndex--;
    if (menuIndex < 0) {
      menuIndex = menuItemsCount - 1;
      menuScrollOffset = menuItemsCount - visibleItemsCount;
    }
    
    // Update scroll window
    if (menuIndex < menuScrollOffset) {
      menuScrollOffset = menuIndex;
    }
    
    drawMenu();
    lastButtonPress = millis();
  }
  else if (digitalRead(BTN_DOWN) == LOW) {
    menuIndex++;
    if (menuIndex >= menuItemsCount) {
      menuIndex = 0;
      menuScrollOffset = 0;
    }

    // Update scroll window
    if (menuIndex >= menuScrollOffset + visibleItemsCount) {
      menuScrollOffset = menuIndex - visibleItemsCount + 1;
    }

    drawMenu();
    lastButtonPress = millis();
  }
  else if (digitalRead(BTN_SEL) == LOW) {
    // Transition to the selected mode
    switch(menuIndex) {
      case 0: currentState = MANUAL_PWM; break;
      case 1: currentState = STEP_THROTTLE; break;
      case 2: currentState = AUTO_SWEEP; break;
      case 3: currentState = NOISY_PWM; break;
      case 4: currentState = PWM_READER; break;
      case 5: currentState = PPM_READER; break;
      case 6: currentState = STRESS_TEST; break;
      case 7: currentState = CALIBRATION; break;
      case 8: currentState = PPM_GENERATOR; break;
      case 9: currentState = SETTINGS; break;
    }
    display.clearDisplay();
    display.setCursor(0,0);
    display.print(F("Entering: "));
    display.println(menuItems[menuIndex]);
    display.display();
    stateTimer = millis();
    lastButtonPress = millis();
  }
}

void loadSettings() {
  EEPROM.get(0, sysSettings);
  if (sysSettings.magic != SETTINGS_MAGIC || sysSettings.minPulse < 500 || sysSettings.maxPulse > 2500 || sysSettings.frequency < 10) {
    Serial.println(F("Restoring default settings..."));
    sysSettings.magic = SETTINGS_MAGIC;
    sysSettings.minPulse = 1000;
    sysSettings.maxPulse = 2000;
    sysSettings.frequency = 50;
    saveSettings();
  }
  // Guard against a stored rate that predates the protocol limit.
  clampFrequencyToProtocol();
}

void saveSettings() {
  EEPROM.put(0, sysSettings);
}

void handleSettings() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("----   OPTIONS   ----"));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  // Labels
  display.setCursor(5, 13);  display.print(F("Min Pulse:"));
  display.setCursor(5, 23);  display.print(F("Max Pulse:"));
  display.setCursor(5, 33);  display.print(F("Freq (Hz):"));

  // Values
  display.setCursor(80, 13); display.print(sysSettings.minPulse);
  display.setCursor(80, 23); display.print(sysSettings.maxPulse);
  display.setCursor(80, 33); display.print(sysSettings.frequency);

  // Selection Box
  int boxY = 12 + (settingsIndex * 10);
  display.drawRect(0, boxY, 128, 10, SSD1306_WHITE);

  // Show the ceiling the Max Pulse imposes, so a skipped rate is explainable.
  display.setCursor(5, 47);
  display.print(F("Max rate: "));
  display.print(maxSafeFrequency(sysSettings.maxPulse));
  display.print(F("Hz"));
  display.setCursor(5, 56);
  display.print(F("(pulse + 50us guard)"));

  display.display();

  if (millis() - lastButtonPress < debounceDelay) return;

  if (digitalRead(BTN_UP) == LOW) {
    settingsIndex = (settingsIndex - 1 + 3) % 3;
    lastButtonPress = millis();
  } else if (digitalRead(BTN_DOWN) == LOW) {
    settingsIndex = (settingsIndex + 1) % 3;
    lastButtonPress = millis();
  } else if (digitalRead(BTN_SEL) == LOW) {
    if (settingsIndex == 0) {
      if (sysSettings.minPulse == 1000) sysSettings.minPulse = 800;
      else if (sysSettings.minPulse == 800) sysSettings.minPulse = 1200;
      else sysSettings.minPulse = 1000;
    }
    else if (settingsIndex == 1) {
      if (sysSettings.maxPulse == 2000) sysSettings.maxPulse = 2200;
      else if (sysSettings.maxPulse == 2200) sysSettings.maxPulse = 1800;
      else sysSettings.maxPulse = 2000;
      // A longer pulse needs a longer period, so the rate may no longer fit.
      clampFrequencyToProtocol();
    }
    else if (settingsIndex == 2) {
      // Advance to the next rate that the current Max Pulse can actually carry.
      int maxHz = maxSafeFrequency(sysSettings.maxPulse);
      int idx = 0;
      for (int i = 0; i < freqOptionCount; i++) {
        if (freqOptions[i] == sysSettings.frequency) { idx = i; break; }
      }
      for (int n = 1; n <= freqOptionCount; n++) {
        int cand = freqOptions[(idx + n) % freqOptionCount];
        if (cand <= maxHz) { sysSettings.frequency = cand; break; }
      }
    }
    lastButtonPress = millis();
  }
}

void handleManualPWM() {
  if (!pwmOutputEnabled) {
    currentThrottle = sysSettings.minPulse;
    updatePWMParams(sysSettings.frequency, currentThrottle);
    pwmOutputEnabled = true;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("---- MANUAL  PWM ----"));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(10, 20);
  display.print(currentThrottle);
  display.print(F(" us"));
  
  // Percent display
  int percent = map(currentThrottle, sysSettings.minPulse, sysSettings.maxPulse, 0, 100);
  display.setCursor(10, 40);
  display.print(F("PWR: ")); display.print(percent); display.print(F("%"));

  display.display();

  updatePWMParams(sysSettings.frequency, currentThrottle);

  if (millis() - lastButtonPress < 30) return; // Faster response

  if (digitalRead(BTN_UP) == LOW) {
    currentThrottle += 5;
    if (currentThrottle > sysSettings.maxPulse) currentThrottle = sysSettings.maxPulse;
    lastButtonPress = millis();
  } else if (digitalRead(BTN_DOWN) == LOW) {
    currentThrottle -= 5;
    if (currentThrottle < sysSettings.minPulse) currentThrottle = sysSettings.minPulse;
    lastButtonPress = millis();
  }
}

void handleAutoSweep() {
  if (!pwmOutputEnabled) {
    currentThrottle = sysSettings.minPulse;
    updatePWMParams(sysSettings.frequency, currentThrottle);
    pwmOutputEnabled = true;
    sweepDirection = true;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("---- AUTO  SWEEP ----"));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(10, 20);
  display.print(currentThrottle);
  display.print(F(" us"));

  int percent = map(currentThrottle, sysSettings.minPulse, sysSettings.maxPulse, 0, 100);
  display.setCursor(10, 40);
  display.print(F("PWR: ")); display.print(percent); display.print(F("%"));
  
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print(sweepDirection ? F("Ramping UP...") : F("Ramping DOWN..."));
  display.display();

  updatePWMParams(sysSettings.frequency, currentThrottle);

  if (millis() - lastSweepUpdate > sweepInterval) {
    if (sweepDirection) {
      currentThrottle += 5;
      if (currentThrottle >= sysSettings.maxPulse) sweepDirection = false;
    } else {
      currentThrottle -= 5;
      if (currentThrottle <= sysSettings.minPulse) sweepDirection = true;
    }
    lastSweepUpdate = millis();
  }
}

void pwmMeasureISR() {
  // Sample the pin before anything else. micros() takes a few microseconds and
  // ISR entry can be delayed by another interrupt, so a narrow pulse may have
  // already flipped back by the time we get here - reading late misclassifies
  // the edge and corrupts the measurement.
  uint8_t level = PWM_IN_LEVEL();
  unsigned long now = micros();

  if (level) {
    if (lastRise > 0) {
      unsigned long period = now - lastRise;
      // Compare against the running nominal period, not the one we just
      // measured. The original compared period against 2x itself, which is
      // never true, so drops were never counted.
      if (nominalPeriod > 0 && period > nominalPeriod + (nominalPeriod >> 1)) {
        droppedFrameCount += (int)(period / nominalPeriod) - 1;
      } else if (nominalPeriod == 0) {
        nominalPeriod = period;
      } else {
        // Slow moving average, only fed by periods that look healthy.
        nominalPeriod = nominalPeriod - (nominalPeriod >> 3) + (period >> 3);
      }
      pulsePeriod = period;
    }
    lastRise = now;
    pulseStart = now;
  } else if (pulseStart > 0) {
    pulseInValue = (int)(now - pulseStart);
  }
}

void ppmMeasureISR() {
  unsigned long now = micros();
  unsigned long diff = now - lastPpmTime;
  lastPpmTime = now;
  
  if (diff > 3000) {
    ppmInChannel = 0;
    ppmFrameCount++;
  } else {
    if (ppmInChannel < 12) {
      ppmInValues[ppmInChannel] = (int)diff;
      ppmInChannel++;
    }
  }
}

void handlePWMReader() {
  if (!readerAttached) {
    pinMode(PWM_IN_PIN, INPUT_PULLUP);
    // Reset stats and history before arming, so a stale timestamp from a
    // previous session cannot be read as one enormous period.
    noInterrupts();
    lastRise = 0; pulseStart = 0; pulsePeriod = 0;
    nominalPeriod = 0; pulseInValue = 0; droppedFrameCount = 0;
    interrupts();
    minObserved = 3000; maxObserved = 0;
    signalFreq = 0;
    attachInterrupt(digitalPinToInterrupt(PWM_IN_PIN), pwmMeasureISR, CHANGE);
    readerAttached = true;
  }

  // Snapshot the ISR-owned values together. These are 2- and 4-byte reads that
  // an edge interrupt can land in the middle of, yielding a half-updated value.
  uint8_t sreg = SREG;
  cli();
  int pulseUs = pulseInValue;
  unsigned long periodUs = pulsePeriod;
  int drops = droppedFrameCount;
  SREG = sreg;

  // Calculate Stats
  if (pulseUs > 0) {
    if (pulseUs < minObserved) minObserved = pulseUs;
    if (pulseUs > maxObserved) maxObserved = pulseUs;
  }
  if (periodUs > 0) signalFreq = 1000000.0 / periodUs;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("---SIGNAL ANALYZER---"));
  
  // Pulse & Freq
  display.setCursor(0, 12);
  display.print(pulseUs); display.print(F("us "));
  display.print(signalFreq, 1); display.print(F("Hz"));

  // Statistics
  display.setCursor(0, 22);
  display.print(F("Min:")); display.print(minObserved);
  display.print(F(" Max:")); display.print(maxObserved);
  display.print(F(" \nJit:")); display.print(maxObserved - minObserved);
  display.print(F(" Drp:")); display.print(drops);

  // --- Mini Scope (Logic Analyzer Style) ---
  // Draw a clean baseline
  display.drawLine(0, 63, 127, 63, SSD1306_WHITE);

  // Calculate pulse width visually
  int highWidth = map(pulseUs, 800, 2200, 2, 120);
  if (highWidth < 2) highWidth = 2; // Visibility
  if (highWidth > 120) highWidth = 120;

  // Draw the pulse as a solid block (high-end aesthetic)
  display.fillRect(4, 44, highWidth, 19, SSD1306_WHITE);
  
  // Add a small "trigger" marker at the start
  display.drawLine(4, 40, 4, 63, SSD1306_WHITE);
  
  // Subtle dots for the remaining period
  for (int x = 4 + highWidth; x < 124; x += 4) {
    display.drawPixel(x, 63, SSD1306_WHITE);
  }

  display.display();

  if (digitalRead(BTN_SEL) == LOW) {
    minObserved = 3000; maxObserved = 0;
    noInterrupts();
    droppedFrameCount = 0;
    interrupts();
  }
  // Teardown lives in the global BACK handler in loop(); this function only
  // runs while currentState is PWM_READER, so a check for leaving it here
  // could never fire and the reader stayed dead on the second entry.
}

void handlePPMReader() {
  static unsigned long lastFpsCalc = 0;
  static unsigned int lastFrameCount = 0;

  if (!readerAttached) {
    pinMode(PWM_IN_PIN, INPUT_PULLUP);
    noInterrupts();
    for(int i=0; i<12; i++) ppmInValues[i] = 0;
    ppmInChannel = 0;
    lastPpmTime = micros();
    ppmFrameCount = 0;
    interrupts();
    lastFrameCount = 0;
    lastFpsCalc = millis();
    attachInterrupt(digitalPinToInterrupt(PWM_IN_PIN), ppmMeasureISR, RISING);
    readerAttached = true;
  }

  // Snapshot the ISR-owned channel data in one go.
  int chVals[8];
  unsigned int frames;
  uint8_t sreg = SREG;
  cli();
  for (int i = 0; i < 8; i++) chVals[i] = ppmInValues[i];
  frames = ppmFrameCount;
  SREG = sreg;

  // Calculate Refresh Rate
  if (millis() - lastFpsCalc >= 1000) {
    droppedFPS = (int)(frames - lastFrameCount);
    lastFrameCount = frames;
    lastFpsCalc = millis();
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("--- PPM ANALYZER ---"));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  // Show 8 channels with mini bars
  for (int i = 0; i < 8; i++) {
    int x = (i % 2) * 64;
    int y = 14 + (i / 2) * 12;
    
    display.setCursor(x, y);
    display.print(F("C")); display.print(i + 1); display.print(F(":"));
    display.print(chVals[i]);

    // Mini bar
    int barW = map(constrain(chVals[i], 800, 2200), 800, 2200, 0, 28);
    display.drawRect(x + 34, y + 1, 30, 5, SSD1306_WHITE);
    display.fillRect(x + 35, y + 2, barW, 3, SSD1306_WHITE);
  }

  // Monitor refresh rate
  display.setCursor(0, 56);
  display.print(F("RATE: ")); 
  display.print(droppedFPS);
  display.print(F(" Hz"));
  
  display.display();
  // Teardown is handled by the global BACK handler in loop().
}

// Timer 1 Interrupts for manual PWM on Pin 3
ISR(TIMER1_COMPA_vect) {
  if (currentState == PPM_GENERATOR) {
    static unsigned long usedTicks = 0;
    if (ppmPhase == 0) usedTicks = 0;

    if (ppmPhase % 2 == 0) {
      PWM_PIN_HIGH();
      OCR1A = 600; // 300us pulse
      usedTicks += 600;
    } else {
      PWM_PIN_LOW();
      int chan = ppmPhase / 2;
      if (chan < 12) {
        long ticks = ((long)ppmValues[chan] * TICKS_PER_US) - 600;
        if (ticks < 200) ticks = 200; // Never let a short channel underflow
        unsigned int valTicks = (unsigned int)ticks;
        OCR1A = valTicks;
        usedTicks += valTicks;
      } else {
        // Sync gap: constant 27ms frame total (54000 ticks)
        if (usedTicks < 54000) OCR1A = 54000 - usedTicks;
        else OCR1A = 2000; // Failsafe
      }
    }
    ppmPhase++;
    if (ppmPhase >= 26) ppmPhase = 0;
    return;
  }

  // TOP: the counter has just reset, so this is the one point in the cycle
  // where new period/width values can be loaded without missing a match.
  if (pwmParamsDirty) {
    OCR1A = pwmTop;
    OCR1B = pwmCompare;
    pwmParamsDirty = false;
  } else {
    OCR1B = pwmCompare; // Undo any jitter applied to the previous cycle
  }

  if (!pwmOutputEnabled) return; // Pin is already low and COMPB keeps it there

  if (currentState == NOISY_PWM) {
    // Dropped Frame Logic: skip the pulse entirely, leaving the line low.
    if (random(100) < NOISE_DROP_PERCENT) return;

    // Jitter Logic (+/- 10us), clamped so noise can never breach the guard.
    long width = (long)pwmCompare + random(-NOISE_JITTER_TICKS, NOISE_JITTER_TICKS + 1);
    long maxWidth = (long)pwmTop - PWM_GUARD_TICKS;
    if (width > maxWidth) width = maxWidth;
    if (width < 10) width = 10;
    OCR1B = (unsigned int)width;
  }

  // Cycle has finished (resets to 0). Start new pulse.
  PWM_PIN_HIGH();
}

ISR(TIMER1_COMPB_vect) {
  if (currentState == PPM_GENERATOR) return;
  // Pulse width reached. End pulse.
  PWM_PIN_LOW();
}

// Highest rate that can still carry maxPulseUs and return low in between.
// At the 2000us standard this is ~487Hz, which is why the option list stops
// at 490 and why 600/1000Hz were never valid for servo-protocol pulses.
int maxSafeFrequency(int maxPulseUs) {
  return (int)(1000000UL / (unsigned long)(maxPulseUs + PWM_GUARD_US));
}

// Pull the stored frequency down to something the current Max Pulse supports.
void clampFrequencyToProtocol() {
  int maxHz = maxSafeFrequency(sysSettings.maxPulse);
  if (sysSettings.frequency <= maxHz && sysSettings.frequency >= PWM_MIN_HZ) return;

  int best = freqOptions[0];
  for (int i = 0; i < freqOptionCount; i++) {
    if (freqOptions[i] <= maxHz) best = freqOptions[i];
  }
  sysSettings.frequency = best;
}

void updatePWMParams(int hz, int us) {
  // Never emit a rate the pulse width cannot fit inside.
  int maxHz = maxSafeFrequency(sysSettings.maxPulse);
  if (hz > maxHz) hz = maxHz;
  if (hz < PWM_MIN_HZ) hz = PWM_MIN_HZ;

  // 16MHz / 8 = 2MHz (0.5us per tick)
  unsigned long top = (2000000UL / (unsigned long)hz) - 1UL;
  if (top > 65535UL) top = 65535UL; // TOP is a 16-bit register

  unsigned long compare = (unsigned long)us * TICKS_PER_US;
  unsigned long maxCompare = (top > PWM_GUARD_TICKS) ? (top - PWM_GUARD_TICKS) : 10UL;
  if (compare > maxCompare) compare = maxCompare;
  if (compare < 10UL) compare = 10UL;

  // Stage only. The COMPA ISR loads these into OCR1A/OCR1B at TOP, where the
  // counter has just reset and a write can never be missed.
  uint8_t sreg = SREG;
  cli();
  pwmTop = (unsigned int)top;
  pwmCompare = (unsigned int)compare;
  pwmParamsDirty = true;
  SREG = sreg;
}

void handleCalibration() {
  if (!pwmOutputEnabled) {
    pwmOutputEnabled = true;
    updatePWMParams(sysSettings.frequency, sysSettings.maxPulse);
    currentCalStep = CAL_HIGH;
    stateTimer = millis();
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("---- CALIBRATION ----"));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setCursor(0, 20);
  switch (currentCalStep) {
    case CAL_HIGH:
      display.println(F("1. Power OFF ESC"));
      display.println(F("2. Press [SEL]"));
      updatePWMParams(sysSettings.frequency, sysSettings.maxPulse);
      break;
    case CAL_WAIT:
      display.println(F("3. Power ON ESC"));
      display.println(F("4. Wait for Beeps"));
      display.println(F("5. Press [SEL]"));
      break;
    case CAL_LOW:
      display.println(F("6. Setting LOW..."));
      updatePWMParams(sysSettings.frequency, sysSettings.minPulse);
      if (millis() - stateTimer > 2000) currentCalStep = CAL_DONE;
      break;
    case CAL_DONE:
      display.println(F("Calibration Done!"));
      display.println(F("Press [BACK] to exit"));
      break;
  }
  display.display();

  if (millis() - lastButtonPress < debounceDelay) return;

  if (digitalRead(BTN_SEL) == LOW) {
    if (currentCalStep == CAL_HIGH) currentCalStep = CAL_WAIT;
    else if (currentCalStep == CAL_WAIT) {
      currentCalStep = CAL_LOW;
      stateTimer = millis();
    }
    lastButtonPress = millis();
  }
}

void handleStepThrottle() {
  if (!pwmOutputEnabled) {
    currentThrottle = sysSettings.minPulse;
    updatePWMParams(sysSettings.frequency, currentThrottle);
    pwmOutputEnabled = true;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("--- STEP THROTTLE ---"));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(10, 20);
  display.print(currentThrottle);
  display.print(F(" us"));

  int percent = map(currentThrottle, sysSettings.minPulse, sysSettings.maxPulse, 0, 100);
  display.setCursor(10, 40);
  display.print(F("PWR: ")); display.print(percent); display.print(F("%"));

  // Progress Bar for steps
  int barWidth = map(currentThrottle, sysSettings.minPulse, sysSettings.maxPulse, 0, 126);
  display.drawRect(0, 54, 128, 8, SSD1306_WHITE);
  display.fillRect(1, 55, barWidth, 6, SSD1306_WHITE);

  display.display();

  updatePWMParams(sysSettings.frequency, currentThrottle);

  if (millis() - lastButtonPress < debounceDelay) return;

  if (digitalRead(BTN_UP) == LOW) {
    currentThrottle += 100;
    if (currentThrottle > sysSettings.maxPulse) currentThrottle = sysSettings.maxPulse;
    lastButtonPress = millis();
  } else if (digitalRead(BTN_DOWN) == LOW) {
    currentThrottle -= 100;
    if (currentThrottle < sysSettings.minPulse) currentThrottle = sysSettings.minPulse;
    lastButtonPress = millis();
  }
}

void handleStressTest() {
  static unsigned long lastMoveTime = 0;
  static bool targetHigh = true;
  const int pauseTime = 1000; // 1 second pause at each end

  if (!pwmOutputEnabled) {
    pwmOutputEnabled = true;
    currentThrottle = sysSettings.minPulse;
    updatePWMParams(sysSettings.frequency, currentThrottle);
    lastMoveTime = millis();
    targetHigh = true;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("---- STRESS TEST ----"));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(10, 20);
  display.print(currentThrottle);
  display.print(F(" us"));

  int percent = map(currentThrottle, sysSettings.minPulse, sysSettings.maxPulse, 0, 100);
  display.setCursor(10, 40);
  display.print(F("PWR: ")); display.print(percent); display.print(F("%"));

  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print(targetHigh ? F("State: AT LOW") : F("State: AT HIGH"));
  display.display();

  if (millis() - lastMoveTime > pauseTime) {
    if (targetHigh) {
      currentThrottle = sysSettings.maxPulse;
      targetHigh = false;
    } else {
      currentThrottle = sysSettings.minPulse;
      targetHigh = true;
    }
    updatePWMParams(sysSettings.frequency, currentThrottle);
    lastMoveTime = millis();
  }
}

void handlePPMGenerator() {
  if (!pwmOutputEnabled) {
    pwmOutputEnabled = true;
    ppmPhase = 0;
    // Constrain all channels to current settings on entry
    for(int i=0; i<12; i++) {
        ppmValues[i] = constrain(ppmValues[i], sysSettings.minPulse, sysSettings.maxPulse);
    }
    noInterrupts();
    TCNT1 = 0;
    OCR1A = 600;
    interrupts();
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("--- PPM GENERATOR ---"));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setCursor(5, 15);
  display.print(F("CHANNEL: ")); 
  if ((currentPpmChan + 1) < 10) display.print(F("0"));
  display.print(currentPpmChan + 1);
  
  // Draw simple channel grid/indicator
  for(int i=0; i<12; i++) {
    int x = (i % 6) * 20;
    int y = 45 + (i / 6) * 8;
    if (i == currentPpmChan) display.drawRect(x, y, 18, 7, SSD1306_WHITE);
    else display.drawPixel(x+9, y+3, SSD1306_WHITE);
  }

  display.setTextSize(2);
  display.setCursor(10, 25);
  display.print(ppmValues[currentPpmChan]);
  display.print(F(" us"));

  display.display();

  // Handle channel switching (Slower)
  if (digitalRead(BTN_SEL) == LOW && (millis() - lastButtonPress > 200)) {
    currentPpmChan++;
    if (currentPpmChan >= 12) currentPpmChan = 0;
    lastButtonPress = millis();
  }

  if (millis() - lastButtonPress < 30) return; // Fast adjustment for PWM values

  // The PPM ISR reads these, and a 2-byte store is not atomic on AVR, so an
  // interrupt landing mid-write would emit a garbage channel width.
  if (digitalRead(BTN_UP) == LOW) {
    int v = ppmValues[currentPpmChan] + 5;
    if (v > sysSettings.maxPulse) v = sysSettings.maxPulse;
    noInterrupts(); ppmValues[currentPpmChan] = v; interrupts();
    lastButtonPress = millis();
  } else if (digitalRead(BTN_DOWN) == LOW) {
    int v = ppmValues[currentPpmChan] - 5;
    if (v < sysSettings.minPulse) v = sysSettings.minPulse;
    noInterrupts(); ppmValues[currentPpmChan] = v; interrupts();
    lastButtonPress = millis();
  }
}

void handleNoisyPWM() {
  if (!pwmOutputEnabled) {
    currentThrottle = sysSettings.minPulse;
    updatePWMParams(sysSettings.frequency, currentThrottle);
    pwmOutputEnabled = true;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("---- NOISY SIGNAL ----"));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(10, 20);
  display.print(currentThrottle);
  display.print(F(" us"));
  
  // Percent display
  int percent = map(currentThrottle, sysSettings.minPulse, sysSettings.maxPulse, 0, 100);
  display.setCursor(10, 40);
  display.print(F("PWR: ")); display.print(percent); display.print(F("%"));

  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print(F("Jitter:+/-10us Drp:5%"));

  display.display();

  updatePWMParams(sysSettings.frequency, currentThrottle);

  if (millis() - lastButtonPress < 30) return; // Faster response

  if (digitalRead(BTN_UP) == LOW) {
    currentThrottle += 5;
    if (currentThrottle > sysSettings.maxPulse) currentThrottle = sysSettings.maxPulse;
    lastButtonPress = millis();
  } else if (digitalRead(BTN_DOWN) == LOW) {
    currentThrottle -= 5;
    if (currentThrottle < sysSettings.minPulse) currentThrottle = sysSettings.minPulse;
    lastButtonPress = millis();
  }
}
