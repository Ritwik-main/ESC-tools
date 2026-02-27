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
#define PWM_OUT_PIN 3
#define PWM_IN_PIN  2

// Display Object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS);

// Custom PWM Variables (Timer 1 driven)
volatile unsigned int pwmTop = 39999;     // TOP for freq (Default 50Hz)
volatile unsigned int pwmCompare = 2000;  // Pulse width in ticks
volatile bool pwmOutputEnabled = false;

int currentThrottle = 1000;

// PWM Reader Variables
volatile unsigned long pulseStart = 0;
volatile unsigned long lastRise = 0;
volatile unsigned long pulsePeriod = 0;
volatile int pulseInValue = 0;

int minObserved = 3000;
int maxObserved = 0;
float signalFreq = 0;
uint8_t scopeBuffer[64]; // Waveform history
uint8_t scopeIdx = 0;

// Dropped Frame Tracking
volatile unsigned long lastPulseTime = 0;
volatile int droppedFrameCount = 0;
unsigned long lastDropReset = 0;
int droppedFPS = 0;

// PPM Reader Variables
volatile int ppmInValues[12];
volatile byte ppmInChannel = 0;
volatile unsigned long lastPpmTime = 0;
volatile unsigned int ppmFrameCount = 0;

// Sweep Variables
bool sweepDirection = true;
unsigned long lastSweepUpdate = 0;
int sweepInterval = 30; // ms

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
  PPM_GENERATOR
};

SystemState currentState = SPLASH;
unsigned long stateTimer = 0;

// Menu Variables
int menuIndex = 0;
int menuScrollOffset = 0;
const int menuItemsCount = 9;
const int visibleItemsCount = 5; 
const char* menuItems[] = {
  "Manual PWM",
  "Step Throttle",
  "Auto Sweep",
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
      }
      if (currentState == SETTINGS) {
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
        int progress = map(elapsed, 0, 2500, 0, 100);
        drawSplashScreen(progress);
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

void drawSplashScreen(int progress) {
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
      case 3: currentState = PWM_READER; break;
      case 4: currentState = PPM_READER; break;
      case 5: currentState = STRESS_TEST; break;
      case 6: currentState = CALIBRATION; break;
      case 7: currentState = PPM_GENERATOR; break;
      case 8: currentState = SETTINGS; break;
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
    }
    else if (settingsIndex == 2) {
      if (sysSettings.frequency == 50) sysSettings.frequency = 60;
      else if (sysSettings.frequency == 60) sysSettings.frequency = 400;
      else if (sysSettings.frequency == 400) sysSettings.frequency = 600;
      else if (sysSettings.frequency == 600) sysSettings.frequency = 1000;
      else sysSettings.frequency = 50;
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
  unsigned long now = micros();
  if (digitalRead(PWM_IN_PIN) == HIGH) {
    if (lastRise > 0) {
      pulsePeriod = now - lastRise;
      // If gap is > 2x the expected period (e.g. > 40ms at 50Hz), it's a drop
      unsigned long expectedMax = (pulsePeriod > 0) ? (pulsePeriod * 2) : 40000;
      if (now - lastRise > expectedMax && lastRise > 0) droppedFrameCount++;
    }
    lastRise = now;
    pulseStart = now;
  } else {
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
  static bool interruptAttached = false;
  if (!interruptAttached) {
    pinMode(PWM_IN_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PWM_IN_PIN), pwmMeasureISR, CHANGE);
    interruptAttached = true;
    minObserved = 3000; maxObserved = 0; // Reset stats
  }

  // Calculate Stats
  if (pulseInValue > 0) {
    if (pulseInValue < minObserved) minObserved = pulseInValue;
    if (pulseInValue > maxObserved) maxObserved = pulseInValue;
  }
  if (pulsePeriod > 0) signalFreq = 1000000.0 / pulsePeriod;

  if (!pwmOutputEnabled) pwmOutputEnabled = true; if (millis() - lastSweepUpdate > 20) { if (sweepDirection) { currentThrottle += 10; if (currentThrottle >= 2000) sweepDirection = false; } else { currentThrottle -= 10; if (currentThrottle <= 1000) sweepDirection = true; } updatePWMParams(50, currentThrottle); lastSweepUpdate = millis(); }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("---SIGNAL ANALYZER---"));
  
  // Pulse & Freq
  display.setCursor(0, 12);
  display.print(pulseInValue); display.print(F("us "));
  display.print(signalFreq, 1); display.print(F("Hz"));

  // Statistics
  display.setCursor(0, 22);
  display.print(F("Min:")); display.print(minObserved);
  display.print(F(" Max:")); display.print(maxObserved);
  display.print(F(" \nJit:")); display.print(maxObserved - minObserved);
  display.print(F(" Drp:")); display.print(droppedFrameCount);

  // --- Mini Scope (Logic Analyzer Style) ---
  // Draw a clean baseline
  display.drawLine(0, 63, 127, 63, SSD1306_WHITE);

  // Calculate pulse width visually
  int highWidth = map(pulseInValue, 800, 2200, 2, 120);
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
  }

  // Handle exiting (detach interrupt)
  if (currentState != PWM_READER) {
    detachInterrupt(digitalPinToInterrupt(PWM_IN_PIN));
    interruptAttached = false;
  }
}

void handlePPMReader() {
  static bool interruptAttached = false;
  static unsigned long lastFpsCalc = 0;
  static unsigned int lastFrameCount = 0;
  
  if (!interruptAttached) {
    pinMode(PWM_IN_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PWM_IN_PIN), ppmMeasureISR, RISING);
    interruptAttached = true;
    for(int i=0; i<12; i++) ppmInValues[i] = 0;
    ppmFrameCount = 0;
    lastFrameCount = 0;
  }

  // Calculate Refresh Rate
  if (millis() - lastFpsCalc > 1000) {
    droppedFPS = ppmFrameCount - lastFrameCount;
    lastFrameCount = ppmFrameCount;
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
    display.print(ppmInValues[i]);
    
    // Mini bar
    int barW = map(constrain(ppmInValues[i], 800, 2200), 800, 2200, 0, 28);
    display.drawRect(x + 34, y + 1, 30, 5, SSD1306_WHITE);
    display.fillRect(x + 35, y + 2, barW, 3, SSD1306_WHITE);
  }

  // Monitor refresh rate
  display.setCursor(0, 56);
  display.print(F("RATE: ")); 
  display.print(droppedFPS);
  display.print(F(" Hz"));
  
  display.display();

  if (currentState != PPM_READER) {
    detachInterrupt(digitalPinToInterrupt(PWM_IN_PIN));
    interruptAttached = false;
  }
}

// Timer 1 Interrupts for manual PWM on Pin 3
ISR(TIMER1_COMPA_vect) {
  if (currentState == PPM_GENERATOR) {
    static unsigned long usedTicks = 0;
    if (ppmPhase == 0) usedTicks = 0;

    if (ppmPhase % 2 == 0) {
      digitalWrite(PWM_OUT_PIN, HIGH);
      OCR1A = 600; // 300us pulse
      usedTicks += 600;
    } else {
      digitalWrite(PWM_OUT_PIN, LOW);
      int chan = ppmPhase / 2;
      if (chan < 12) {
        unsigned int valTicks = (ppmValues[chan] * 2) - 600;
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
  } else {
    // Cycle has finished (resets to 0). Start new pulse.
    if (pwmOutputEnabled) digitalWrite(PWM_OUT_PIN, HIGH);
  }
}

ISR(TIMER1_COMPB_vect) {
  if (currentState == PPM_GENERATOR) return;
  // Pulse width reached. End pulse.
  digitalWrite(PWM_OUT_PIN, LOW);
}

void updatePWMParams(int hz, int us) {
  // 16MHz / 8 = 2MHz (0.5us per tick)
  unsigned long top = (2000000UL / hz) - 1;
  unsigned int compare = us * 2; // us to ticks (0.5us each)
  
  // Safety check
  if (compare >= top) compare = top - 10;
  if (compare < 10) compare = 10;

  noInterrupts();
  pwmTop = (unsigned int)top;
  pwmCompare = compare;
  OCR1A = pwmTop;
  OCR1B = pwmCompare;
  interrupts();
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
  int barWidth = map(currentThrottle, sysSettings.minPulse, sysSettings.maxPulse, 0, 128);

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

  if (digitalRead(BTN_UP) == LOW) {
    ppmValues[currentPpmChan] += 5;
    if (ppmValues[currentPpmChan] > sysSettings.maxPulse) ppmValues[currentPpmChan] = sysSettings.maxPulse;
    lastButtonPress = millis();
  } else if (digitalRead(BTN_DOWN) == LOW) {
    ppmValues[currentPpmChan] -= 5;
    if (ppmValues[currentPpmChan] < sysSettings.minPulse) ppmValues[currentPpmChan] = sysSettings.minPulse;
    lastButtonPress = millis();
  }
}
