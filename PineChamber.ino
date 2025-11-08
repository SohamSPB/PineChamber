#include "config.h"
#include <Wire.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "TM1638.h"


// PINOUT RATIONALE:
// This pinout is carefully designed to accommodate all peripherals on the NodeMCU ESP8266,
// specifically addressing the sensitive boot requirements of GPIO0 (D3), GPIO2 (D4), and GPIO15 (D8).
//
// A0: SOIL_MOISTURE_ANALOG_PIN (Analog input for soil moisture sensor)
// D0 (GPIO16): DATA_PIN (74HC595 DS(14)) - Safe general-purpose I/O.
//
// D1 (GPIO5): SCL (OLED I2C) - Standard hardware I2C pin, safe.
// D2 (GPIO4): SDA (OLED I2C) - Standard hardware I2C pin, safe.
// D3 (GPIO0): ONE_WIRE_BUS (DS18B20s)
//             - Boot Requirement: Must be HIGH at boot.
//             - Reasoning: OneWire bus typically uses a pull-up resistor, ensuring this pin is HIGH.
// D4 (GPIO2): TM_STB (TM1638 Strobe)
//             - Boot Requirement: Must be HIGH at boot.
//             - Reasoning: The TM1638 library initializes this pin as an output and sets it HIGH,
//                          perfectly matching the GPIO2 boot requirement.
// D5 (GPIO14): DHTPIN_OUTSIDE (DHT22) - Safe general-purpose I/O.
// D6 (GPIO12): TM_DIO (TM1638 Data In/Out) - Safe general-purpose I/O, suitable for bidirectional communication.
// D7 (GPIO13): SHARED_CLOCK_PIN (74HC595 SH_CP(11) & TM1638 CLK)
//             - Reasoning: GPIO13 is a safe general-purpose I/O. Sharing the clock works because
//                          only one device is addressed at a time via its STB/LATCH pin.
// D8 (GPIO15): LATCH_PIN (74HC595 ST_CP(12))
//             - Boot Requirement: Must be LOW at boot.
//             - Reasoning: This pin is configured as an output and explicitly set LOW in setup()
//                          immediately, satisfying the GPIO15 boot requirement.
//
// 74HC595 Outputs (controlled via LATCH_PIN, CLOCK_PIN, DATA_PIN):
//   Bit 0: Red LED
//   Bit 1: Blue LED
//   Bit 2: Green LED
//   Bit 3: Peltier (via relay)
//   Bit 4: Buzzer

// ---------- Pin Config ----------
#define ONE_WIRE_BUS D3
// #define DHTPIN_INSIDE D4 // REMOVED, replaced by DS18B20
#define DHTPIN_OUTSIDE D5
#define DHTTYPE DHT22
#define SOIL_MOISTURE_ANALOG_PIN A0 // Analog pin for soil moisture sensor
// #define SOIL_MOISTURE_PIN D0 // REMOVED to free up pin, using analog reading only
#define SOIL_ANALOG_DRY_THRESHOLD 750 // ADC value above which soil is considered "DRY". Adjust as needed.

// 74HC595 Shift Register Pins
#define LATCH_PIN D8
#define CLOCK_PIN D7
#define DATA_PIN D0

// TM1638 Display Pins
#define TM_STB D4
#define TM_CLK D7 // SHARED with 74HC595 CLOCK_PIN
#define TM_DIO D6

// 74HC595 Output Bit Assignments
#define LED_RED_BIT 0
#define LED_BLUE_BIT 1
#define  LED_GREEN_BIT 2
#define PELTIER_BIT 3
#define BUZZER_BIT 4

// ---------- Sensors ----------
// DHT dhtInside(DHTPIN_INSIDE, DHTTYPE); // REMOVED
DHT dhtOutside(DHTPIN_OUTSIDE, DHTTYPE);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds(&oneWire);
TM1638 module(TM_CLK, TM_DIO, TM_STB);

// DS18B20 addresses
DeviceAddress chamberSensor = { 0x28, 0xE0, 0xE6, 0x57, 0x04, 0xE1, 0x3C, 0x1D };
DeviceAddress waterSensor   = { 0x28, 0x01, 0x96, 0x43, 0xD4, 0xE1, 0x3D, 0x1F };
DeviceAddress insideSensor  = { 0x28, 0x89, 0x84, 0xBC, 0x00, 0x00, 0x00, 0x35 };

// ---------- OLED ----------
#ifdef U8x8_HAVE_HW_I2C
#include <Wire.h>
#endif
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// ---------- Timing ----------
unsigned long lastRead = 0;
const unsigned long ALARM_PAGE_TOGGLE_INTERVAL_MS = 5000; // 5 seconds

// ---------- State ----------
int dotCount = 0;
bool coolingShutdownEnabled = false;
bool coolingOffStage1 = false;
unsigned long coolingOffStart = 0;
float soilMoistureAnalog = 0; // Variable to store analog soil moisture reading
bool soilIsDry = false; // Derived from analog reading. True if soil is dry.
static byte shiftRegisterState = 0; // single source of truth for shift register output bits
bool alarmActive = false; // Global variable to track if an alarm is active
unsigned long alarmStartTime = 0; // Global variable to track when an alarm is activated
String currentAlarmMessage = ""; // Store the current alarm message to display repeatedly

// TM1638 LED & Button Feature State
enum LedState { NORMAL, WARNING, CRITICAL };
LedState currentLedState = NORMAL;
bool blinkLedState = false;
unsigned long lastBlinkTime = 0;
bool buzzerSilenced = false;
bool showAlarmPage = false; // Toggles between alarm and data pages during an alarm
unsigned long lastAlarmPageToggle = 0; // Timer for the alarm page toggle

// Buzzer state machine
bool buzzerActive = false;
unsigned long buzzerOnStart = 0;
unsigned long buzzerOffStart = 0;
int buzzerPatternCount = 0;
int buzzerPatternRemaining = 0;
int buzzerOnTime = 120;
int buzzerOffTime = 120;
bool buzzerIsOn = false;


// Page switching
int currentPage = 0;
unsigned long lastPageSwitch = 0;

// Operator override for bench testing. Default false.
bool OPERATOR_OVERRIDE = false;

// Sensor fault tracking
uint8_t chamberSensorFailCount = 0;
uint8_t waterSensorFailCount = 0;
bool chamberPersistentAlarm = false;
bool waterPersistentAlarm = false;
unsigned long lastChamberSensorAlarm = 0;
unsigned long lastWaterSensorAlarm = 0;

// Forward declarations for get/set APIs
void sr_setBit(uint8_t bit, bool level);
void sr_applyBits(uint8_t mask, uint8_t values);
bool sr_getBit(uint8_t bit);
void setPeltier(bool on);
void setBuzzer(bool on);
void setLedColorRed();
void setLedColorYellow();
void setLedColorGreen();
void updateTm1638Display(float chamberTemp, float outsideTemp);

void logEvent(const char *level, const char *message);
void setSafeActuators();
void centerText(String strr);

void displayAlarm(String message) {
  // Update the alarm display with new message
  alarmActive = true;
  alarmStartTime = millis();
  currentAlarmMessage = message;
  buzzerSilenced = false; // Ensure new alarms are not silenced by default

  // Reset alarm display toggling
  showAlarmPage = true;
  lastAlarmPageToggle = millis();

  centerText(message);  // Use the corrected centerText function
}


// ---------- Scrolling title ----------
String titleText = "Pine Chamber   ";
int titleX = 0;
unsigned long lastScroll = 0;

// ---------- Helpers ----------
char *getChars(String input) {
  int count = input.length() + 1;
  char *charArray = (char *)malloc(count);
  if (charArray != NULL) input.toCharArray(charArray, count);
  return charArray;
}

void printText(String strr, int lineNo, bool buf = false) {
  u8g2.setCursor(0, (lineNo + 1) * 10);
  u8g2.print(strr);
  if (buf) u8g2.sendBuffer();
}

void rightText(String strr, int lineNo, bool buf = false) {
  char *charArray = getChars(strr);
  u8g2.setCursor((u8g2.getDisplayWidth() - u8g2.getUTF8Width(charArray)) - 2, (lineNo + 1) * 10);
  u8g2.print(strr);
  if (buf) u8g2.sendBuffer();
  free(charArray);
}

void centerText(String message) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB10_tr); // Larger font for alarm
    
    // Calculate text width to ensure it fits on screen
    char *charArray = getChars(message);
    int textWidth = u8g2.getUTF8Width(charArray);
    int displayWidth = u8g2.getDisplayWidth();
    
    // If text is wider than display, try a smaller font
    if (textWidth >= displayWidth) {
      u8g2.setFont(u8g2_font_ncenB08_tr); // Smaller font for longer messages
      textWidth = u8g2.getUTF8Width(charArray);
    }
    
    // Calculate centered position, ensuring it doesn't go negative and text stays on screen
    int x = max(0, (displayWidth - textWidth) / 2);
    // Ensure we don't exceed display boundaries when centered
    if (x + textWidth > displayWidth) {
      x = 0; // If still too wide, align to left
    }
    
    u8g2.setCursor(x, 35); // Center vertically on screen
    u8g2.print(message);
    u8g2.sendBuffer();
    free(charArray);
}

// ---------- 74HC595 Helper ----------
void updateShiftRegister(byte value) {
#if INVERT_SHIFT_REGISTER_OUTPUTS
    value = ~value;
#endif
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, value);
  digitalWrite(LATCH_PIN, HIGH);
}

void sr_setBit(uint8_t bit, bool level) {
  if (level) shiftRegisterState |= (1 << bit);
  else shiftRegisterState &= ~(1 << bit);
  updateShiftRegister(shiftRegisterState);
}

void sr_applyBits(uint8_t mask, uint8_t values) {
  shiftRegisterState = (shiftRegisterState & ~mask) | (values & mask);
  updateShiftRegister(shiftRegisterState);
}

bool sr_getBit(uint8_t bit) {
  return (shiftRegisterState & (1 << bit)) != 0;
}

void setPeltier(bool on) {
  sr_setBit(PELTIER_BIT, on);
}

void setBuzzer(bool on) {
  sr_setBit(BUZZER_BIT, on);
}

void setLedColorRed() {
  uint8_t mask = (1 << LED_RED_BIT) | (1 << LED_GREEN_BIT) | (1 << LED_BLUE_BIT);
  uint8_t vals = (1 << LED_RED_BIT);
  Serial.printf("setting color to red\n");
  sr_applyBits(mask, vals);
}

void setLedColorYellow() {
  uint8_t mask = (1 << LED_RED_BIT) | (1 << LED_GREEN_BIT) | (1 << LED_BLUE_BIT);
  uint8_t vals = (1 << LED_RED_BIT) | (1 << LED_GREEN_BIT);
  Serial.printf("setting color to yellow\n");
  sr_applyBits(mask, vals);
}

void setLedColorGreen() {
  uint8_t mask = (1 << LED_RED_BIT) | (1 << LED_GREEN_BIT) | (1 << LED_BLUE_BIT);
  uint8_t vals = (1 << LED_GREEN_BIT);
  Serial.printf("setting color to green\n");
  sr_applyBits(mask, vals);
}

// ---------- TM1638 Features ----------
void setLedStatus(float chamberTemp) {
  if (chamberTemp > TEMP_CRITICAL) {
    currentLedState = CRITICAL;
  } else if (chamberTemp > TEMP_WARN) {
    currentLedState = WARNING;
  } else {
    currentLedState = NORMAL;
  }
}

void updateTm1638Leds() {
  switch (currentLedState) {
    case NORMAL:
      module.writeLeds(TM1638_LED_NORMAL);
      break;
    case WARNING:
      module.writeLeds(TM1638_LED_WARNING);
      break;
    case CRITICAL:
      if (millis() - lastBlinkTime > TM1638_BLINK_INTERVAL_MS) {
        lastBlinkTime = millis();
        blinkLedState = !blinkLedState;
        module.writeLeds(blinkLedState ? TM1638_LED_CRITICAL : 0b00000000);
      }
      break;
  }
}

void handleButtons() {
  uint8_t buttons = module.getButtons();
  if (buttons & (1 << BUTTON_SILENCE_ALARM)) {
    buzzerSilenced = true;
  }
}

// ---------- Buzzer patterns ----------
void startBuzzerPattern(int count, int onTime = 120, int offTime = 120) {
  buzzerPatternRemaining = count;
  buzzerOnTime = onTime;
  buzzerOffTime = offTime;
  buzzerActive = true;
  buzzerIsOn = true;
  setBuzzer(true);
  buzzerOnStart = millis();
}

void updateBuzzerPattern() {
  if (buzzerSilenced) {
    setBuzzer(false);
    buzzerActive = false;
    return;
  }
  if (!buzzerActive) return;
  
  unsigned long now = millis();
  
  if (buzzerIsOn) {
    // Currently buzzer is on
    if (now - buzzerOnStart >= buzzerOnTime) {
      // Turn buzzer off
      setBuzzer(false);
      buzzerIsOn = false;
      buzzerOffStart = now;
    }
  } else {
    // Currently buzzer is off
    if (now - buzzerOffStart >= buzzerOffTime) {
      // Either start next beep or finish pattern
      buzzerPatternRemaining--;
      if (buzzerPatternRemaining > 0) {
        // Start next beep in pattern
        setBuzzer(true);
        buzzerIsOn = true;
        buzzerOnStart = now;
      } else {
        // Pattern complete
        buzzerActive = false;
        setBuzzer(false);
      }
    }
  }
}

// ---------- Logging ----------
void logEvent(const char *level, const char *message) {
  // Minimal logging: uptime timestamp in ms
  char buf[128];
  unsigned long t = millis();
  snprintf(buf, sizeof(buf), "[%lu] %s: %s", t, level, message);
  Serial.println(buf);
}

void setSafeActuators() {
  // Move actuators to safe state. Respect operator override during bench testing.
  if (OPERATOR_OVERRIDE) {
    logEvent("INFO", "Operator override enabled - skipping safe actuator enforcement.");
    return;
  }
  setPeltier(true);
  setBuzzer(false);
  setLedColorRed(); // visible critical state
  logEvent("INFO", "Actuators moved to safe state.");
}

void checkAlarms(float chamberTemp, float waterTemp, bool soilMoisture) {
  static unsigned long lastBeepTime = 0;
  unsigned long now = millis();

  // --- Sensor NaN handling (always check) ---
  // Chamber sensor
  if (isnan(chamberTemp)) {
    // throttle repeated logs/alarms
    if (now - lastChamberSensorAlarm > SENSOR_NAN_ALARM_REPEAT_MS) {
      chamberSensorFailCount++;
      lastChamberSensorAlarm = now;
      logEvent("ALARM", "Chamber sensor read NaN");
      displayAlarm("SENSOR ERROR");
      // brief audible pattern to indicate sensor failure
      startBuzzerPattern(3, 150, 100);
      // move actuators to safe state if failure persists
      if (chamberSensorFailCount >= SENSOR_NAN_ALARM_RETRY_COUNT) {
        if (!chamberPersistentAlarm) {
          chamberPersistentAlarm = true;
          logEvent("ALARM", "Chamber sensor persistent failure - escalating to safe mode");
          setSafeActuators();
        }
      }
    }
  } else {
    // successful read: clear counters and persistent state if present
    if (chamberSensorFailCount > 0 || chamberPersistentAlarm) {
      chamberSensorFailCount = 0;
      if (chamberPersistentAlarm) {
        chamberPersistentAlarm = false;
        logEvent("INFO", "Chamber sensor recovered");
      }
    }
  }

  // Water sensor
  if (isnan(waterTemp)) {
    if (now - lastWaterSensorAlarm > SENSOR_NAN_ALARM_REPEAT_MS) {
      waterSensorFailCount++;
      lastWaterSensorAlarm = now;
      logEvent("ALARM", "Water sensor read NaN");
      displayAlarm("SENSOR ERROR");
      startBuzzerPattern(3, 150, 100);
      if (waterSensorFailCount >= SENSOR_NAN_ALARM_RETRY_COUNT) {
        if (!waterPersistentAlarm) {
          waterPersistentAlarm = true;
          logEvent("ALARM", "Water sensor persistent failure - escalating to safe mode");
          setSafeActuators();
        }
      }
    }
  }
  else {
    if (waterSensorFailCount > 0 || waterPersistentAlarm) {
      waterSensorFailCount = 0;
      if (waterPersistentAlarm) {
        waterPersistentAlarm = false;
        logEvent("INFO", "Water sensor recovered");
      }
    }
  }

  // If either sensor has escalated to persistent alarm, remain in safe mode and avoid other actuator actions.
  if (chamberPersistentAlarm || waterPersistentAlarm) {
    // ensure actuators are in safe state
    setSafeActuators();
    return; // do not proceed to normal alarm branching while persistent sensor fault is active
  }

  // --- Normal alarm branching ---
  if (now - lastBeepTime < ALARM_DEBOUNCE_MS) return;

#if SOIL_MOISTURE_ENABLED
  if (soilMoisture == SOIL_DIGITAL_DRY_STATE) {
    displayAlarm("SOIL DRY!");
    startBuzzerPattern(7, 60, 60); // Distinct pattern for critically low soil moisture (digital)
    logEvent("ALARM", "Soil moisture digital indicates DRY");
    lastBeepTime = now;
    return;
  }
#endif

  if (waterTemp > TEMP_HIGH) {
    displayAlarm("WATER TEMP HIGH!");
    startBuzzerPattern(5, 150, 150); // Distinct pattern for high water temp
    logEvent("ALARM", "Water temperature exceeds threshold");
    lastBeepTime = now;
    return;
  }

  if (chamberTemp > TEMP_CRITICAL) {
    displayAlarm("CHAMBER TEMP HIGH!");
    startBuzzerPattern(4, 80, 80);
    logEvent("ALARM", "Chamber temperature critical");
    lastBeepTime = now;
    return;
  }

  if (chamberTemp > TEMP_WARN) {
    displayAlarm("CHAMBER TEMP WARM");
    startBuzzerPattern(3);
    logEvent("WARN", "Chamber temperature warning");
    lastBeepTime = now;
    return;
  }

  if (chamberTemp > TEMP_WARM1) {
    displayAlarm("CHAMBER TEMP WARM");
    startBuzzerPattern(2);
    logEvent("WARN", "Chamber temperature warm");
    lastBeepTime = now;
    return;
  }

  if (chamberTemp > TEMP_NOTICE) {
    displayAlarm("CHAMBER TEMP NOTICE");
    startBuzzerPattern(1);
    logEvent("INFO", "Chamber temperature notice");
    lastBeepTime = now;
    return;
  }
}

void updateLEDStatus(float chamberTemp, float waterTemp, bool soilMoisture) {
#if SOIL_MOISTURE_ENABLED
  if (soilMoisture == SOIL_DIGITAL_DRY_STATE || chamberTemp > TEMP_CRITICAL) {
    setLedColorRed();
  }
#else
  if (chamberTemp > TEMP_CRITICAL) {
    setLedColorRed();
  }
#endif
  else if (waterTemp > TEMP_HIGH || chamberTemp > TEMP_WARN) {
    setLedColorYellow();
  }
  else {
    setLedColorGreen();
  }
}

// ---------- Cooling shutdown sequence ----------
void handleCoolingShutdown(float chamberTemp) {
  if (!coolingShutdownEnabled) return;

  if (chamberTemp > TEMP_HIGH && !coolingOffStage1) {
    Serial.println("ALERT: Chamber too hot. Starting staged shutdown...");
    // Turn off peltiers
    setPeltier(false);
    coolingOffStart = millis();
    coolingOffStage1 = true;
    logEvent("INFO", "Cooling staged shutdown started");
  }

  if (coolingOffStage1 && (millis() - coolingOffStart > COOLING_SHUTDOWN_DELAY_MS)) {
    Serial.println("Stage 2: Turning off fans and pump...");
    logEvent("INFO", "Cooling stage 2 actions executed");
    coolingOffStage1 = false;
  }
}

// ---------- TM1638 Display Update ----------

// 7-segment patterns for 0-9, A-F. Copied from TM1638 library to add decimal points.
const uint8_t digits[] = {
  0b00111111,0b00000110,0b01011011,0b01001111,
  0b01100110,0b01101101,0b01111101,0b00000111,
  0b01111111,0b01101111,0b01110111,0b01111100,
  0b00111001,0b01011110,0b01111001,0b01110001
};
// Custom patterns
const uint8_t pattern_r = 0b01010000;
const uint8_t pattern_blank = 0x00;
const uint8_t dp = 0b10000000; // decimal point bit

// Custom patterns for scrolling text
const uint8_t pattern_P = 0b01110011;
const uint8_t pattern_I = 0b00110000;
const uint8_t pattern_N = 0b01010100;
const uint8_t pattern_E = 0b01111001;
const uint8_t pattern_C = 0b00111001;
const uint8_t pattern_H = 0b01110110;
const uint8_t pattern_A = 0b01110111;
const uint8_t pattern_M = 0b00010101; // Approximation
const uint8_t pattern_B = 0b01111100;
const uint8_t pattern_R = 0b01010000;


void runStartupAnimation() {
  // 1. OLED Animation: Fade "CEDRUS"
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_logisoso24_tr); // A large, stylish font
    u8g2.drawStr((u8g2.getDisplayWidth() - u8g2.getStrWidth("CEDRUS")) / 2, u8g2.getDisplayHeight() / 2, "CEDRUS");
  } while (u8g2.nextPage());

  for (int i = 0; i <= 255; i += 5) { // Fade in
    u8g2.setContrast(i);
    delay(20);
  }
  for (int i = 255; i >= 0; i -= 5) { // Fade out
    u8g2.setContrast(i);
    delay(20);
  }

  // 2. TM1638 Animation: Scroll "PINE CHAMBER"
  uint8_t scrollText[] = {pattern_P, pattern_I, pattern_N, pattern_E, pattern_blank, pattern_C, pattern_H, pattern_A, pattern_M, pattern_B, pattern_E, pattern_R, pattern_blank, pattern_blank};
  int textLen = sizeof(scrollText);

  for (int i = 0; i < textLen + 8; i++) {
    module.displayClear();
    for (int k = 0; k < 8; k++) {
      int charIndex = i - (7 - k);
      if (charIndex >= 0 && charIndex < textLen) {
        module.displayDig(k, scrollText[charIndex]);
      }
    }
    delay(150);
  }
  module.reset(); // Clear display after animation
}

void updateTm1638Display(float chamberTemp, float outsideTemp) {
  // This function is written for the 'dvarrel/TM1638' library.
  // Digits are numbered 7(left) to 0(right).
  module.displayClear();

  // Display Chamber Temp on digits 7,6,5,4: " XX.X"
  if (chamberTemp > -100) {
    module.displayDig(7, pattern_blank); // Blank for spacing
    int tempInt = abs((int)chamberTemp);
    int tempFrac = abs((int)(chamberTemp * 10)) % 10;
    module.displayDig(6, (tempInt >= 10) ? digits[(tempInt / 10) % 10] : pattern_blank);
    module.displayDig(5, digits[tempInt % 10] | dp); // Add decimal point
    module.displayDig(4, digits[tempFrac]);
  } else {
    // Display " Err "
    module.displayDig(7, pattern_blank);
    module.displayDig(6, digits[14]); // E
    module.displayDig(5, pattern_r);   // r
    module.displayDig(4, pattern_r);   // r
  }

  // Display Outside Temp on digits 3,2,1,0: " YY.Y"
  if (!isnan(outsideTemp)) {
    module.displayDig(3, pattern_blank); // Blank for spacing
    int tempInt = abs((int)outsideTemp);
    int tempFrac = abs((int)(outsideTemp * 10)) % 10;
    module.displayDig(2, (tempInt >= 10) ? digits[(tempInt / 10) % 10] : pattern_blank);
    module.displayDig(1, digits[tempInt % 10] | dp); // Add decimal point
    module.displayDig(0, digits[tempFrac]);
  } else {
    // Display " Err "
    module.displayDig(3, pattern_blank);
    module.displayDig(2, digits[14]); // E
    module.displayDig(1, pattern_r);   // r
    module.displayDig(0, pattern_r);   // r
  }
}


// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  Serial.println("Pine Chamber Init");

  // dhtInside.begin(); // REMOVED
  dhtOutside.begin();
  ds.begin();

  // Initialize TM1638
  module.displayTurnOn();
  module.displaySetBrightness(PULSE14_16); // Max brightness
  module.reset(); // Clears both digits and LEDs

  // Initialize 74HC595 pins
  pinMode(LATCH_PIN, OUTPUT);
  digitalWrite(LATCH_PIN, LOW); // IMPORTANT: Keep D8 (GPIO15) LOW for boot
  // pinMode(CLOCK_PIN, OUTPUT); // This is handled by the TM1638 constructor, as D7 is a shared pin.
  pinMode(DATA_PIN, OUTPUT);

  // Initialize to safe defaults BEFORE any blocking operations
  setPeltier(true);
  setBuzzer(false);
  setLedColorYellow();

  Wire.begin(); // SDA=D2, SCL=D1
  u8g2.begin();
  u8g2.enableUTF8Print();

  // Run startup animations
  runStartupAnimation();

  titleX = u8g2.getDisplayWidth();  // start scrolling from right
  logEvent("INFO", "Pine Chamber startup complete. Actuators set to safe defaults.");
}

// ---------- Loop ----------
void loop() {
  unsigned long now = millis();

  // ----- Sensor readings every READ_INTERVAL_MS -----
  static float insideTemp=0, tOut=0, hOut=0, chamberTemp=0, waterTemp=0;
  if (now - lastRead >= READ_INTERVAL_MS) {
    lastRead = now;

    // DHT (Outside only)
    tOut = dhtOutside.readTemperature();
    hOut = dhtOutside.readHumidity();

    // DS18B20
    ds.requestTemperatures();
    insideTemp = ds.getTempC(insideSensor);
    chamberTemp = ds.getTempC(chamberSensor);
    waterTemp = ds.getTempC(waterSensor);

#if SOIL_MOISTURE_ENABLED
    // Soil Moisture
    soilMoistureAnalog = analogRead(SOIL_MOISTURE_ANALOG_PIN);
    soilIsDry = (soilMoistureAnalog > SOIL_ANALOG_DRY_THRESHOLD);
    
    // Serial debug
    Serial.printf("In: %.1fC | Out: %.1fC %.1f%% | Chamber: %.1fC | Water: %.1fC | Soil Analog: %.0f | Soil is Dry: %s\n",
                  insideTemp, tOut, hOut, chamberTemp, waterTemp, soilMoistureAnalog, soilIsDry ? "YES" : "NO");
#else
    // Serial debug (without soil moisture)
    Serial.printf("In: %.1fC | Out: %.1fC %.1f%% | Chamber: %.1fC | Water: %.1fC\n",
                  insideTemp, tOut, hOut, chamberTemp, waterTemp);
#endif

    // Logic
#if SOIL_MOISTURE_ENABLED
    checkAlarms(chamberTemp, waterTemp, soilIsDry);
    updateLEDStatus(chamberTemp, waterTemp, soilIsDry);
#else
    checkAlarms(chamberTemp, waterTemp, false); // Pass default false value when soil sensor is disabled
    updateLEDStatus(chamberTemp, waterTemp, false); // Pass default false value when soil sensor is disabled
#endif
    setLedStatus(chamberTemp); // Set status for TM1638 LEDs
    handleCoolingShutdown(chamberTemp);

    // Update TM1638 display
    updateTm1638Display(chamberTemp, tOut);
  }

  // Handle features that need to run on every loop iteration
  handleButtons();
  updateTm1638Leds();
  updateBuzzerPattern();

  // ----- Display -----
  // Handle alarm state and page toggling
  if (alarmActive) {
    // Check if the total alarm duration has expired
    if (millis() - alarmStartTime >= ALARM_DISPLAY_TIME_MS) {
      alarmActive = false;
      showAlarmPage = false;
    }

    // Check if it's time to toggle between alarm and data screens
    if (millis() - lastAlarmPageToggle > ALARM_PAGE_TOGGLE_INTERVAL_MS) {
      lastAlarmPageToggle = millis();
      showAlarmPage = !showAlarmPage;
    }
  }
  
  // Handle normal page switching (only when no alarm is active)
  if (!alarmActive && (now - lastPageSwitch >= PAGE_SWITCH_INTERVAL_MS)) {
    lastPageSwitch = now;
    currentPage = (currentPage + 1) % 2;
  }

  u8g2.clearBuffer();
  
  // Decide what to draw
  if (alarmActive && showAlarmPage) {
    // If an alarm is active and it's time to show it, draw the alarm message
    centerText(currentAlarmMessage);
  } else {
    // Otherwise, draw the normal data pages (this now also happens during an alarm)
    u8g2.setFont(u8g2_font_ncenB08_tr);

    // Scroll animation (independent)
    if (now - lastScroll > SCROLL_INTERVAL_MS) {
      titleX -= 2;  // move left
      if (titleX < -u8g2.getUTF8Width(titleText.c_str()))
        titleX = u8g2.getDisplayWidth();
      lastScroll = now;
    }
    u8g2.setCursor(titleX, 10);
    u8g2.print(titleText);

    if (currentPage == 0) {
    // Inside
    printText("In:", 2);
    rightText(insideTemp > -100 ? String(insideTemp,1)+"C":"Err", 2);

    // Outside
    printText("Out:", 3);
    rightText(!isnan(tOut)&&!isnan(hOut)? String(tOut,1)+"C "+String((int)hOut)+"%":"Err", 3);

    // Chamber
    printText("Chamber:", 4);
    rightText(chamberTemp>-100? String(chamberTemp,1)+"C":"Err", 4);

    // Water
    printText("Water:", 5);
    rightText(waterTemp>-100? String(waterTemp,1)+"C":"Err", 5);
  } else { // page 1
#if SOIL_MOISTURE_ENABLED
    // Soil Moisture Analog
    printText("Soil A:", 2);
    rightText(String((int)soilMoistureAnalog), 2);

    // Soil Moisture Digital
    printText("Soil D:", 3);
    rightText(soilIsDry ? "DRY" : "WET", 3);
    
    // Uptime
    unsigned long total_seconds = millis() / 1000;
    unsigned long days = total_seconds / (24 * 3600);
    total_seconds %= (24 * 3600);
    unsigned long hours = total_seconds / 3600;
    total_seconds %= 3600;
    unsigned long minutes = total_seconds / 60;

    char uptime_buf[20];
    snprintf(uptime_buf, sizeof(uptime_buf), "%lud %02luh %02lum", days, hours, minutes);
    printText("Uptime:", 4);
    rightText(uptime_buf, 4);

    // Status
    printText("Cooling:", 5);
    rightText(sr_getBit(PELTIER_BIT) ? "ON" : "OFF", 5);
#else
    // Uptime (no soil moisture display)
    unsigned long total_seconds = millis() / 1000;
    unsigned long days = total_seconds / (24 * 3600);
    total_seconds %= (24 * 3600);
    unsigned long hours = total_seconds / 3600;
    total_seconds %= 3600;
    unsigned long minutes = total_seconds / 60;

    char uptime_buf[20];
    snprintf(uptime_buf, sizeof(uptime_buf), "%lud %02luh %02lum", days, hours, minutes);
    printText("Uptime:", 2);
    rightText(uptime_buf, 2);

    // Status
    printText("Cooling:", 3);
    rightText(sr_getBit(PELTIER_BIT) ? "ON" : "OFF", 3);
    
    // Empty lines to maintain layout consistency
    printText("", 4);
    rightText("", 4);
    printText("", 5);
    rightText("", 5);
#endif
  }
  }
  u8g2.sendBuffer();
}
