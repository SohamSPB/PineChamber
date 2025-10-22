#include "config.h"
#include <Wire.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>


// A0: SOIL_MOISTURE_ANALOG_PIN (HW-080 analog reading)
// D0: SOIL_MOISTURE_PIN (HW-080 digital reading)
// D1: SCL (OLED)
// D2: SDA (OLED)
// D3: ONE_WIRE_BUS (DS18B20s)
// D4: DHTPIN_INSIDE (DHT22)
// D5: DHTPIN_OUTSIDE (DHT22)
// D6: LATCH_PIN (74HC595 ST_CP(12) )
// D7: CLOCK_PIN (74HC595 SH_CP(11) )
// D8: DATA_PIN (74HC595 DS(14) ) 
// 74HC595 Outputs:
//   Bit 0: Red LED
//   Bit 1: Blue LED
//   Bit 2: Green LED
//   Bit 3: Peltier (via relay)
//   Bit 4: Buzzer

// ---------- Pin Config ----------
#define ONE_WIRE_BUS D3
#define DHTPIN_INSIDE D4
#define DHTPIN_OUTSIDE D5
#define DHTTYPE DHT22
#define SOIL_MOISTURE_ANALOG_PIN A0 // Analog pin for soil moisture sensor
#define SOIL_MOISTURE_PIN D0 // Digital pin for soil moisture sensor

// 74HC595 Shift Register Pins
#define LATCH_PIN D6
#define CLOCK_PIN D7
#define DATA_PIN D8

// 74HC595 Output Bit Assignments
#define LED_RED_BIT 0
#define LED_BLUE_BIT 1
#define  LED_GREEN_BIT 2
#define PELTIER_BIT 3
#define BUZZER_BIT 4

// ---------- Sensors ----------
DHT dhtInside(DHTPIN_INSIDE, DHTTYPE);
DHT dhtOutside(DHTPIN_OUTSIDE, DHTTYPE);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds(&oneWire);

// DS18B20 addresses
DeviceAddress chamberSensor = { 0x28, 0xE0, 0xE6, 0x57, 0x04, 0xE1, 0x3C, 0x1D };
DeviceAddress waterSensor   = { 0x28, 0x01, 0x96, 0x43, 0xD4, 0xE1, 0x3D, 0x1F };

// ---------- OLED ----------
#ifdef U8x8_HAVE_HW_I2C
#include <Wire.h>
#endif
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// ---------- Timing ----------
unsigned long lastRead = 0;

// ---------- State ----------
int dotCount = 0;
bool coolingShutdownEnabled = false;
bool coolingOffStage1 = false;
unsigned long coolingOffStart = 0;
float soilMoistureAnalog = 0; // Variable to store analog soil moisture reading
bool soilMoisture = HIGH; // Variable to store digital soil moisture reading (HIGH for dry, LOW for wet, typically)
static byte shiftRegisterState = 0; // single source of truth for shift register output bits
bool alarmActive = false; // Global variable to track if an alarm is active
unsigned long alarmStartTime = 0; // Global variable to track when an alarm is activated

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

void logEvent(const char *level, const char *message);
void setSafeActuators();
void centerText(String strr, int lineNo, bool buf);

void displayAlarm(String message) {
  alarmActive = true;
  alarmStartTime = millis();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr); // Larger font for alarm
  centerText(message, 3, true);
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

void centerText(String strr, int lineNo, bool buf = false) {
  char *charArray = getChars(strr);
  u8g2.setCursor((u8g2.getDisplayWidth() - u8g2.getUTF8Width(charArray)) / 2, (lineNo + 1) * 10);
  u8g2.print(strr);
  if (buf) u8g2.sendBuffer();
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
  Serial.printf("setting color to red");
  sr_applyBits(mask, vals);
}

void setLedColorYellow() {
  uint8_t mask = (1 << LED_RED_BIT) | (1 << LED_GREEN_BIT) | (1 << LED_BLUE_BIT);
  uint8_t vals = (1 << LED_RED_BIT) | (1 << LED_GREEN_BIT);
  Serial.printf("setting color to yellow");
  sr_applyBits(mask, vals);
}

void setLedColorGreen() {
  uint8_t mask = (1 << LED_RED_BIT) | (1 << LED_GREEN_BIT) | (1 << LED_BLUE_BIT);
  uint8_t vals = (1 << LED_GREEN_BIT);
  Serial.printf("setting color to green");
  sr_applyBits(mask, vals);
}

// ---------- Buzzer patterns ----------
void beepPattern(int count, int onTime = 120, int offTime = 120) {
  for (int i = 0; i < count; i++) {
    setBuzzer(true);
    delay(onTime);
    setBuzzer(false);
    delay(offTime);
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
      beepPattern(3, 150, 100);
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
      beepPattern(3, 150, 100);
      if (waterSensorFailCount >= SENSOR_NAN_ALARM_RETRY_COUNT) {
        if (!waterPersistentAlarm) {
          waterPersistentAlarm = true;
          logEvent("ALARM", "Water sensor persistent failure - escalating to safe mode");
          setSafeActuators();
        }
      }
    }
  } else {
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
    beepPattern(7, 60, 60); // Distinct pattern for critically low soil moisture (digital)
    logEvent("ALARM", "Soil moisture digital indicates DRY");
    lastBeepTime = now;
    return;
  }
#endif

  if (waterTemp > TEMP_HIGH) {
    displayAlarm("WATER TEMP HIGH!");
    beepPattern(5, 150, 150); // Distinct pattern for high water temp
    logEvent("ALARM", "Water temperature exceeds threshold");
    lastBeepTime = now;
    return;
  }

  if (chamberTemp > TEMP_CRITICAL) {
    displayAlarm("CHAMBER TEMP HIGH!");
    beepPattern(4, 80, 80);
    logEvent("ALARM", "Chamber temperature critical");
    lastBeepTime = now;
    return;
  }

  if (chamberTemp > TEMP_WARN) {
    displayAlarm("CHAMBER TEMP WARM");
    beepPattern(3);
    logEvent("WARN", "Chamber temperature warning");
    lastBeepTime = now;
    return;
  }

  if (chamberTemp > TEMP_WARM1) {
    displayAlarm("CHAMBER TEMP WARM");
    beepPattern(2);
    logEvent("WARN", "Chamber temperature warm");
    lastBeepTime = now;
    return;
  }

  if (chamberTemp > TEMP_NOTICE) {
    displayAlarm("CHAMBER TEMP NOTICE");
    beepPattern(1);
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

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  dhtInside.begin();
  dhtOutside.begin();
  ds.begin();

#if SOIL_MOISTURE_ENABLED
  pinMode(SOIL_MOISTURE_PIN, INPUT_PULLUP); // Digital soil moisture pin as input with pullup
#endif

  // Initialize 74HC595 pins
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);

  // Initialize to safe defaults BEFORE any blocking operations
  setPeltier(true);
  setBuzzer(false);
  setLedColorYellow();

  Wire.begin(); // SDA=D2, SCL=D1
  u8g2.begin();
  u8g2.enableUTF8Print();

  titleX = u8g2.getDisplayWidth();  // start scrolling from right
  logEvent("INFO", "Pine Chamber startup complete. Actuators set to safe defaults.");
}

// ---------- Loop ----------
void loop() {
  unsigned long now = millis();

  // ----- Sensor readings every READ_INTERVAL_MS -----
  static float tIn=0, hIn=0, tOut=0, hOut=0, chamberTemp=0, waterTemp=0;
  if (now - lastRead >= READ_INTERVAL_MS) {
    lastRead = now;

    // DHT
    tIn = dhtInside.readTemperature();
    hIn = dhtInside.readHumidity();
    tOut = dhtOutside.readTemperature();
    hOut = dhtOutside.readHumidity();

    // DS18B20
    ds.requestTemperatures();
    chamberTemp = ds.getTempC(chamberSensor);
    waterTemp = ds.getTempC(waterSensor);

#if SOIL_MOISTURE_ENABLED
    // Soil Moisture
    soilMoistureAnalog = analogRead(SOIL_MOISTURE_ANALOG_PIN);
    soilMoisture = digitalRead(SOIL_MOISTURE_PIN);
    
    // Serial debug
    Serial.printf("In: %.1fC %.1f%% | Out: %.1fC %.1f%% | Chamber: %.1fC | Water: %.1fC | Soil Analog: %.0f | Soil Digital: %s\n",
                  tIn, hIn, tOut, hOut, chamberTemp, waterTemp, soilMoistureAnalog, soilMoisture == SOIL_DIGITAL_DRY_STATE ? "DRY" : "WET");
#else
    // Serial debug (without soil moisture)
    Serial.printf("In: %.1fC %.1f%% | Out: %.1fC %.1f%% | Chamber: %.1fC | Water: %.1fC\n",
                  tIn, hIn, tOut, hOut, chamberTemp, waterTemp);
#endif

    // Logic
#if SOIL_MOISTURE_ENABLED
    checkAlarms(chamberTemp, waterTemp, soilMoisture);
    updateLEDStatus(chamberTemp, waterTemp, soilMoisture);
#else
    checkAlarms(chamberTemp, waterTemp, HIGH); // Pass default HIGH value when soil sensor is disabled
    updateLEDStatus(chamberTemp, waterTemp, HIGH); // Pass default HIGH value when soil sensor is disabled
#endif
    handleCoolingShutdown(chamberTemp);
  }

  // ----- Display -----
  if (alarmActive) {
    if (millis() - alarmStartTime >= 5000) {
      alarmActive = false;
    }
  } else { // Don't switch pages if an alarm is active
      if (now - lastPageSwitch >= PAGE_SWITCH_INTERVAL_MS) {
        lastPageSwitch = now;
        currentPage = (currentPage + 1) % 2;
      }
  }

  u8g2.clearBuffer();
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
    rightText(!isnan(tIn)&&!isnan(hIn)? String(tIn,1)+"C "+String((int)hIn)+"%":"Err", 2);

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
    rightText(soilMoisture == SOIL_DIGITAL_DRY_STATE ? "DRY" : "WET", 3);
    
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

  u8g2.sendBuffer();
}
