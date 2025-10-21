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
// D6: LATCH_PIN (74HC595)
// D7: CLOCK_PIN (74HC595)
// D8: DATA_PIN (74HC595)
// 74HC595 Outputs:
//   Bit 0: Red LED
//   Bit 1: Green LED
//   Bit 2: Blue LED
//   Bit 3: Relay
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
#define LED_GREEN_BIT 1
#define LED_BLUE_BIT 2
#define RELAY_BIT 3
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
const unsigned long readInterval = 2000;

// ---------- State ----------
int dotCount = 0;
bool coolingShutdownEnabled = false;
bool coolingOffStage1 = false;
unsigned long coolingOffStart = 0;
float soilMoistureAnalog = 0; // Variable to store analog soil moisture reading
bool soilMoisture = HIGH; // Variable to store digital soil moisture reading (HIGH for dry, LOW for wet, typically)
byte shiftRegisterState = 0; // Global variable to hold the current state of the shift register outputs
bool alarmActive = false; // Global variable to track if an alarm is active
unsigned long alarmStartTime = 0; // Global variable to track when an alarm is activated

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
const unsigned long scrollInterval = 40; // adjust for speed

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
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, value);
  digitalWrite(LATCH_PIN, HIGH);
}

// ---------- Buzzer patterns ----------
void beepPattern(int count, int onTime = 120, int offTime = 120) {
  for (int i = 0; i < count; i++) {
    shiftRegisterState |= (1 << BUZZER_BIT); // Turn buzzer ON
    updateShiftRegister(shiftRegisterState);
    delay(onTime);
    shiftRegisterState &= ~(1 << BUZZER_BIT); // Turn buzzer OFF
    updateShiftRegister(shiftRegisterState);
    delay(offTime);
  }
}

void checkAlarms(float chamberTemp, float waterTemp, bool soilMoisture) {
  static unsigned long lastBeepTime = 0;
  unsigned long now = millis();
  if (now - lastBeepTime < 10000 || isnan(chamberTemp) || isnan(waterTemp)) return;

  // Digital soil moisture alarm (HIGH typically means dry)
  if (soilMoisture == HIGH) {
    displayAlarm("SOIL DRY!");
    beepPattern(7, 60, 60); // Distinct pattern for critically low soil moisture (digital)
    lastBeepTime = now;
  } else if (waterTemp > 28) {
    displayAlarm("WATER TEMP HIGH!");
    beepPattern(5, 50, 50); // Distinct pattern for high water temp
    lastBeepTime = now;
  } else if (chamberTemp > 29) {
    displayAlarm("CHAMBER TEMP HIGH!");
    beepPattern(4, 80, 80);
    lastBeepTime = now;
  } else if (chamberTemp > 27) {
    displayAlarm("CHAMBER TEMP WARM");
    beepPattern(3);
    lastBeepTime = now;
  } else if (chamberTemp > 25.5) {
    displayAlarm("CHAMBER TEMP WARM");
    beepPattern(2);
    lastBeepTime = now;
  } else if (chamberTemp > 24) {
    displayAlarm("CHAMBER TEMP WARM");
    beepPattern(1);
    lastBeepTime = now;
  }
}

void updateLEDStatus(float chamberTemp, float waterTemp, bool soilMoisture) {
  // Clear previous LED bits in shiftRegisterState
  shiftRegisterState &= ~((1 << LED_RED_BIT) | (1 << LED_GREEN_BIT) | (1 << LED_BLUE_BIT));

  // Red (Critical Alarm)
  if (soilMoisture == HIGH || chamberTemp > 29) {
    shiftRegisterState |= (1 << LED_RED_BIT); // Red
  } 
  // Yellow (Warning)
  else if (waterTemp > 28 || chamberTemp > 27 || chamberTemp > 25.5 || chamberTemp > 24) {
    shiftRegisterState |= (1 << LED_RED_BIT) | (1 << LED_GREEN_BIT); // Yellow (Red + Green)
  }
  // Green (Normal)
  else {
    shiftRegisterState |= (1 << LED_GREEN_BIT); // Green
  }
  updateShiftRegister(shiftRegisterState);
}

// ---------- Cooling shutdown sequence ----------
void handleCoolingShutdown(float chamberTemp) {
  if (!coolingShutdownEnabled) return;

  if (chamberTemp > 28 && !coolingOffStage1) {
    Serial.println("ALERT: Chamber too hot. Starting staged shutdown...");
    shiftRegisterState &= ~(1 << RELAY_BIT);  // turn off peltiers (LOW for relay ON, HIGH for OFF)
    updateShiftRegister(shiftRegisterState);
    coolingOffStart = millis();
    coolingOffStage1 = true;
  }

  if (coolingOffStage1 && (millis() - coolingOffStart > 45000)) {
    Serial.println("Stage 2: Turning off fans and pump...");
    coolingOffStage1 = false;
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  dhtInside.begin();
  dhtOutside.begin();
  ds.begin();

  pinMode(SOIL_MOISTURE_PIN, INPUT_PULLUP); // Digital soil moisture pin as input with pullup

  // Initialize 74HC595 pins
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);

  // Initialize shift register outputs (all off, relay HIGH for off)
  shiftRegisterState = (1 << RELAY_BIT); // Set relay bit HIGH (off) initially
  updateShiftRegister(shiftRegisterState);

  Wire.begin(); // SDA=D2, SCL=D1
  u8g2.begin();
  u8g2.enableUTF8Print();

  titleX = u8g2.getDisplayWidth();  // start scrolling from right
}

// ---------- Loop ----------
void loop() {
  unsigned long now = millis();

  // ----- Sensor readings every 2 seconds -----
  static float tIn=0, hIn=0, tOut=0, hOut=0, chamberTemp=0, waterTemp=0;
  if (now - lastRead >= readInterval) {
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

    // Soil Moisture
    soilMoistureAnalog = analogRead(SOIL_MOISTURE_ANALOG_PIN);
    soilMoisture = digitalRead(SOIL_MOISTURE_PIN);

    // Serial debug
    Serial.printf("In: %.1fC %.1f%% | Out: %.1fC %.1f%% | Chamber: %.1fC | Water: %.1fC | Soil Analog: %.0f | Soil Digital: %s\n",
                  tIn, hIn, tOut, hOut, chamberTemp, waterTemp, soilMoistureAnalog, soilMoisture == HIGH ? "DRY" : "WET");

    // Logic
    checkAlarms(chamberTemp, waterTemp, soilMoisture);
    updateLEDStatus(chamberTemp, waterTemp, soilMoisture);
    handleCoolingShutdown(chamberTemp);
  }

  // ----- Display -----
  if (alarmActive) {
    if (millis() - alarmStartTime >= 5000) {
      alarmActive = false;
    }
  }
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  // Scroll animation (independent)
  if (now - lastScroll > scrollInterval) {
    titleX -= 2;  // move left
    if (titleX < -u8g2.getUTF8Width(titleText.c_str()))
      titleX = u8g2.getDisplayWidth();
    lastScroll = now;
  }
  u8g2.setCursor(titleX, 10);
  u8g2.print(titleText);

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

  // Soil Moisture Analog
  printText("Soil A:", 6);
  rightText(String((int)soilMoistureAnalog), 6);

  // Soil Moisture Digital
  printText("Soil D:", 7);
  rightText(soilMoisture == HIGH ? "DRY" : "WET", 7);

  u8g2.sendBuffer();
}
