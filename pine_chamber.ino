#include <Wire.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ---------- Pin Config ----------
#define ONE_WIRE_BUS D3
#define RELAY_PIN D5
#define DHTPIN_INSIDE D6
#define DHTPIN_OUTSIDE D7
#define DHTTYPE DHT22
#define BUZZER_PIN D8

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

// ---------- Buzzer patterns ----------
void beepPattern(int count, int onTime = 120, int offTime = 120) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(onTime);
    digitalWrite(BUZZER_PIN, LOW);
    delay(offTime);
  }
}

void checkAlarms(float chamberTemp) {
  static unsigned long lastBeepTime = 0;
  unsigned long now = millis();
  if (now - lastBeepTime < 10000 || isnan(chamberTemp)) return;

  if (chamberTemp > 29) {
    beepPattern(4, 80, 80);
    lastBeepTime = now;
  } else if (chamberTemp > 27) {
    beepPattern(3);
    lastBeepTime = now;
  } else if (chamberTemp > 25.5) {
    beepPattern(2);
    lastBeepTime = now;
  } else if (chamberTemp > 24) {
    beepPattern(1);
    lastBeepTime = now;
  }
}

// ---------- Cooling shutdown sequence ----------
void handleCoolingShutdown(float chamberTemp) {
  if (!coolingShutdownEnabled) return;

  if (chamberTemp > 28 && !coolingOffStage1) {
    Serial.println("ALERT: Chamber too hot. Starting staged shutdown...");
    digitalWrite(RELAY_PIN, LOW);  // turn off peltiers
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

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

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

    // Serial debug
    Serial.printf("In: %.1fC %.1f%% | Out: %.1fC %.1f%% | Chamber: %.1fC | Water: %.1fC\n",
                  tIn, hIn, tOut, hOut, chamberTemp, waterTemp);

    // Logic
    checkAlarms(chamberTemp);
    handleCoolingShutdown(chamberTemp);
  }

  // ----- Display -----
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

  u8g2.sendBuffer();
}
