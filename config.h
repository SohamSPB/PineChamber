// FILE: config.h
// Centralized configuration and constants for Pine Chamber Controller

#ifndef PINE_CHAMBER_CONFIG_H
#define PINE_CHAMBER_CONFIG_H

// Timing
const unsigned long READ_INTERVAL_MS           = 2000UL;   // sensor read interval
const unsigned long SCROLL_INTERVAL_MS         = 40UL;     // title scroll interval
const unsigned long ALARM_DEBOUNCE_MS          = 10000UL;  // min gap between alarm repeats

// Temperature thresholds (C)
const float TEMP_CRITICAL = 29.0f;   // critical alarm
const float TEMP_HIGH     = 28.0f;   // high (cooling shutdown decision)
const float TEMP_WARN     = 27.0f;   // warning
const float TEMP_WARM1    = 25.5f;   // warm
const float TEMP_NOTICE   = 24.0f;   // notice

// Cooling shutdown timing
const unsigned long COOLING_SHUTDOWN_DELAY_MS = 45000UL;

// Sensor NaN handling (reserved for later batches)
const uint8_t SENSOR_NAN_ALARM_RETRY_COUNT = 3;      // attempts before persistent alarm
const unsigned long SENSOR_NAN_ALARM_REPEAT_MS = 60000UL; // min repeat time for sensor alarm

// Actuator polarity defaults (set to match hardware wiring)
// Typical relay boards are LOW = ON, HIGH = OFF. Adjust if your board is different.
const uint8_t RELAY_ON  = LOW;
const uint8_t RELAY_OFF = HIGH;
const uint8_t BUZZER_ON = HIGH;
const uint8_t BUZZER_OFF = LOW;

// Soil digital pin logic. Set according to your HW-080 wiring: many modules read HIGH when dry.
const uint8_t SOIL_DIGITAL_DRY_STATE = HIGH;
const uint8_t SOIL_DIGITAL_WET_STATE = (SOIL_DIGITAL_DRY_STATE == HIGH) ? LOW : HIGH;

#endif // PINE_CHAMBER_CONFIG_H


// -------------------------
