// FILE: config.h
// Centralized configuration and constants for Pine Chamber Controller

#ifndef PINE_CHAMBER_CONFIG_H
#define PINE_CHAMBER_CONFIG_H

// Timing
const unsigned long READ_INTERVAL_MS           = 2000UL;   // sensor read interval
const unsigned long SCROLL_INTERVAL_MS         = 40UL;     // title scroll interval
const unsigned long ALARM_DEBOUNCE_MS          = 10000UL;  // min gap between alarm repeats
const unsigned long PAGE_SWITCH_INTERVAL_MS    = 5000UL;   // display page switch interval
const unsigned long ALARM_DISPLAY_TIME_MS      = 10000UL;  // time alarm message is displayed (10 seconds)

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

// Define whether 74HC595 outputs are inverted (connected to negative terminals)
#define INVERT_SHIFT_REGISTER_OUTPUTS 1  // Set to 1 if outputs control negative terminals, 0 otherwise

// Soil digital pin logic. Set according to your HW-080 wiring: many modules read HIGH when dry.
const uint8_t SOIL_DIGITAL_DRY_STATE = HIGH;
const uint8_t SOIL_DIGITAL_WET_STATE = (SOIL_DIGITAL_DRY_STATE == HIGH) ? LOW : HIGH;

// Soil moisture sensor enable/disable
#define SOIL_MOISTURE_ENABLED 0  // Set to 1 to enable, 0 to disable soil moisture sensor

#endif // PINE_CHAMBER_CONFIG_H


// -------------------------
