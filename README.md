# Pine Chamber Controller

## Project Overview
This project implements a monitoring and control system for a "Pine Chamber," which is a temperature-controlled environment specifically designed for growing plants. Maintaining a low temperature within the chamber is critical for its intended purpose. The 'chamber temperature' refers to the internal temperature of this chamber.

## Hardware Components
The system utilizes an ESP8266-based microcontroller and the following connected hardware. A detailed pinout can be found as a comment at the beginning of the `pine_chamber.ino` file.

*   **DHT22 Sensors (2):** One for measuring internal chamber temperature and humidity (`DHTPIN_INSIDE`) and another for external ambient temperature and humidity (`DHTPIN_OUTSIDE`). Currently, the internal DHT sensor is not functional, so the `chamberSensor` DS18B20 is exclusively relied upon for internal temperature readings. The code is written to accommodate its future functionality, and it will automatically display readings once operational.
*   **DS18B20 Temperature Sensors (2):** One for precise chamber temperature measurement (`chamberSensor`) and another for monitoring the water cooling system temperature (`waterSensor`). These are connected via a OneWire bus (`ONE_WIRE_BUS`).
*   **HW-080 Soil Moisture Sensor:** Connected for monitoring soil moisture levels (analog reading `SOIL_MOISTURE_ANALOG_PIN`) and for threshold-based detection (digital reading `SOIL_MOISTURE_PIN`).
*   **74HC595 Shift Register:** Used to control multiple outputs (LEDs, Relay, Buzzer) with a minimal number of microcontroller pins (LATCH_PIN, CLOCK_PIN, DATA_PIN).
*   **OLED Display:** An SH1106 128x64 OLED display (I2C) for real-time display of sensor readings and system status.

## Software Requirements
*   Arduino IDE
*   DHT sensor library
*   OneWire library
*   DallasTemperature library
*   U8g2lib (for SH1106 OLED)
*   **Peltier Cooler Relay:** A relay controlled via the 74HC595 shift register, which is the primary cooling mechanism for the chamber.
*   **Buzzer:** Controlled via the 74HC595 shift register for audible alarms.
*   **OLED Display:** An SH1106 128x64 OLED display (I2C, SDA=D2, SCL=D1) for real-time display of sensor readings and system status.
*   **Water Pump & Fans:** Currently directly controlled (manual operation), but planned for integration into the automated system.

## Current Functionality
1.  **Sensor Data Acquisition:** Reads temperature and humidity data from DHT22 sensors, precise temperatures from DS18B20 sensors, and both analog (`soilMoistureAnalog`) and digital (`soilMoisture`) soil moisture levels from the HW-080 sensor every 2 seconds.
2.  **OLED Display:** Shows current internal/external temperature and humidity, chamber and water temperatures, and both analog (`soilMoistureAnalog`) and digital (`soilMoisture`) soil moisture levels on a scrolling display. Shows alarm message when an alarm is active.
3.  **Alarms:** Triggers a buzzer (controlled via 74HC595) with varying patterns (e.g., single beep for >24°C, multiple beeps for higher temperatures, dry soil, etc).
4.  **Cooling Shutdown Sequence:** Implements a safety shutdown for the cooling system. If the chamber temperature exceeds 28°C, the Peltier cooler is turned off via the relay (controlled via 74HC595). A placeholder is in place for a second stage that would turn off fans and the water pump after a delay.
5.  **Multi-color LED Status Indicator:** A multi-color LED (controlled via a 74HC595 shift register) for quick visual status:
    *   **Red:** Critical alarms (e.g., dry soil, very high chamber temperature).
    *   **Yellow:** Warnings (e.g., high water temperature, moderately high chamber temperature).
    *   **Green:** Normal operation.

**Configuration:** Temperature thresholds for alarms and cooling control are currently hardcoded within the firmware. Any changes to these thresholds require modifying the source code and reflashing the microcontroller.

## Future Plans
*   **OLED Auto-Off with Button Activation:** Implement a feature where the OLED display automatically turns off after 2 minutes of inactivity to prevent burn-in. The display should turn on for 2 minutes when a button is pressed or when a buzzer alarm is triggered.
*   **Remove Analog Soil Moisture Pin:** Due to limited analog pins and the prioritization of the digital output for alarms, the analog soil moisture reading will eventually be removed entirely, with the pin becoming available for other uses.
*   **Automated Water Pump and Fan Control:** Integrate the water pump and fans into the automated control system. This will involve connecting them to additional relays and programming their control logic, especially within the cooling shutdown sequence.
* **Automated two-bucket swap system:** Since heated water bucket cools down when not used for some time, we can have 2 water reservoirs being used alternatively based on water temperature. Alarm and human intervention will only be needed if both reservoirs have high temperatures.

## Parts Yet to Purchase/Integrate
*   **Internal DHT Sensor:** The `DHTPIN_INSIDE` sensor is currently non-functional and needs to be purchased/replaced.
*   **Additional Relays:** Required for controlling the water pump and fans to enable their integration into the automated system.