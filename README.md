# Pine Chamber Controller

## Project Overview
This project implements a monitoring and control system for a "Pine Chamber," which is a temperature-controlled environment. Maintaining a low temperature within the chamber is critical for its intended purpose. The controller manages sensors, actuators, and multiple displays to provide real-time data and alerts.

## Hardware Components
The system utilizes an ESP8266-based microcontroller and the following connected hardware. A detailed, up-to-date pinout and the reasoning for each pin choice can be found in the header comment of the `PineChamber.ino` file.

*   **DS18B20 Temperature Sensors (3):** One for precise chamber temperature (`chamberSensor`), one for the water cooling system (`waterSensor`), and one for the ambient indoor temperature (`insideSensor`). All are connected via a single OneWire bus.
*   **DHT22 Sensor (1):** For measuring external ambient temperature and humidity (`DHTPIN_OUTSIDE`).
*   **HW-080 Soil Moisture Sensor:** Connected for monitoring soil moisture levels via its analog output.
*   **OLED Display:** An SH1106 128x64 I2C display for detailed real-time sensor readings and system status.
*   **TM1638 Module:** An 8-digit, 8-button, 8-LED module used as a secondary display for at-a-glance temperature readings and as a user interface.
*   **74HC595 Shift Register:** Used to control multiple outputs (a multi-color LED, a Peltier relay, and a buzzer) with a minimal number of microcontroller pins.
*   **Peltier Cooler Relay:** The primary cooling mechanism for the chamber, controlled via the 74HC595.
*   **Buzzer:** For audible alarms, also controlled via the 74HC595.

## Software Requirements
*   Arduino IDE
*   U8g2lib (for SH1106 OLED)
*   OneWire library
*   DallasTemperature library
*   DHT sensor library
*   TM1638 library (dvarrel/TM1638)

## Current Functionality
1.  **Sensor Data Acquisition:** Reads temperature data from all DS18B20 and DHT22 sensors, and analog soil moisture levels every 2 seconds.
2.  **Dual Displays:**
    *   **OLED Display:** Shows detailed data across two pages, including inside, outside, chamber, and water temperatures, humidity, soil moisture, uptime, and cooling status. It also displays critical alarm messages by alternating with the data screens.
    *   **TM1638 Display:** Shows the primary chamber and outside temperatures for a quick, at-a-glance view (`XX.X YY.Y`).
3.  **Startup Animation:** On boot, the system plays an animation across both displays, with "CEDRUS" fading on the OLED and "PINE CHAMBER" scrolling on the TM1638 module.
4.  **Multi-Level Alarms:** Triggers a buzzer with different patterns for various warning and critical thresholds for chamber temperature, water temperature, and soil moisture.
5.  **User Interface:**
    *   **Button to Silence:** The 8th button (S8) on the TM1638 module can be pressed to silence any active buzzer alarm.
6.  **Status Indicators:**
    *   **Multi-color LED Status Indicator:** A multi-color LED (controlled via a 74HC595 shift register) for quick visual status:
    *   **Red:** Critical alarms (e.g., dry soil, very high chamber temperature).
    *   **Yellow:** Warnings (e.g., high water temperature, moderately high chamber temperature).
    *   **Green:** Normal operation.
    *   **TM1638 LEDs:** A "bar graph" of red LEDs on the TM1638 module indicates system status: 1 LED for Normal, 2 LEDs for Warning, and 3 blinking LEDs for Critical.
7.  **Cooling Control:** Manages a Peltier cooler via a relay and includes a safety shutdown sequence if temperatures become too high.

**Configuration:** All timing, temperature thresholds, and feature settings are centralized in the `config.h` file for easy modification.

## Future Plans
*   **Automated Water Pump and Fan Control:** The water pump and fans are currently connected directly to the power supply as a safety precaution. This ensures the Peltier cooler never runs without the pump and fans, even in the event of a software or relay failure.
    *   **Simple Automation Method:** For basic automation, the pump and fan relay could be triggered by the same signal as the Peltier relay (output `Q3` from the 74HC595). This would turn all cooling components on and off together. **Warning:** This method still carries a risk. If the pump/fan relay were to fail while the Peltier relay works, the Peltier could run without cooling, leading to overheating.
    *   **Advanced Integration:** A more robust solution involves connecting the pump and fan relay to a separate shift register output (e.g., `Q5`) and implementing independent software control. This would allow for features like running the fans/pump for a short period after the Peltier shuts off to dissipate residual heat.
*   **Automated two-bucket swap system:** Since a heated water bucket cools down when not used for some time, we can have 2 water reservoirs being used alternatively based on water temperature. An alarm and human intervention will only be needed if both reservoirs have high temperatures.