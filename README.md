# RS485 URM14 ultrasonic distance sensor on ESP32C6 XIAO → Zigbee2MQTT

Originally this project was an adaptation of the original **esp32c6_zigbee_ultrasonic_distance_sensor** by Aralox (many thanks to him) to support the RS485 version of the sensor.

**But the original A02 sensor was not suitable for my use case of measuring domestic heating oil in a tank, as its wide ultrasonic beam caused reflections on the tank walls, leading to inaccurate readings. I therefore switched to the DFRobot URM14 sensor, which has a much narrower beam and provides more reliable measurements.**
Aralox did all the work and this forked project is 80% adapted by AI, please check his repo for more informations. I'm not a developer, AI probably made things the wrong way, but it works for my hobbyist use case.

---

## Hardware
- XIAO ESP32-C6
- URM14 Ultrasonic Sensor (RS485)
- Seeed Studio XIAO-RS485-Expansion-Board
- Official external antenna (enabled in this code, modify it if you don't use it)

---

## Pin Mapping
| URM14 sensor wires | RS485-Expansion-Board |
|--------|------|
| Brown | 12V |
| Black | GND |
| Blue | B |
| White | A |


---

## Principle of Operation
- The firmware periodically reads the distance from the URM14 sensor using Modbus RTU (function 0x03, register 0x0005, 1 register).
- The measured distance is returned in 0.1 mm units.
- A smoothing filter is applied (moving average).
- The value is published over Zigbee using:
  - cluster: msIlluminanceMeasurement
  - attribute: measuredValue
- Zigbee2MQTT uses an external converter (tank_sensor.js) to expose this value
as a numeric distance sensor (millimeters).

This approach reuses a standard Zigbee cluster (uint16), ensuring compatibility
with Zigbee2MQTT and Home Assistant.

---

## Build and Flash (ESP-IDF)

### Requirements
- VSCode
- ESP-IDF **v5.0 or newer**
- ESP Zigbee SDK (automatically downloaded via the IDF component manager)

### In menuconfig :
Make sure to enable Zigbee as endpoint, and custom partition table pointing to partitions.csv 

---

## In zigbee2mqtt configuration.yaml folder :
Copy tank_sensor.js to zigbee2mqtt/external_converters and restart z2m before pairing.

---

## Default Measurement & Reporting Behavior

Measurement interval: 60 seconds
Reportable change: 1 mm
Values are smoothed using a moving average (5 samples)
