# RS485 A02 ultrasonic distance sensor ESP32C6 XIAO → Zigbee2MQTT

This project is an adaptation of the original **esp32c6_zigbee_ultrasonic_distance_sensor** by Aralox (many thanks to him)
to support the RS485 version of the sensor. Aralox did all the work and this forked project is 100% adapted by AI, please check his repo for more informations. I'm not a developer, AI probably made things the wrong way, but it works for my hobbyist use case.

---

## Hardware
- XIAO ESP32-C6
- A02 Ultrasonic Sensor (RS485)
- Seeed Studio XIAO-RS485-Expansion-Board
- External Zigbee antenna (enabled in this code, modify it if you don't use it)

---

## Pin Mapping
| Sensor wires | RS485-Expansion-Board |
|--------|------|
| Red | 5V |
| Black | GND |
| Yellow | B |
| White | A |


---

## Principle of Operation
- The firmware periodically reads the distance from the A02 sensor using **Modbus RTU**
  (function `0x03`, register `0x0101`, 1 register).
- The measured distance is published over Zigbee by updating the attribute
  `measuredValue` of the **Illuminance Measurement cluster** (`msIlluminanceMeasurement`).
- Zigbee2MQTT uses an **external converter** (`tank_sensor.js`) to expose this value
  as a numeric **distance sensor (millimeters)** in MQTT and Home Assistant.

This approach reuses a standard Zigbee cluster with a `uint16` attribute, ensuring
good compatibility and reliable reporting.

---

## Build and Flash (ESP-IDF)

### Requirements
- VSCode
- ESP-IDF **v5.0 or newer**
- ESP Zigbee SDK (automatically downloaded via the IDF component manager)

### In menuconfig :
Make sure to enable Zigbee as endpoint, and custom partition table pointing to partition.csv 
