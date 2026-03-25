RS-485 Modbus Wind Sensor Task
==============================

This directory contains a user task that reads wind speed and direction from an RS-485 Modbus wind sensor and sends the data as NMEA0183 MWV sentences.

## Supported Sensor

**Veinasa XS-WSDS01** — Small Integrated Wind Speed and Direction Sensor (Polycarbonate)

| Parameter                  | Value                                   |
|----------------------------|-----------------------------------------|
| Wind Speed Range           | 0–30 m/s (0–60 m/s customizable)        |
| Wind Direction Range       | 0–360°                                  |
| Starting Wind Speed        | ≤ 0.3 m/s                               |
| Accuracy                   | ±(0.3 + 0.03V) m/s, ±1°                |
| Response Time              | < 1 second                              |
| Power Supply (RS485 mode)  | 5–24V DC                                |
| Communication              | RS485 Modbus-RTU, 9600 baud, 8N1        |
| Default Modbus Address     | 0x01                                    |
| Working Environment        | 0–50°C, < 95% RH                        |
| Dimensions                 | 202 × 193 × 101 mm (+ 180 mm arm)       |

The sensor manual is available at `doc/Veinasa XS-WSDS01 Small Integrated Wind Speed and Direction Sensor.pdf`.

The code for reading the sensor has been adapted from [RS485_Arduino_Wind_Direction_Speed_Sensors](https://github.com/wilson-malone/RS485_Arduino_Wind_Direction_Speed_Sensors).

## Required Hardware

You need an RS-485 transceiver to connect the sensor to an ESP32. The reference setup uses:

- **WeMos D1 Mini ESP32** (or compatible ESP32 board)
- **[TaaraLabs RS-485+CAN Bus+DCDC Shield](https://taaralabs.eu/category/modbus/)** for WeMos D1 Mini32
  - RS-485 transceiver: THVD1406 with automatic flow control
  - CAN bus transceiver (for NMEA2000)
  - DC/DC converter: 3.8V–32V input (12VDC nominal)

## Sensor Wiring

The XS-WSDS01 has 4 wires in RS485 mode:

| Wire Color | Function   |
|------------|------------|
| Red        | Power+ (5–24V DC) |
| Black      | GND        |
| Yellow     | RS485 A    |
| Green      | RS485 B    |

Connect the yellow and green wires to the RS-485 transceiver's A and B terminals respectively. Power the sensor with 5–24V DC on the red/black wires.

```
XS-WSDS01 Sensor          RS-485 Shield            ESP32
+---------------+        +---------------+        +-------+
| Yellow (A)    |--------| A             |        |       |
| Green  (B)    |--------| B          TX |--------| RX 16 |
| Red   (V+)  -|--+     |            RX |--------| TX 17 |
| Black (GND) -|--+-----| GND       VCC |--------| 3.3V  |
+---------------+  |     +---------------+        +-------+
                   |
              5-24VDC supply
```

## Pin Configuration

The default pins (matching the TaaraLabs RS-485+CAN shield for D1 Mini32):

| Function       | GPIO | Build Flag              |
|----------------|------|-------------------------|
| RS-485 RX      | 16   | `GWWINDSENSORMODBUS_RX` |
| RS-485 TX      | 17   | `GWWINDSENSORMODBUS_TX` |
| CAN bus RX     | 19   | `ESP32_CAN_RX_PIN`      |
| CAN bus TX     | 18   | `ESP32_CAN_TX_PIN`      |

To override the default RS-485 pins, set the build flags in your `platformio.ini`:

```ini
build_flags =
    -D GWWINDSENSORMODBUS_RX=16
    -D GWWINDSENSORMODBUS_TX=17
```

## Serial Port Selection

The task automatically selects an available serial port:

- If `GWSERIAL_TYPE`/`GWSERIAL_MODE` is **not** defined (Serial1 is free) → uses **Serial1**
- If Serial1 is used by the core but Serial2 is free → uses **Serial2**
- If both serial ports are used → **compile-time error**

To explicitly select a serial port (e.g., on S3 boards with 3 UARTs), use the `GWWINDSENSOR_SERIAL` build flag:

```ini
build_flags =
    -D GWWINDSENSOR_SERIAL=2
```

## Modbus Protocol

The sensor uses standard Modbus-RTU. The task queries 4 holding registers starting at address 0x0000:

**Query (host → sensor):**

| Byte  | Value  | Description             |
|-------|--------|-------------------------|
| 0     | 0x01   | Sensor address          |
| 1     | 0x03   | Function code (read)    |
| 2–3   | 0x0000 | Start register address  |
| 4–5   | 0x0004 | Number of registers     |
| 6–7   |        | CRC16                   |

**Response (sensor → host):**

| Register | Content             | Unit / Note                   |
|----------|---------------------|-------------------------------|
| 0        | Wind Speed          | × 0.1 m/s                    |
| 1        | Wind Scale          | Beaufort (0–17)               |
| 2        | Wind Direction Angle| × 0.1 degrees                |
| 3        | Wind Direction      | 0x00=N, 0x01=NNE, ... 0x0F=NNW (16 compass points) |

## NMEA0183 Output

The task sends **MWV** (Wind Speed and Angle) sentences using the NMEA0183 library's `NMEA0183SetMWV()` function:

- **Wind angle**: 0–360° (apparent/relative), from sensor register 2 (× 0.1°)
- **Wind speed**: m/s, from sensor register 0 (× 0.1 m/s)
- **Unit**: "M" (meters per second) — as per the NMEA0183 library convention
- **Reference**: "R" (apparent wind, relative to vessel)
- **Talker ID**: "WI" (weather instrument)

Example output: `$WIMWV,135.0,R,3.6,M,A*hh`

## Configuration

The task adds two configuration items (via `windconfig.json`, included via `custom_config`):

- **mbEnable** — Enable/disable the Modbus wind sensor (default: false)
- **mbAddr** — Modbus address of the sensor (default: 1)
