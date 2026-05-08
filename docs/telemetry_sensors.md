# Telemetry Sensors

This document lists all sensors supported by MeshCore's telemetry subsystem
(`EnvironmentSensorManager`). Each sensor is gated behind a build flag
(`ENV_INCLUDE_*`) and reports readings into a `CayenneLPP` payload via
`querySensors()`. Reporting is gated by the requester's
`TELEM_PERM_ENVIRONMENT` / `TELEM_PERM_LOCATION` permission bits.

All implementations live in `src/helpers/sensors/EnvironmentSensorManager.cpp`.

## Temperature / Humidity / Pressure

| Sensor       | Build flag            | I²C addr | Readings reported                                              |
| ------------ | --------------------- | -------- | -------------------------------------------------------------- |
| AHT10 / AHT20 | `ENV_INCLUDE_AHTX0`   | `0x38`   | temperature, humidity                                          |
| BME280       | `ENV_INCLUDE_BME280`  | `0x76`   | temperature, humidity, pressure, altitude                      |
| BMP280       | `ENV_INCLUDE_BMP280`  | `0x76`   | temperature, pressure, altitude                                |
| BME680       | `ENV_INCLUDE_BME680`  | `0x76`   | temperature, humidity, pressure, altitude, gas resistance      |
| BMP085       | `ENV_INCLUDE_BMP085`  | `0x77`   | temperature, pressure, altitude                                |
| SHTC3        | `ENV_INCLUDE_SHTC3`   | `0x70`   | temperature, humidity                                          |
| SHT4X        | `ENV_INCLUDE_SHT4X`   | `0x44`   | temperature, humidity                                          |
| LPS22HB      | `ENV_INCLUDE_LPS22HB` | `0x5C`   | temperature, pressure                                          |
| MLX90614     | `ENV_INCLUDE_MLX90614`| `0x5A`   | IR object temperature, ambient temperature                     |

## Power / Current

| Sensor   | Build flag            | I²C addr | Channels | Readings reported          |
| -------- | --------------------- | -------- | -------- | -------------------------- |
| INA219   | `ENV_INCLUDE_INA219`  | `0x40`   | 1        | voltage, current, power    |
| INA226   | `ENV_INCLUDE_INA226`  | `0x44`   | 1        | voltage, current, power    |
| INA260   | `ENV_INCLUDE_INA260`  | `0x41`   | 1        | voltage, current, power    |
| INA3221  | `ENV_INCLUDE_INA3221` | `0x42`   | 3        | voltage, current, power    |

## Distance

| Sensor  | Build flag           | I²C addr | Readings reported          |
| ------- | -------------------- | -------- | -------------------------- |
| VL53L0X | `ENV_INCLUDE_VL53L0X`| `0x29`   | time-of-flight distance    |

## Soil

| Sensor   | Build flag            | I²C addr | Readings reported                                |
| -------- | --------------------- | -------- | ------------------------------------------------ |
| RAK12035 | `ENV_INCLUDE_RAK12035`| `0x20`   | soil moisture, temperature                       |

The RAK12035 also exposes a calibration channel (capacitance, wet/dry
reference points) when `ENABLE_RAK12035_CALIBRATION` is defined.

## GPS / Location

| Provider                       | Build flag                          | Interface | Notes                              |
| ------------------------------ | ----------------------------------- | --------- | ---------------------------------- |
| Generic NMEA GPS               | `ENV_INCLUDE_GPS`                   | UART      | Uses `MicroNMEALocationProvider`   |
| RAK12500 (u-blox GNSS)         | `ENV_INCLUDE_GPS` + `RAK_BOARD`     | I²C `0x42`| Auto-probes RAK base-board sockets |

GPS readings are gated by `TELEM_PERM_LOCATION` instead of
`TELEM_PERM_ENVIRONMENT`.
