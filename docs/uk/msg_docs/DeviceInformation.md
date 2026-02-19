---
pageClass: is-wide-page
---

# DeviceInformation (UORB message)

Device information.

Can be used to uniquely associate a device_id from a sensor topic with a physical device using serial number.
as well as tracking of the used firmware versions on the devices.

**TOPICS:** device_information

## Fields

| Назва                                 | Тип        | Unit [Frame] | Range/Enum                                                                                                                                                              | Опис                                                                                                                            |
| ------------------------------------- | ---------- | ---------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                             | `uint64`   |                                                                  |                                                                                                                                                                         | time since system start (microseconds)                                                                       |
| device_type      | `uint8`    |                                                                  | [DEVICE_TYPE](#DEVICE_TYPE)                                                                                                                        | Type of the device. Matches MAVLink DEVICE_TYPE enum                                       |
| vendor_name      | `char[32]` |                                                                  |                                                                                                                                                                         | Name of the device vendor                                                                                                       |
| model_name       | `char[32]` |                                                                  |                                                                                                                                                                         | Name of the device model                                                                                                        |
| `uint32`                              |            |                                                                  | Unique device ID for the sensor. Does not change between power cycles. (Invalid: 0 if not available) |                                                                                                                                 |
| firmware_version | `char[24]` |                                                                  |                                                                                                                                                                         | Firmware version. (Invalid: empty if not available)                          |
| hardware_version | `char[24]` |                                                                  |                                                                                                                                                                         | Hardware version. (Invalid: empty if not available)                          |
| serial_number    | `char[33]` |                                                                  |                                                                                                                                                                         | Device serial number or unique identifier. (Invalid: empty if not available) |

## Enums

### DEVICE_TYPE {#DEVICE_TYPE}

| Назва                                                                                                                                              | Тип     | Значення | Опис                   |
| -------------------------------------------------------------------------------------------------------------------------------------------------- | ------- | -------- | ---------------------- |
| <a href="#DEVICE_TYPE_GENERIC"></a> DEVICE_TYPE_GENERIC                                                  | `uint8` | 0        | Generic/unknown sensor |
| <a href="#DEVICE_TYPE_AIRSPEED"></a> DEVICE_TYPE_AIRSPEED                                                | `uint8` | 1        | Датчик швидкості       |
| <a href="#DEVICE_TYPE_ESC"></a> DEVICE_TYPE_ESC                                                          | `uint8` | 2        | ESC                    |
| <a href="#DEVICE_TYPE_SERVO"></a> DEVICE_TYPE_SERVO                                                      | `uint8` | 3        | Servo                  |
| <a href="#DEVICE_TYPE_GPS"></a> DEVICE_TYPE_GPS                                                          | `uint8` | 4        | GPS                    |
| <a href="#DEVICE_TYPE_MAGNETOMETER"></a> DEVICE_TYPE_MAGNETOMETER                                        | `uint8` | 5        | Магнітометр            |
| <a href="#DEVICE_TYPE_PARACHUTE"></a> DEVICE_TYPE_PARACHUTE                                              | `uint8` | 6        | Парашут                |
| <a href="#DEVICE_TYPE_RANGEFINDER"></a> DEVICE_TYPE_RANGEFINDER                                          | `uint8` | 7        | Rangefinder            |
| <a href="#DEVICE_TYPE_WINCH"></a> DEVICE_TYPE_WINCH                                                      | `uint8` | 8        | Winch                  |
| <a href="#DEVICE_TYPE_BAROMETER"></a> DEVICE_TYPE_BAROMETER                                              | `uint8` | 9        | Барометр               |
| <a href="#DEVICE_TYPE_OPTICAL_FLOW"></a> DEVICE_TYPE_OPTICAL_FLOW                   | `uint8` | 10       | Optical flow           |
| <a href="#DEVICE_TYPE_ACCELEROMETER"></a> DEVICE_TYPE_ACCELEROMETER                                      | `uint8` | 11       | Accelerometer          |
| <a href="#DEVICE_TYPE_GYROSCOPE"></a> DEVICE_TYPE_GYROSCOPE                                              | `uint8` | 12       | Gyroscope              |
| <a href="#DEVICE_TYPE_DIFFERENTIAL_PRESSURE"></a> DEVICE_TYPE_DIFFERENTIAL_PRESSURE | `uint8` | 13       | Differential pressure  |
| <a href="#DEVICE_TYPE_BATTERY"></a> DEVICE_TYPE_BATTERY                                                  | `uint8` | 14       | Battery                |
| <a href="#DEVICE_TYPE_HYGROMETER"></a> DEVICE_TYPE_HYGROMETER                                            | `uint8` | 15       | Hygrometer             |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DeviceInformation.msg)

:::details
Click here to see original file

```c
# Device information
#
# Can be used to uniquely associate a device_id from a sensor topic with a physical device using serial number.
# as well as tracking of the used firmware versions on the devices.

uint64 timestamp  # time since system start (microseconds)

uint8 device_type  # [@enum DEVICE_TYPE] Type of the device. Matches MAVLink DEVICE_TYPE enum

uint8 DEVICE_TYPE_GENERIC = 0                 # Generic/unknown sensor
uint8 DEVICE_TYPE_AIRSPEED = 1                # Airspeed sensor
uint8 DEVICE_TYPE_ESC = 2                     # ESC
uint8 DEVICE_TYPE_SERVO = 3                   # Servo
uint8 DEVICE_TYPE_GPS = 4                     # GPS
uint8 DEVICE_TYPE_MAGNETOMETER = 5            # Magnetometer
uint8 DEVICE_TYPE_PARACHUTE = 6               # Parachute
uint8 DEVICE_TYPE_RANGEFINDER = 7             # Rangefinder
uint8 DEVICE_TYPE_WINCH = 8                   # Winch
uint8 DEVICE_TYPE_BAROMETER = 9               # Barometer
uint8 DEVICE_TYPE_OPTICAL_FLOW = 10           # Optical flow
uint8 DEVICE_TYPE_ACCELEROMETER = 11          # Accelerometer
uint8 DEVICE_TYPE_GYROSCOPE = 12              # Gyroscope
uint8 DEVICE_TYPE_DIFFERENTIAL_PRESSURE = 13  # Differential pressure
uint8 DEVICE_TYPE_BATTERY = 14                # Battery
uint8 DEVICE_TYPE_HYGROMETER = 15             # Hygrometer

char[32] vendor_name  # Name of the device vendor
char[32] model_name   # Name of the device model

uint32   device_id         # [-] [@invalid 0 if not available] Unique device ID for the sensor. Does not change between power cycles.
char[24] firmware_version  # [-] [@invalid empty if not available] Firmware version.
char[24] hardware_version  # [-] [@invalid empty if not available] Hardware version.
char[33] serial_number     # [-] [@invalid empty if not available] Device serial number or unique identifier.
```

:::
