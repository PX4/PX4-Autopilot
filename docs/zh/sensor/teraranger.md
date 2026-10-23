# Terabee TeraRanger Rangefinders (Discontinued)

<Badge type="tip" text="PX4 v1.0" /> <Badge type="info" text="Discontinued" />

:::warning
TeraRanger Evo sensors were discontinued by Terabee in May 2024.
Limited stock may still be available from third-party resellers (see [Where to Buy](#store)).
:::

Terabee TeraRanger sensors are small, lightweight [rangefinders](../sensor/rangefinders.md) based on infrared Time-of-Flight (ToF) technology, with maximum ranges from 3 m to 60 m (depending on the model).
They connect to the flight controller using I2C.

:::info
The driver for these rangefinders is not included in PX4 firmware by default.
See [Firmware Support](#firmware_support) for how to add it.
:::

## Supported Models {#supported_models}

PX4 supports the following models.

| Model                | 范围                       | Current draw | [SENS\_EN\_TRANGER][SENS_EN_TRANGER] | 备注                                                   |
| -------------------- | ------------------------ | ------------ | ------------------------------------ | ---------------------------------------------------- |
| TeraRanger Evo 60m   | 0.5–60 m | 90–330 mA    | `3`                                  | Requires the Evo I2C/UART backboard. |
| TeraRanger Evo 600Hz | 0.75–8 m | 95–190 mA    | `4`                                  | Requires the Evo I2C/UART backboard. |
| TeraRanger Evo 3m    | 0.1–3 m  | 70–250 mA    | `5`                                  | Requires the Evo I2C/UART backboard. |
| TeraRanger One       | 0.2–14 m |              | `2`                                  |                                                      |

[SENS_EN_TRANGER]: ../advanced_config/parameter_reference.md#SENS_EN_TRANGER

## Specifications {#specifications}

The following specifications apply to TeraRanger Evo sensors fitted with the Evo I2C/UART backboard.

- **Technology:** Infrared Time-of-Flight (LED)
- **Field of view:** Approx. 2° (Evo 60m)
- **Interface:** I2C (3.3 V logic, address `0x31`)
- **Input voltage:** 5 V DC ±5%
- **Dimensions:** Approx. 29 x 29 x 22 mm (sensor and backboard)
- **Weight:** 9 g (sensor) + 3 g (backboard)

## Where to Buy {#store}

TeraRanger sensors are no longer sold by Terabee.
At the time of writing the following are still listed by the reseller Tribotix:

- [TeraRanger Evo 60m](https://tribotix.com/product/teraranger-evo-60m/)
- [TeraRanger Evo 600Hz](https://tribotix.com/product/teraranger-evo-600hz/)
- [TeraRanger Evo 3m](https://tribotix.com/product/teraranger-evo-3m/)

Select the **I2C/UART** interface option when ordering (PX4 cannot use the USB backboard).

## Hardware Setup {#hardware_setup}

### Mounting {#mounting}

PX4 assumes that the sensor is facing down (the orientation cannot be configured).

For TeraRanger Evo sensors:

- Do not mount anything directly in front of the sensor, or within a cone of approximately ±15° around its optical axis.
- Within the first metre, avoid highly reflective objects within a cone of approximately ±45° around the optical axis.
- Do not add a cover in front of the sensor.
- Rotor blades in the field of view can affect the readings.

### Pinouts {#pinouts}

TeraRanger Evo sensors must be fitted with the Evo I2C/UART backboard, which has a single 9-pin Hirose DF13 connector.

| 针脚 | 信号  | 描述                                                             |
| -- | --- | -------------------------------------------------------------- |
| 1  | Tx  | UART transmit (3.3 V logic) |
| 2  | Rx  | UART receive (3.3 V logic)  |
| 3  | GND | Ground                                                         |
| 4  | SDA | I2C data (3.3 V logic)      |
| 5  | SCL | I2C clock (3.3 V logic)     |
| 6  | -   | Reserved                                                       |
| 7  | 5V  | +5 V supply input                                              |
| 8  | GND | Ground                                                         |
| 9  | -   | Reserved                                                       |

### Wiring {#wiring}

All TeraRanger sensors must be connected via the I2C bus, using an external `I2C` port on the flight controller.

| Backboard pin                   | Connect to                |
| ------------------------------- | ------------------------- |
| 4 (SDA)      | Flight controller I2C SDA |
| 5 (SCL)      | Flight controller I2C SCL |
| 7 (5V)       | 5 V DC supply             |
| 3 or 8 (GND) | Flight controller GND     |

The UART pins are not used by PX4.

The sensor requires a 5 V DC (±5%) supply, and can draw up to 330 mA depending on the [supported model](#supported_models).
Check that the flight controller port can supply this, or power the sensor from a separate supply that shares a ground with the flight controller.

The backboard has built-in 10 kΩ pull-up resistors on SDA and SCL, so do not add further pull-ups on the same bus.
The sensor must be left at its default I2C address (`0x31`).

## Firmware Support {#firmware_support}

The rangefinder is supported by the [teraranger](../modules/modules_driver_distance_sensor.md#teraranger) driver, which is built into firmware by the [board configuration](../hardware/porting_guide_config.md) key `CONFIG_DRIVERS_DISTANCE_SENSOR_TERARANGER=y`.

The driver is not part of the common set of rangefinder drivers (`CONFIG_COMMON_DISTANCE_SENSOR`), so it is only present in the firmware for boards that set that key.

To check your board, look for the key in its configuration file, `boards/<vendor>/<board>/default.px4board`, or check whether [SENS\_EN\_TRANGER][SENS_EN_TRANGER] is listed in _QGroundControl_ (parameters are only shown for drivers that are in the firmware).

To add the driver, add the key to the board's `default.px4board` file, or enable it in [boardconfig](../hardware/porting_guide_config.md#px4-menuconfig-setup) under **Drivers > Distance sensors**, and then [build and upload the firmware](../dev_setup/building_px4.md).

The driver does not start until it is enabled using the parameter in [PX4 Configuration](#px4_configuration).

## PX4 Configuration {#px4_configuration}

Set [SENS\_EN\_TRANGER][SENS_EN_TRANGER] to the value for your sensor in the [Supported Models](#supported_models) table, and then reboot.

You can instead set the parameter to `1` to have PX4 detect the sensor.

:::info
If using auto-detect for Evo sensors the minimum and maximum values for the range are set to the lowest and highest possible readings across the Evo family (currently 0.1–60 m).
In order to use the correct max/min values the appropriate model of the Evo sensor should be set in the parameter (instead of using autodetect).
:::

If the parameter is not available in _QGroundControl_ then the driver is not in the firmware: see [Firmware Support](#firmware_support).

The configuration that is common to all rangefinders, such as the sensor position offsets used by the estimator, is covered in [Rangefinders > Configuration/Setup](../sensor/rangefinders.md#configuration).

## Testing {#testing}

You can check that the driver is running, and see its error counters, using the _QGroundControl MAVLink Console_:

```sh
teraranger status
```

You can observe the published measurements with:

```sh
listener distance_sensor
```

The measurements can also be viewed in the `DISTANCE_SENSOR` message in the _QGroundControl MAVLink Inspector_.
For more information see [Rangefinders > Testing](../sensor/rangefinders.md#testing).

## Further Information {#further_information}

- [Modules Reference: Distance Sensor (Driver): teraranger](../modules/modules_driver_distance_sensor.md#teraranger)
- [User Manual for TeraRanger Evo single point distance sensors and backboards](https://web.archive.org/web/20240611084754/http://www.terabee.com/wp-content/uploads/2021/02/User-Manual-for-TeraRanger-Evo-single-point-distance-sensors-and-backboards.pdf) (Terabee, archived copy)
- [TeraRanger Evo Specification Sheet](https://acroname.com/sites/default/files/assets/teraranger-evo-specification-sheet.pdf) (Terabee, hosted by Acroname)
