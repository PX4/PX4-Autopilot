# Magnetometer (Compass) Hardware & Setup

PX4 uses a magnetometer (compass) for determining the yaw and heading of the vehicle relative to the earth's magnetic field.

[Pixhawk series](../flight_controller/pixhawk_series.md) flight controllers, and many others, include an [internal compass](#internal-compass).
This is used for automatic rotation detection of external magnetometers and for autopilot bench testing.
It should not be used otherwise, and is automatically disabled after [calibration](../config/compass.md) if an external compass is available.

On most vehicles, and in particular on small vehicles, we recommend using a _combined GPS + Compass_ [mounted as far away from the motor/ESC power supply lines as possible](../assembly/mount_gps_compass.md) - typically on a pedestal or wing (for fixed-wing).
While you can use a [stand-alone external compass](#stand-alone-compass-modules) (as listed below) it is far more common to use a [combined GPS/Compass module](#combined-gnss-compass-modules).

Magnetometers support connection to either the I2C/SPI-bus (Pixhawk `GPS1` or `GPS2` ports) or to the CAN bus.
If a module doesn't include "CAN" in its product name then it is probably an I2C/SPI compass.

Up to 4 internal or external magnetometers can be connected, though only one will actually be used as a heading source.
The system automatically chooses the best available compass based on their _priority_ (external magnetometers have a higher priority than internal magnetometers).
If the primary compass fails in-flight, it will failover to the next one.
If it fails before flight, arming will be denied.

## Supported Compasses

### Compass Parts

PX4 can be used with many magnetometer parts, including: Bosch BMM 150 MEMS (via I2C bus), HMC5883 / HMC5983 (I2C or SPI), IST8310 (I2C), LIS3MDL (I2C or SPI), RM3100, and more.
Other supported magnetometer parts and their busses can be inferred from the drivers listed in [Modules Reference: Magnetometer (Driver)](../modules/modules_driver_magnetometer.md).

These parts are included in stand alone compass modules, combined compass/GNSS modules, and also in many flight controllers,

### Combined GNSS/Compass Modules

See [Global Navigation Satellite Systems (GNSS)](../gps_compass/index.md#supported-gnss) for a list of appropriate modules.

::: info
If GNSS is required, then a combined GNSS/Compass module will be preferred over the stand-alone modules below.
:::

### Stand-Alone Compass Modules

This list contains stand-alone magnetometer modules (without GNSS).

| Device                                                                                                           | Compass | DroneCan |
| :--------------------------------------------------------------------------------------------------------------- | :-----: | :------: |
| [Avionics Anonymous UAVCAN Magnetometer](https://www.tindie.com/products/avionicsanonymous/uavcan-magnetometer/) |    ?    |          |
| [Holybro DroneCAN RM3100 Compass/Magnetometer](https://holybro.com/products/dronecan-rm3100-compass)             | RM3100  |    ✓     |
| [RaccoonLab DroneCAN/Cyphal Magnetometer RM3100](https://holybro.com/products/dronecan-rm3100-compass)           | RM3100  |    ✓     |

Note:

- ✓ or a specific part number indicate that a features is supported, while ✘ or empty show that the feature is not supported.
  "?" indicates "unknown".
- A compass that is not "DroneCAN" can be assumed to be SPI or I2C.

### Internal Compass

Internal compasses are not recommended for real use as a heading source, because the performance is almost always very poor.

This is particularly true on on small vehicles where the flight controller has to be mounted close to motor/ESC power lines and other sources of electromagnetic interference.
While they may be better on larger vehicles (e.g. VTOL), where it is possible to reduce electromagnetic interference by mounting the flight controller a long way from power supply lines, an external compass will almost always be better.

::: tip
They might in theory be used if there is no external magnetometer. but only with [EKF2_MAG_TYPE_INIT = Init (`6`)](../advanced_config/parameter_reference.md#EKF2_MAG_TYPE), and only if their measurements are roughly ok before arming.
:::

Internal compasses are disabled by default if an external compass is available.

## Mounting

[Mounting the Compass](../assembly/mount_gps_compass.md) explains how to mount a compass or GPS/Compass module.

## I2C/SPI Compass Setup

On [Pixhawk Series](../flight_controller/pixhawk_series.md) flight controllers you can connect to either the `GPS1` or `GPS2` ports (which have pins for I2C/SPI).
No further configuration is required.

<!-- On flight controllers that do not follow the Pixhawk connector standard, you will need to connect to an I2C/SPI port. -->

## CAN Compass Setup

[DroneCAN](../dronecan/index.md) covers the setup for DroneCAN peripherals, including compasses.

You will need to connect the compass to the [CAN bus](../can/index.md#wiring), enable DroneCAN, and specifically enable magnetometers (search for `UAVCAN_SUB_MAG`).

## Calibration

[Compass Calibration](../config/compass.md) explains how to calibrate all compasses on the vehicle.

The process is straightforward and will autodetect, [set default rotations](../advanced_config/parameter_reference.md#SENS_MAG_AUTOROT), calibrate, and prioritise, all connected magnetometers.

## See also

- [Compass Power Compensation](../advanced_config/compass_power_compensation.md)
