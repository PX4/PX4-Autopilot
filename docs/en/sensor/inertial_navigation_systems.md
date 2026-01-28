# IMU, AHRS, and INS

PX4 typically runs on flight controllers that include an IMU, such as the Pixhawk series, and fuse the sensor data along with GNSS information in the EKF2 estimator to determine vehicle attitude, heading, position, and velocity.

However PX4 can also use some INS devices as either sources of raw data, or as an external estimator, replacing EKF2.

## Supported INS Systems

INS systems that can be used as a replacement for EKF2 in PX4:

- [InertialLabs](../sensor/inertiallabs.md)
- [MicroStrain](../sensor/microstrain.md): Includes VRU, AHRS, INS, and GNSS/INS devices.
- [SBG Systems](../sensor/sbgecom.md): IMU/AHRS, GNSS/INS, Dual GNSS/INS systems that can be used as an external INS or as a source of raw sensor data.
- [VectorNav](../sensor/vectornav.md): IMU/AHRS, GNSS/INS, Dual GNSS/INS systems that can be used as an external INS or as a source of raw sensor data.

## PX4 Firmware

The driver module for your INS system may not be included in the PX4 firmware for your flight controller by default.

You can check by searching the [default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6c/default.px4board#L25) configuration file for your target board for either:

- `CONFIG_COMMON_INS`, which includes drivers for [all INS systems](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/ins/Kconfig).
- The key for the particular INS system you are using, such as:
  - `CONFIG_DRIVERS_INS_ILABS`
  - `CONFIG_DRIVERS_INS_MICROSTRAIN`
  - `CONFIG_DRIVERS_INS_VECTORNAV`

If the required key is not present you can include the module in firmware by adding the key to the `default.px4board` file, or using the [kconfig board configuration tool](../hardware/porting_guide_config.md#px4-board-configuration-kconfig) and then select the driver you want (`Drivers -> INS`).
Note that if you're working on a flight controller where flash memory is limited, you're better off installing just the modules you need.

You will then need to rebuild the firmware.

## Glossary

### Inertial Measurement Unit (IMU)

An IMU is a device that contains a 3-axis accelerometer (motion sensor), 3-axis gyroscope (rotation sensor), and sometimes a magnetometer (compass).
The gyroscope and accelerometer provide raw sensor data that allow measurement of a system's angular rate and acceleration.
The magnetometer, if present, provides sensor data that can be used to provide the direction that the vehicle is facing/heading.

### Attitude Heading Reference System (AHRS)

An AHRS system that includes both an IMU and a processing system that can provide attitude and heading information from the IMUs raw data.

### Inertial Navigation System (INS)

An INS is a navigation device that uses accelerometers, gyroscopes, possibly magnetometers, and a computer to calculate the attitude, position, and velocity of a moving object without the need for external references.
Essentially it is an AHRS that also includes position/velocity estimation.

## Further Information

- [What is an Inertial Navigation System?](https://www.vectornav.com/resources/inertial-navigation-articles/what-is-an-ins) (VectorNav)
- [Inertial Navigation Primer](https://www.vectornav.com/resources/inertial-navigation-primer) (VectorNav)
