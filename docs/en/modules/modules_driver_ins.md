# Modules Reference: Ins (Driver)

## MicroStrain

Source: [drivers/ins/microstrain](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/ins/microstrain)

### Description

MicroStrain by HBK Inertial Sensor Driver.
Currently supports the following sensors:

-[CV7-AR](https://www.hbkworld.com/en/products/transducers/inertial-sensors/vertical-reference-units--vru-/3dm-cv7-ar) -[CV7-AHRS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/attitude-and-heading-reference-systems--ahrs-/3dm-cv7-ahrs) -[CV7-INS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/inertial-navigation-systems--ins-/3dm-cv7-ins) -[CV7-GNSS/INS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/inertial-navigation-systems--ins-/3dm-cv7-gnss-ins)

This driver is not included in the firmware by default.
Include the module in firmware by setting the
[kconfig](../hardware/porting_guide_config.md#px4-board-configuration-kconfig) variables:
`CONFIG_DRIVERS_INS_MICROSTRAIN` or `CONFIG_COMMON_INS`.

### Examples

Attempt to start driver on a specified serial device.
The driver will choose /dev/ttyS4 by default if no port is specified

```
microstrain start -d /dev/ttyS1
```

Stop driver

```
microstrain stop
```

### Usage {#MicroStrain_usage}

```
MicroStrain <command> [arguments...]
 Commands:
   start         Start driver
     [-d <val>]  Port
                 values: <file:dev>, default: /dev/ttyS4

   stop          Stop driver

   status        Driver status
```

## ilabs

Source: [drivers/ins/ilabs](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/ins/ilabs)

### Description

Serial bus driver for the ILabs sensors.

Most boards are configured to enable/start the driver on a specified UART using the SENS_ILABS_CFG.
After that you can use the ILABS_MODE parameter to config outputs:

- Only raw sensor output (the default).
- Sensor output and INS data such as position and velocity estimates.

Setup/usage information: https://docs.px4.io/main/en/sensor/inertiallabs.html

### Examples

Attempt to start driver on a specified serial device.

```
ilabs start -d /dev/ttyS1
```

Stop driver

```
ilabs stop
```

### Usage {#ilabs_usage}

```
ilabs <command> [arguments...]
 Commands:
   start         Start driver
     -d <val>    Serial device

   status        Driver status

   stop          Stop driver

   status        Print driver status
```

## vectornav

Source: [drivers/ins/vectornav](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/ins/vectornav)

### Description

Serial bus driver for the VectorNav VN-100, VN-200, VN-300.

Most boards are configured to enable/start the driver on a specified UART using the SENS_VN_CFG parameter.

Setup/usage information: https://docs.px4.io/main/en/sensor/vectornav.html

### Examples

Attempt to start driver on a specified serial device.

```
vectornav start -d /dev/ttyS1
```

Stop driver

```
vectornav stop
```

### Usage {#vectornav_usage}

```
vectornav <command> [arguments...]
 Commands:
   start         Start driver
     -d <val>    Serial device

   status        Driver status

   stop          Stop driver

   status        Print driver status
```
