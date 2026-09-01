# Making Serial Port Drivers User-Configurable

This topic explains how to register a serial driver as a protocol, so that an end user can select it on any of a flight controller board's configurable serial ports.

## Preconditions

The driver is assumed to already exist, and be started in the shell using the command syntax:

```sh
<driver_name> start -d <serial_port> [-b <baudrate> | -b p:<param_name>]
```

where,

- `-d`: serial port name.
- `-b`: Baud rate (optional) if the driver supports multiple baud rates.
  If supported, the driver must allow you to specify the rate as both a bare baudrate and as a parameter name in the form `-b p:<param_name>` (which can be parsed with `px4_get_parameter_value()`).
  :::tip
  See the [gps driver](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/gps/gps.cpp#L1023) for an example.
  :::

## Making the Driver Configurable

To make driver configurable:

1. Create a YAML module configuration file:
   - Add a new file in the driver's source directory named **module.yaml**
   - Insert the following text and adjust as needed:

     ```yaml
     module_name: uLanding Radar
     serial_config:
         - command: ulanding_radar start -d ${SERIAL_DEV} -b p:${BAUD_PARAM}
           protocol_id: 35
           protocol_name: uLanding Radar
     ```

     `protocol_id` is the value a `SER_<tag>_PROTO` parameter takes to select this driver.
     Pick an integer that no other **module.yaml** uses, and never reuse or renumber one: stored parameter values refer to it.
     `protocol_name` is the label shown in the parameter metadata, and defaults to `module_name`.

     Optional keys: `default` names a serial tag (`TEL2`, `GPS1`, …) the protocol occupies out of the box; `secondary_command` (dual GPS) is expanded with the second port's device and baud and substituted for `${DUAL_GPS_ARGS}` in the single `command` run for the first port; `num_instances` > 1 without `secondary_command` runs `command` once per port with `${i}` set, which only MAVLink uses.

     ::: info
     The full documentation of the module configuration file can be found in the [validation/module_schema.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/validation/module_schema.yaml) file.
     This is also used to validate all configuration files in CI.
     :::

1. Add the module configuration to the **CMakeLists.txt** file for the driver module:

   ```cmake
   px4_add_module(
   	MODULE drivers__ulanding
   	MAIN ulanding_radar
   	SRCS
   		ulanding.cpp
   	MODULE_CONFIG
   		module.yaml
   	)
   ```
