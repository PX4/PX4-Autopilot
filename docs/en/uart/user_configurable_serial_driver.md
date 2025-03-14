# Making Serial Port Drivers User-Configurable

This topic explains how to set up a serial driver so that it can be end-user configured (via parameters) to run on any of a flight controller board's configurable serial ports.

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

     ```cmake
     module_name: uLanding Radar
     serial_config:
         - command: ulanding_radar start -d ${SERIAL_DEV} -b p:${BAUD_PARAM}
           port_config_param:
             name: SENS_ULAND_CFG
             group: Sensors
     ```

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
