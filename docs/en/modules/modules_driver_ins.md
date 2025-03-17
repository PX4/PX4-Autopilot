# Modules Reference: Ins (Driver)
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

<a id="vectornav_usage"></a>
### Usage
```
vectornav <command> [arguments...]
 Commands:
   start         Start driver
     -d <val>    Serial device

   status        Driver status

   stop          Stop driver

   status        Print driver status
```
