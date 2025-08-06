# Modules Reference: Ins (Driver)

## ilabs

Source: [drivers/ins/ilabs](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/ins/ilabs)

### Опис

Serial bus driver for the ILabs sensors.

Most boards are configured to enable/start the driver on a specified UART using the SENS_ILABS_CFG.
After that you can use the ILABS_MODE parameter to config outputs:

- Only raw sensor output (the default).
- Sensor output and INS data such as position and velocity estimates.

Setup/usage information: https://docs.px4.io/main/en/sensor/ilabs.html

### Приклади

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

### Опис

Драйвер послідовної шини для VectorNav VN-100, VN-200, VN-300.

Більшість плат налаштовано на увімкнення/запуск драйвера на вказаному UART за допомогою параметра SENS_VN_CFG.

Інформація про налаштування/використання: https://docs.px4.io/main/en/sensor/vectornav.html

### Приклади

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
