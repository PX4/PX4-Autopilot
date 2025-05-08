# Modules Reference: Optical Flow (Driver)

## thoneflow

Source: [drivers/optical_flow/thoneflow](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/optical_flow/thoneflow)

### Опис

Драйвер послідовної шини для optical flow сенсору ThoneFlow-3901U.

Більшість плат налаштовано на ввімкнення/вимкнення драйвера на вказаному UART за допомогою параметра SENS_TFLOW_CFG.

Інформація про налаштування/використання: https://docs.px4.io/main/en/sensor/pmw3901.html#thone-thoneflow-3901u

### Приклади

Спроба запустити драйвер на вказаному послідовному пристрої.

```
thoneflow start -d /dev/ttyS1
```

Stop driver

```
thoneflow stop
```

<a id="thoneflow_usage"></a>

### Використання

```
thoneflow <command> [arguments...]
 Commands:
   start         Start driver
     -d <val>    Serial device

   stop          Stop driver

   info          Print driver information
```
