# Готуємо драйвер Послідовного Порту до користувацьких налаштувань

Ця тема пояснює, як підготувати драйвер послідовного порту для того, щоб користувач міг налаштувати його (через параметри) для роботи на послідовних портах будь-якого політного контролера.

## Передумови

Очікується, що драйвер вже існує і запускається в командному рядку, використовуючи синтаксис команди:

```sh
<driver_name> start -d <serial_port> [-b <baudrate> | -b p:<param_name>]
```

де

- `-d`: serial port name.
- `-b`: Baud rate (optional) if the driver supports multiple baud rates.
  If supported, the driver must allow you to specify the rate as both a bare baudrate and as a parameter name in the form `-b p:<param_name>` (which can be parsed with `px4_get_parameter_value()`).
  :::tip
  See the [gps driver](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/gps/gps.cpp#L1023) for an example.

:::

## Робимо драйвер налаштовуваним

Аби зробити драйвер налаштовуваним:

1. Створіть конфігураційний файл модуля YAML:

  - Add a new file in the driver's source directory named **module.yaml**
  - Вставте наступний текст і підлаштуйте за потреби:

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
    Це також використовується для перевірки всіх файлів конфігурації в CI.

:::

2. Add the module configuration to the **CMakeLists.txt** file for the driver module:

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
