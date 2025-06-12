# 使串口驱动为用户可配置。

本主题介绍了如何设置一个串行驱动程序，使其能够被最终用户配置(通过参数) 在飞行控制器板的任何可配置串行端口上运行。

## 操作前提

假定驱动程序已经存在，并使用命令语法在shell中启动:

```sh
<driver_name> start -d <serial_port> [-b <baudrate> | -b p:<param_name>]
```

上述命令中：

- `-d`: serial port name.
- `-b`: Baud rate (optional) if the driver supports multiple baud rates.
  If supported, the driver must allow you to specify the rate as both a bare baudrate and as a parameter name in the form `-b p:<param_name>` (which can be parsed with `px4_get_parameter_value()`).
  :::tip
  See the [gps driver](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/gps/gps.cpp#L1023) for an example.

:::

## 使驱动程序可配置

使驱动程序可配置：

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
