# 직렬 포트 드라이버 사용자 설정

비행 콘트롤러에서 설정 가능 직렬 포트에서 실행되도록 사용자가 (매개변수를 통하여) 구성할 수 있도록 직렬 드라이버를 설정하는 방법을 설명합니다.

## 전제 조건

제공되는 드라이버를 사용하여, 쉘에서 다음 명령어로 시작합니다.

```sh
<driver_name> start -d <serial_port> [-b <baudrate> | -b p:<param_name>]
```

여기서,

- `-d`: serial port name.
- `-b`: Baud rate (optional) if the driver supports multiple baud rates.
  If supported, the driver must allow you to specify the rate as both a bare baudrate and as a parameter name in the form `-b p:<param_name>` (which can be parsed with `px4_get_parameter_value()`).
  :::tip
  See the [gps driver](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/gps/gps.cpp#L1023) for an example.

:::

## 설정 가능 드라이버 제작

드라이버를 설정 가능하게 하려면:

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
