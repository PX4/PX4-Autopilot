# LeddarOne 라이다

[LeddarOne](https://leddartech.com/solutions/leddarone/) is small Lidar module with a narrow, yet diffuse beam that offers excellent overall detection range and performance, in a robust, reliable, cost-effective package.
감지 범위는 1cm ~ 40m이며 UART/직렬 버스로 연결합니다.

<img src="../../assets/hardware/sensors/leddar_one.jpg" alt="LeddarOne Lidar rangefinder" width="200px" />

## 하드웨어 설정

LeddarOne can be connected to any unused _serial port_ (UART), e.g.: TELEM2, TELEM3, GPS2 etc.

보드 핀배열과 LeddarOne 핀배열(아래 참조)에 따라 적절한 케이블을 사용합니다. 5V, TX, RX 및 GND 핀만 연결하면 됩니다.

| 핀 | LeddarOne |
| - | --------- |
| 1 | GND       |
| 2 | -         |
| 3 | VCC       |
| 4 | RX        |
| 5 | TX        |
| 6 | -         |

## 매개변수 설정

[Configure the serial port](../peripherals/serial_configuration.md) on which the lidar will run using [SENS_LEDDAR1_CFG](../advanced_config/parameter_reference.md#SENS_LEDDAR1_CFG).
포트 전송속도는 드라이버에 의해 설정되므로, 추가로 설정하지 않아도 됩니다.

:::info
If the configuration parameter is not available in _QGroundControl_ then you may need to [add the driver to the firmware](../peripherals/serial_configuration.md#parameter_not_in_firmware):

```plain
CONFIG_DRIVERS_DISTANCE_SENSOR_LEDDAR_ONE=y
```

:::

## 추가 정보

- [LeddarOne Spec sheet](https://leddartech.com/app/uploads/dlm_uploads/2021/04/Spec-Sheet_LeddarOne_V10.0_EN-1.pdf)
