# Modules Reference: Ins (Driver)

## vectornav

Source: [drivers/ins/vectornav](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/ins/vectornav)

### 설명

Serial bus driver for the VectorNav VN-100, VN-200, VN-300.

Most boards are configured to enable/start the driver on a specified UART using the SENS_VN_CFG parameter.

Setup/usage information: https://docs.px4.io/main/en/sensor/vectornav.html

### 예

지정된 직렬 장치에서 드라이버를 시작하려고 합니다.

```
vectornav start -d /dev/ttyS1
```

드라이버를 중지합니다.

```
vectornav stop
```

<a id="vectornav_usage"></a>

### 사용법

```
vectornav <command> [arguments...]
 Commands:
   start         Start driver
     -d <val>    Serial device

   status        Driver status

   stop          Stop driver

   status        Print driver status
```
