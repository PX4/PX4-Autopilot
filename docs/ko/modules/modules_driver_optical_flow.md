# Modules Reference: Optical Flow (Driver)

## thoneflow

Source: [drivers/optical_flow/thoneflow](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/optical_flow/thoneflow)

### 설명

Serial bus driver for the ThoneFlow-3901U optical flow sensor.

Most boards are configured to enable/start the driver on a specified UART using the SENS_TFLOW_CFG parameter.

Setup/usage information: https://docs.px4.io/main/en/sensor/pmw3901.html#thone-thoneflow-3901u

### 예

지정된 직렬 장치에서 드라이버를 시작하려고 합니다.

```
thoneflow start -d /dev/ttyS1
```

드라이버를 중지합니다.

```
thoneflow stop
```

<a id="thoneflow_usage"></a>

### 사용법

```
thoneflow <command> [arguments...]
 Commands:
   start         Start driver
     -d <val>    Serial device

   stop          Stop driver

   info          Print driver information
```
