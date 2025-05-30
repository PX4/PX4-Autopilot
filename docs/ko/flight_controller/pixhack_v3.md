# Pixhack V3

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://store.cuav.net/) for hardware support or compliance issues.
:::

The CUAV _Pixhack V3_ flight controller board is a flexible autopilot intended primarily for manufacturers of commercial systems.

The board is a variant of the SOLO Pixhawk<sup>&reg;</sup> 2 (PH2) flight controller, which is in turn based on the [Pixhawk-project](https://pixhawk.org/) **FMUv3** open hardware design.
It runs PX4 on the [NuttX](https://nuttx.apache.org/) OS, and is fully compatible with both PX4 or ArduPilot<sup>&reg;</sup> (APM) firmware.

_Pixhack V3_ has significant improvements with respect to the original design, including better interface layout and the addition of vibration damping and a thermostat system.

![Pixhack v3](../../assets/flight_controller/pixhack_v3/pixhack_v3_157_large_default.jpg)

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## 요약

- 마이크로 프로세서:
  - STM32F427
  - STM32F100 (오류복구 코프로세서)
- 센서:
  - 가속도계 (3): LS303D, MPU6000, MPU9250/hmc5983
  - 자이로스코프 (3): L3GD20, MPU6000, MPU9250
  - 나침반 (2): LS303D, MPU9250
  - 기압계 (2): MS5611 X2
- 인터페이스:
  - MAVLink UART (2)
  - GPS UART (2)
  - 디버그 UART (1)
  - RC 입력(PPM, SBUS, DSM/DSM2 용)
  - RSSI 입력: PWM 또는 3.3ADC
  - I2C (2)
  - CAN 버스 (1)
  - ADC 입력: 3.3V X1 , 6.6V X1
  - PWM 출력: 8 PWM IO + 4 IO
- 전원시스템
  - PM 전원 입력: 4.5 ~ 5.5V
  - USB 전원 입력: 5.0V +- 0.25v
- 중량과 크기
  - 중량: 63g
  - 폭: 68mm
  - 두께: 17mm
  - 길이: 44mm
- 기타 특성:
  - 작동 온도: -20 ~ 60°c

## 구매처

보드는 아래에서 구입할 수 있습니다.

- [store.cuav.net](http://store.cuav.net/index.php?id_product=8&id_product_attribute=0&rewrite=pixhack-v3-autopilot&controller=product&id_lang=3)
- [leixun.aliexpress.com/store](https://leixun.aliexpress.com/store)

## 펌웨어 빌드

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make px4_fmu-v3_default
```

## 핀배열과 회로도

- [Documentation/wiring guides](http://doc.cuav.net/flight-controller/pixhack/en/pixhack-v3.html)

## 시리얼 포트 매핑

| UART   | 장치         | 포트                                |
| ------ | ---------- | --------------------------------- |
| UART1  | /dev/ttyS0 | IO 디버그                            |
| USART2 | /dev/ttyS1 | TELEM1 (흐름 제어) |
| USART3 | /dev/ttyS2 | TELEM2 (흐름 제어) |
| UART4  |            |                                   |
| UART7  | 콘솔         |                                   |
| UART8  | SERIAL4    |                                   |
