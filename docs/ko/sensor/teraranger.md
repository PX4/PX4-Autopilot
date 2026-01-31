# TeraRanger 거리계

TeraRanger는 적외선 ToF(Time-of-Flight) 기반의 다양한 경량 거리측정 센서입니다.
They are typically faster and have greater range than sonar, and smaller and lighter than laser-based systems.

PX4는 아래 장치들을 지원합니다.

- [TeraRanger Evo 60m](https://www.terabee.com/shop/lidar-tof-range-finders/teraranger-evo-60m/) (0.5 – 60 m)
- [TeraRanger Evo 600Hz](https://www.terabee.com/shop/lidar-tof-range-finders/teraranger-evo-600hz/) (0.75 - 8 m)

:::info
PX4 also supports _TeraRanger One_ (I2C adapter required).
This has been discontinued.
:::

## 구매처

- TBD

## 핀배열

## 배선

모든 TeraRanger 센서는 I2C 버스로 연결됩니다.

## 소프트웨어 설정

The sensors are enabled using the parameter [SENS_EN_TRANGER](../advanced_config/parameter_reference.md#SENS_EN_TRANGER) (you can set the type of sensor or that PX4 should auto-detect the type).

:::info
If using auto-detect for Evo sensors the minimum and maximum values for the range are set to the lowest and highest possible readings across the Evo family (currently 0.5 - 60 m).
올바른 최대/최소 값을 사용하려면 Evo 센서의 적절한 모델을 매개변수에 설정합니다 (자동 감지를 사용하는 대신).
:::

:::tip
The driver for this rangefinder is usually present in firmware. If missing, you would also need to add the driver (`distance_sensor/teraranger`) to the board configuration.
:::

## 추가 정보

- [Modules Reference: Distance Sensor (Driver) : teraranger](../modules/modules_driver_distance_sensor.md#teraranger)
