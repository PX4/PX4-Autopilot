# CUAV V5+ 배선 개요

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://store.cuav.net/) for hardware support or compliance issues.
:::

This quick start guide shows how to power the [CUAV V5+](../flight_controller/cuav_v5_plus.md) flight controller and connect its most important peripherals.

![V5+ AutoPilot - Hero Image](../../assets/flight_controller/cuav_v5_plus/v5+_01.png)

## 배선 개요

아래의 이미지는 주요 센서와 주변 장치(모터 및 서보 출력 제외)들의 연결 방법을 설명합니다.
다음 섹션에서 각 장치에 대하여 자세히 설명합니다.

![V5+ AutoPilot](../../assets/flight_controller/cuav_v5_plus/connection/v5+_quickstart_01.png)

| 인터페이스                              | 기능                                                                                                                                  |
| :--------------------------------- | :---------------------------------------------------------------------------------------------------------------------------------- |
| Power1                             | 전원 연결. Power input with _analog_ voltage and current detection. 이 커넥터에 Digital 전원모듈을 사용하지 마십시오!     |
| Power2                             | i2c 스마트 배터리를 연결합니다.                                                                                                 |
| TF CARD                            | 로그 저장용 SD 카드 (카드는 공장에서 미리 삽입됨).                                                                  |
| M1~M8              | PWM 출력 모터와 서보를 콘트롤 합니다.                                                                                             |
| A1~A6              | PWM 출력 모터와 서보를 콘트롤 합니다.                                                                                             |
| DSU7                               | FMU 디버그에 사용되며 디버그 정보를 획득합니다.                                                                                        |
| I2C1/I2C2                          | 외부 나침반과 같은 I2C 장치를 연결합니다.                                                                                           |
| CAN1/CAN2                          | CAN GPS와 같은 UAVCAN 장치를 연결합니다.                                                                                       |
| TYPE-C\(USB\) | 펌웨어로드 등의 작업을 위하여 비행 콘트롤러와 컴퓨터간의 통신을 위하여 컴퓨터에 연결합니다.                                                                 |
| SBUS OUT                           | SBUS 장치(예 : 카메라 짐벌)를 연결합니다.                                                      |
| GPS&SAFETY     | GPS, 안전 스위치 및 부저가 포함된 Neo GPS에 연결합니다.                                                                               |
| TELEM1/TELEM2                      | 텔레메트리에 연결합니다.                                                                                                       |
| DSM/SBUS/RSSI                      | DSM, SBUS, RSSI 신호 입력 인터페이스, DSM 인터페이스는 DSM 위성 수신기에 연결 가능, SBUS 인터페이스는 SBUS 원격 제어 수신기에 연결 가능, 신호 강도 반환 모듈용 RSSI 포함. |

:::info
For more interface information, please read [V5+ Manual](http://manual.cuav.net/V5-Plus.pdf).
:::

![V5+ AutoPilot](../../assets/flight_controller/cuav_v5_plus/connection/v5+_quickstart_02.png)

:::info
If the controller cannot be mounted in the recommended/default orientation (e.g. due to space constraints) you will need to configure the autopilot software with the orientation that you actually used: [Flight Controller Orientation](../gps_compass/rtk_gps.md).
:::

## GPS + 나침반 + 안전 스위치 + LED

The recommended GPS module is the _Neo v2 GPS_, which contains GPS, compass, safety switch, buzzer, LED status light.

:::info
Other GPS modules may not work (see [this compatibility issue](../flight_controller/cuav_v5_nano.md#compatibility_gps)\)).
:::

The GPS/Compass module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker towards the front of the vehicle (_Neo v2 GPS_ arrow is in the same direction as the flight control arrow).
케이블을 사용하여 비행 제어 GPS에 연결합니다.

:::info
If you use the [NEO V2 PRO GNSS (CAN GPS)](http://doc.cuav.net/gps/neo-series-gnss/en/neo-v2-pro.html), please use the cable to connect to the flight control CAN interface.
:::

![V5+ AutoPilot](../../assets/flight_controller/cuav_v5_plus/connection/v5+_quickstart_03.png)

## 안전 스위치

The dedicated safety switch that comes with the V5+ is only required if you are not using the recommended _Neo V2 GPS_ (which has an inbuilt safety switch).

If you are flying without the GPS you must attach the switch directly to the `GPS1` port in order to be able to arm the vehicle and fly (if you use the old 6-pin GPS, please read the definition of the bottom interface to change the line).

## 부저

권장 GPS를 사용하지 않는 경우에는 부저가 작동하지 않을 수 있습니다.

## 무선 조종

차량을 수동 제어에는 무선 조종기가 필요합니다.
PX4의 자율 비행은 무선조종기가 필수 사항은 아닙니다.
기체와 조종자가 통신하기 위하여 송신기와 수신기를 바인딩하여야 합니다.
송신기와 수신기의 매뉴얼을 참고하십시오.

아래 그림은 원격 수신기에 액세스하는 방법을 보여줍니다.
키트에서 SBUS 케이블을 찾으십시오.

![V5+ AutoPilot](../../assets/flight_controller/cuav_v5_plus/connection/v5+_quickstart_04.png)

## Spektrum Satellite 수신기

V5+에는 DSM 전용 케이블이 포함되어 있습니다.
Spektrum 위성 수신기는 비행 콘트롤러 DSM/SBUS/RSSI 인터페이스에 연결하여야합니다.

## 전원

The V5+ kit includes the _HV_PM_ module, which supports 2~14S LiPo batteries.
Connect the 6pin connector of the _HW_PM_ module to the flight control `Power1` interface.

:::warning
The supplied power module is unfused.
Power **must** be turned off while connecting peripherals.
:::

![V5+ AutoPilot](../../assets/flight_controller/cuav_v5_plus/connection/v5+_quickstart_01.png)

:::info
The power module is not a power source for peripherals connected to the PWM outputs.
서보/액추에이터를 연결하는 경우에는 BEC를 사용하여 별도로 전원을 공급하여야 합니다.
:::

## 텔레메트리(선택 사항)

지상국에서는 텔레메트리를 사용하여 기체를 통신, 모니터링, 제어 합니다.
기체를 특정 위치로 움직이도록 지시하거나, 새로운 임무를 업로드할 수 있습니다.

통신 채널은 텔레메트리를 통하여 이루어집니다.
The vehicle-based radio should be connected to either the `TELEM1` or `TELEM2` port (if connected to these ports, no further configuration is required).
다른 라디오는 지상국 컴퓨터 또는 모바일 장치에 USB를 통하여 연결합니다.

![V5+ AutoPilot](../../assets/flight_controller/cuav_v5_plus/connection/v5+_quickstart_06.png)

<a id="sd_card"></a>

## SD 카드(선택 사항)

An [SD card](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory) is inserted in the factory (you do not need to do anything).

## 모터

Motors/servos are connected to the MAIN and AUX ports in the order specified for your vehicle in the [Airframes Reference](../airframes/airframe_reference.md).

![V5+ AutoPilot](../../assets/flight_controller/cuav_v5_plus/connection/v5+_quickstart_07.png)

## 핀배열

Download **V5+** pinouts from [here](http://manual.cuav.net/V5-Plus.pdf).

## 추가 정보

- [Airframe build-log using CUAV v5+ on a DJI FlameWheel450](../frames_multicopter/dji_f450_cuav_5plus.md)
- [CUAV V5+ Manual](http://manual.cuav.net/V5-Plus.pdf) (CUAV)
- [CUAV V5+ docs](http://doc.cuav.net/flight-controller/v5-autopilot/en/v5+.html) (CUAV)
- [FMUv5 reference design pinout](https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY/edit#gid=912976165) (CUAV)
- [CUAV Github](https://github.com/cuav) (CUAV)
- [Base board design reference](https://github.com/cuav/hardware/tree/master/V5_Autopilot/V5%2B/V5%2BBASE) (CUAV)
