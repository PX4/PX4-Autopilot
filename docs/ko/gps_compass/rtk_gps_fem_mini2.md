# Femtones MINI2 Receiver

[MINI2 Receiver](http://www.femtomes.com/#/MiniII?type=0) is an RTK GPS receiver that delivers high-rate and reliable RTK initialization for centimeter level positioning.
It is intended for applications that require high-precision positioning (e.g. navigation and mapping, etc.).

수신기는 직렬 포트 (UART)를 통하여 PX4에 연결되며, 표준 웹 브라우저를 사용하여 설정할 수 있습니다.

![MINI II Receiver](../../assets/hardware/gps/rtk_fem_miniII_receiver.jpg)

:::info
PX4 drivers for Ethernet, CAN and USB are under development.
:::

## 필수 펌웨어 옵션

장치 구매시 다음 펌웨어 옵션을 선택하여야 합니다.

- 5Hz, 10Hz, 20Hz
- INS
- HEADING
- OBS
- RTK
- BASE

## 구매처

Contact [Femtones](http://www.femtomes.com/) directly for sales quote:

- **Email:** [sales@femtomes.com](mailto:sales@femtomes.com)
- **Telephone:** +86-10-53779838

## 기능성 포트

![MINI II 1](../../assets/hardware/gps/rtk_fem_miniII_1.jpg)

## 배선

The [MINI2 Receiver](http://www.femtomes.com) is connected to a UART on the flight controller (GPS port) for data.
모듈에 전원을 공급하기 위하여, 별도의 12V 전원공급장치가 필요합니다.
12핀 커넥터 핀은 아래와 같이 번호가 지정됩니다.

![MINI_II_2](../../assets/hardware/gps/rtk_fem_miniII_2.jpg)

## 설정

방향 추정을 위해 두 안테나는 같은 높이에 있어야하고, 서로 최소 30cm 떨어져 있어야합니다.
The direction that they are facing does not matter as it can be configured with the [GPS_YAW_OFFSET](../advanced_config/parameter_reference.md#GPS_YAW_OFFSET) parameter.

Configure the serial port on which the [MINI2 Receiver](http://www.femtomes.com/#/MiniII?type=0) will run using [GPS_1_CONFIG](../advanced_config/parameter_reference.md#GPS_1_CONFIG), and set the baud rate to 115200 using [SER_GPS1_BAUD](../advanced_config/parameter_reference.md#SER_GPS1_BAUD).

Once configured the receiver is used in the same way as any other [RTK GPS](../gps_compass/rtk_gps.md) (i.e. with respect to the Survey-in process).

## 추가 정보

MINI2는 다음 구성 요소를 통합합니다.

- [FB672](http://www.femtomes.com/#/FB672): Compact, dual antenna, dual frequency GNSS OEM board (delivers centimeter accurate position and precise heading).

  ![FB672](../../assets/hardware/gps/rtk_fem_fb_1.jpg)

- [FB6A0](http://www.femtomes.com/#/FB6A0): Compact, quadruple frequency GNSS OEM board (delivers centimeter-accurate positioning)

  ![FB6A0](../../assets/hardware/gps/rtk_fem_fb_2.jpg)

자세한 제품 매뉴얼은 공식 웹 사이트와 제조사에 문의하십시오.
