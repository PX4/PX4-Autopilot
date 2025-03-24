# ModalAI VOXL Flight

<Badge type="tip" text="PX4 v1.11" />

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://forum.modalai.com/) for hardware support or compliance issues.
:::

The ModalAI [VOXL Flight](https://modalai.com/voxl-flight) ([Datasheet](https://docs.modalai.com/voxl-flight-datasheet)) is one of the first computing platforms to combine the power and sophistication of Snapdragon with the flexibility and ease of use of PX4 on an STM32F7.
Made in the USA, VOXL Flight supports obstacle avoidance and GPS-denied (indoor) navigation fused with a PX4 flight controller on a single PCB.

![VOXL-Flight](../../assets/flight_controller/modalai/voxl_flight/voxl-flight-dk.jpg)

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## 사양

### 시스템

| 기능 | 세부 정보 |
| :- | :---- |
| 중량 | 26 g  |

### 보조 컴퓨터

| 기능                | 세부 정보                                                                                                                                                                                                                                                              |
| :---------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 기본 운영 시스템         | 3.18 커널을 사용하는 Linux Yocto Jethro. Additional Linux Operating Systems can be used by running Docker on VOXL, details [here](https://docs.modalai.com/docker-on-voxl/)                                                               |
| 계산                | Qualcomm Snapdragon 821 w/ 4GB LPDDR4 1866MHz, Snapdragon 821 [Datasheet](https://developer.qualcomm.com/download/sd820e/qualcomm-snapdragon-820e-processor-apq8096sge-device-specification.pdf), [Docs](https://developer.qualcomm.com/hardware/apq-8096sg/tools) |
| CPU               | 최대 2.15GHz의 쿼드 코어 CPU                                                                                                                                                                                                                              |
| GPU               | 624MHz의 Adreno 530 GPU                                                                                                                                                                                                                                             |
| DSP 컴퓨팅           | Hexagon compute DSP (cDSP) 825MHz                                                                                                                                                                                                               |
| 센서 DSP            | Hexagon 센서 DSP (sDSP) 700MHz                                                                                                                                                                                                                    |
| 비디오               | 4k30 비디오 캡처 h.264/5 w/ 720p FPV                                                                                                                                                                                                                    |
| Camera Interfaces | MIPI-CSI2, USB UVC, HDMI 지원                                                                                                                                                                                                                                        |
| Wi-Fi             | Pre-certified Wi-Fi module [QCNFA324 FCC ID:PPD-QCNFA324](https://fccid.io/PPD-QCNFA324), QCA6174A modem, 802.11ac 2x2 Dual-band, Bluetooth 4.2 (dual-mode)                                     |
| 4G LTE            | [Optional add-on module](https://www.modalai.com/collections/voxl-add-ons/products/voxl-lte)                                                                                                                                                                       |
| Microhard pDDL    | [Optional add-on module](https://www.modalai.com/collections/voxl-add-ons/products/voxl-microhard-modem-usb-hub)                                                                                                                                                   |
| GNSS              | WGR7640 10Hz                                                                                                                                                                                                                                                       |
| I/O               | 1x USB3.0 OTG (ADB 포트), 1x USB2.0 (확장 포트), 2x UART, 3x I2C, 추가 GPIO 및 SPI 설정 가능                                                                                                              |
| 저장 장치             | 32GB (UFS 2.0), Micro SD 카드                                                                                                                                                                                                     |
| 소프트웨어             | Docker, OpenCV 2.4.11, 3.4.6, 4.2, ROS Indigo, Qualcomm Machine Vision SDK, see [GitLab](https://gitlab.com/voxl-public) for lots of open source examples!                         |
| 관성계               | ICM-42688 (SPI10), ICM-20948 (SPI1)                                                                                                                                                                                          |
| 기압계               | BMP280                                                                                                                                                                                                                                                             |

### 비행 콘트롤러

| 기능         | 세부 정보                                                            |
| :--------- | :--------------------------------------------------------------- |
| MCU        | 216MHz, 32-bit ARM M7 [STM32F765II][stm32f765ii]                 |
| 메모리        | 256Kb FRAM                                                       |
|            | 2Mbit Flash                                                      |
|            | 512Kbit SRAM                                                     |
| 펌웨어        | [PX4][px4]                                                       |
| 관성계        | [ICM-20602][icm-20602] (SPI1)                                    |
|            | ICM-42688 (SPI2)                              |
|            | [BMI088][bmi088] (SPI6)                                          |
| 기압계        | [BMP388][bmp388] (I2C4)                                          |
| 보안 요소      | [A71CH][a71ch] (I2C4)                                            |
| microSD 카드 | [Information on supported cards](../dev_log/logging.md#sd-cards) |
| 입력         | GPS/자력계                                                          |
|            | Spektrum                                                         |
|            | 텔레메트리                                                            |
|            | CAN 버스                                                           |
|            | PPM                                                              |
| 출력         | LED 6 개 (2xRGB)                               |
|            | PWM 채널 8개                                                        |
| 추가 인터페이스   | 시리얼포트 3개                                                         |
|            | I2C                                                              |
|            | GPIO                                                             |

<!-- reference links for above table (improve layout) -->

[stm32f765ii]: https://www.st.com/en/microcontrollers-microprocessors/stm32f765ii.html
[px4]: https://github.com/PX4/PX4-Autopilot/tree/main/boards/modalai/fc-v1
[icm-20602]: https://www.invensense.com/products/motion-tracking/6-axis/icm-20602/
[bmi088]: https://www.bosch-sensortec.com/bst/products/all_products/bmi088_1
[bmp388]: https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp388/
[a71ch]: https://www.nxp.com/products/security-and-authentication/authentication/plug-and-trust-the-fast-easy-way-to-deploy-secure-iot-connections:A71CH

:::info
More detailed hardware documentation can be found [here](https://docs.modalai.com/voxl-flight-datasheet/).
:::

## 크기

![FlightCoreV1Dimensions](../../assets/flight_controller/modalai/voxl_flight/voxl-flight-dimensions.jpg)

[3D STEP File](https://storage.googleapis.com/modalai_public/modal_drawings/M0019_VOXL-Flight.zip)

## PX4 Firmware Compatibility

_VOXL Flight_ is fully compatible with the official PX4 Firmware from PX4 v1.11.

ModalAI maintains a [branched PX4 version](https://github.com/modalai/px4-firmware/tree/modalai-1.11) for PX4 v1.11.
This includes UART ESC support and improvements in VIO and VOA that are planned to be upstreamed.

More information about the firmware can be found [here](https://docs.modalai.com/flight-core-firmware/).

## QGroundControl 지원

아래 다이어그램은 PX4 v1.11(및 <a href="https://github.com/modalai/px4-firmware/tree/modalai-1.10">ModalAI가 유지하는 PX4 v1.10 브랜치</a>)부터 <code>ROTATION_NONE</code> 권장 방향을 나타냅니다.

## 구매처

- [VOXL Flight Complete Kit](https://modalai.com/voxl-flight)
- [VOXL Flight Board](https://www.modalai.com/products/voxl-flight?variant=31707275362355) (only)
- [VOXL Flight integrated with Obstacle Avoidance Cameras (VOXL Flight Deck)](https://modalai.com/flight-deck) ([Datasheet](https://docs.modalai.com/voxl-flight-deck-platform-datasheet/))
- [VOXL Flight in a ready to fly VOXL m500 Development Drone](https://www.modalai.com/collections/development-drones/products/voxl-m500) ([Datasheet](https://docs.modalai.com/voxl-m500-reference-drone-datasheet/))

## 빠른 시작

A quickstart from the vendor is located [here](https://docs.modalai.com/voxl-flight-quickstart/).

### voxl-vision-px4

The VOXL Flight runs [voxl-vision-px4](https://gitlab.com/voxl-public/modal-pipe-architecture/voxl-vision-px4) on the companion computer portion of the hardware serving as a sort of MAVLink proxy.
For details, the source code is available [here](https://gitlab.com/voxl-public/modal-pipe-architecture/voxl-vision-px4)

### 커넥터

Detailed information about the pinouts can be found [here](https://docs.modalai.com/voxl-flight-datasheet-connectors/).

#### 상단

![VOXLFlightTop](../../assets/flight_controller/modalai/voxl_flight/voxl-flight-top.jpg)

_Note: 1000 Series connectors accessible from the STM32/PX4_

| 커넥터   | 요약                                                                  | 사용처                                                  |
| ----- | ------------------------------------------------------------------- | ---------------------------------------------------- |
| J2    | 4k 이미지 센서 (CSI0) 고용                              | Snapdragon - 리눅스                                     |
| J3    | 스테레오 이미지 센서 (CSI1)                               | Snapdragon - 리눅스                                     |
| J6    | 냉각 팬 커넥터                                                            | Snapdragon - 리눅스                                     |
| J7    | BLSP6 (GPIO) and BLSP9 (UART) | Snapdragon - 리눅스                                     |
| J13   | 확장 B2B                                                              | Snapdragon - 리눅스                                     |
| J14   | 통합 GNSS 안테나 연결                                                      | Snapdragon - 리눅스                                     |
| J1001 | 프로그래밍 및 디버그/UART3                                                   | STM32 - PX4                                          |
| J1002 | UART ESC, UART2/TELEM3                                              | STM32 - PX4                                          |
| J1003 | PPM RC 입력                                                           | STM32 - PX4                                          |
| J1004 | RC 입력, Spektrum/SBus/UART6                                          | STM32 - PX4                                          |
| J1006 | USB 2.0 커넥터(PX4/QGroundControl)  | STM32 - PX4                                          |
| J1007 | 8 채널 PWM/DShot 출력                                                   | STM32 - PX4                                          |
| J1008 | CAN 버스                                                              | STM32 - PX4                                          |
| J1009 | I2C3, UART4                                                         | STM32 - PX4                                          |
| J1010 | 텔레메트리 (TELEM1)                                   | STM32 - PX4                                          |
| J1011 | I2C2, 안전 버튼 입력                                                      | STM32 - PX4                                          |
| J1012 | External GPS & Mag, UART1, I2C1                 | STM32 - PX4                                          |
| J1013 | 전원 입력, I2C3                                                         | STM32 - PX4 (powers whole system) |

#### 하단

![VOXLFlightBottom](../../assets/flight_controller/modalai/voxl_flight/voxl-flight-bottom.jpg)

_Note: 1000 Series connectors accessible from the STM32/PX4_

| 커넥터          | 요약                                     | 사용처                         |
| ------------ | -------------------------------------- | --------------------------- |
| J4           | 추적/광류 이미지 센서 (CSI2) | Snapdragon - 리눅스            |
| J8           | USB 3.0 OTG            | Snapdragon - Linux, **adb** |
| J10          | BLSP7 UART 및 I2C 오프보드                  | Snapdragon - 리눅스            |
| J11          | BLSP12 UART 및 I2C 오프보드                 | Snapdragon - 리눅스            |
| VOXL microSD |                                        | Snapdragon - 리눅스            |
| PX4 microSD  | 32Gb Max                               | STM32 - PX4                 |
| Wi-Fi 안테나    | 포함됨.                   | Snapdragon - 리눅스            |

### 사용자 가이드

The full user guide is available [here](https://docs.modalai.com/voxl-flight-quickstart).

### 빌드 방법

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make modalai_fc-v1
```

## 시리얼 포트 매핑

_Note: mappings shown are for the PX4 controlled interfaces only_

| UART   | 장치         | 포트                                |
| ------ | ---------- | --------------------------------- |
| USART1 | /dev/ttyS0 | GPS1 (J1012)   |
| USART2 | /dev/ttyS1 | TELEM3 (J1002) |
| USART3 | /dev/ttyS2 | 디버그 콘솔(J1001)  |
| UART4  | /dev/ttyS3 | 확장 UART (J6)   |
| UART5  | /dev/ttyS4 | PX4와 보조 컴퓨터간의 UART                |
| USART6 | /dev/ttyS5 | RC (J1004)     |
| UART7  | /dev/ttyS6 | TELEM1 (J1010) |
| UART8  | /dev/ttyS7 | 해당없음                              |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->

## 지원

Please visit the [ModalAI Forum](https://forum.modalai.com/category/8/voxl-flight) for more information.
