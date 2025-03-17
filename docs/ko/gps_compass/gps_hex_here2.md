# HEX/ProfiCNC Here2 GPS (Discontinued)

:::warning
This has been superseded by the [Cube Here 3](https://www.cubepilot.com/#/here/here3)
:::

The [Here2 GPS receiver](http://www.proficnc.com/all-products/152-gps-module.html) is an update to the Here GPS module from HEX.

주요 특징은 다음과 같습니다.

- Concurrent reception of up to 3 GNSS (GPS, Galileo, GLOSNASS, BeiDou)
- 업계 최고의 - 167dBm 탐색 감도
- 보안 및 무결성 보호
- 모든 위성 증강 시스템 지원
- 고급 재밍 및 스푸핑 감지

<img src="../../assets/hardware/gps/here2_gps_module.jpg" />

## 구매처

- [ProfiCNC](http://www.proficnc.com/all-products/152-gps-module.html) (Australia)
- [Other resellers](http://www.proficnc.com/stores)

## 설정

PX4의 설정과 사용법은 대부분 플러그앤플레이입니다.

::: info

- If the GPS is _not detected_ then [update the Here2 firmware](https://docs.cubepilot.org/user-guides/here-2/updating-here-2-firmware).
- If the GPS is detected but does not work, attempt the process outlined in [allocating node uavcan ID](https://docs.cubepilot.org/user-guides/here-2/here-2-can-mode-instruction).

:::

## 배선

The Here2 GPS comes with an 8 pin connector that can be inserted directly into the [Pixhawk 2](http://www.hex.aero/wp-content/uploads/2016/07/DRS_Pixhawk-2-17th-march-2016.pdf) GPS UART port.

Pixhawk 3 Pro와 Pixracer에는 6 핀 GPS 포트 커넥터가 있습니다.
이러한 컨트롤러에서는 GPS 케이블 (아래 그림 참조)을 수정하여 핀 6과 7을 제거할 수 있습니다.

<img src="../../assets/hardware/gps/rtk_here_plug_gps_to_6pin_connector.jpg" width="500px" />

핀 6과 7은 안전 버튼용이며 필요한 경우 부착 가능합니다.

### 핀배열

Here2 GPS 핀배열은 아래에서 제공합니다. 이것은 다른 자동조종보드용 커넥터를 수정할 수 있습니다.

| 핀 | Here2 GPS                       | 핀 | Pixhawk 3 Pro GPS           |
| - | ------------------------------- | - | --------------------------- |
| 1 | VCC_5V     | 1 | VCC                         |
| 2 | GPS_RX     | 2 | GPS_TX |
| 3 | GPS_TX     | 3 | GPS_RX |
| 4 | SCL                             | 4 | SCL                         |
| 5 | SDA                             | 5 | SDA                         |
| 6 | BUTTON                          | - | -                           |
| 7 | BUTTON_LED | - | -                           |
| 8 | GND                             | 6 | GND                         |

## 사양

- **Processor:** STM32F302
- **Sensor**
  - **Compass, Gyro, Accelerometer:** ICM20948
  - **Barometer:** MS5611
- **Receiver Type:** 72-channel u-blox M8N engine, GPS/QZSS L2 C/A, GLONASS L10F, BeiDou B11, Galileo E1B/C, SBAS L1 C/A: WAAS, EGNOS, MSAS, GAGAN
- **Navigation Update Rate:** Max: 10 Hz
- **Positionaing Accuracy:** 3D Fix
- **Time to first fix:**
  - **Cold start:** 26s
  - **Aided start:** 2s
  - **Reacquisition:** 1s
- **Sensitivity:**
  - **Tracking & Navigation:** -167 dBm
  - **Hot start:** -148 dBm
  - **Cold start:** -157 dBm
- **Assisted GNSS**
  - AssistNow GNSS 온라인
  - AssistNow GNSS 오프라인 (최대 35 일)
  - AssistNow Autonomous (최대 6 일)
  - OMA SUPL& 3GPP compliant
- **Oscillator:** TCXO (NEO-8MN/Q)
- **RTC crystal:** Build in
- **ROM:** Flash (NEO-8MN)
- **Available Antennas:** Active Antenna & Passive Antenna
- **Signal Integrity:** Signature feature with SHA 256
- **Protocols & Interfaces:**
  - **UART/I2C/CAN:** JST_GH Main interface, Switch internally.
  - **STM32 Main Programming Interface:** JST_SUR
