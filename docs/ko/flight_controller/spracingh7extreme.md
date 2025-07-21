# SPRacingH7EXTREME (PX4 판)

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://shop.seriouslypro.com) for hardware support or compliance issues.
:::

The [SPRacingH7EXTREME](https://shop.seriouslypro.com/sp-racing-h7-extreme) is a feature packed FC/PDB with DUAL ICM20602 gyros, H7 400/480Mhz(+) CPU, high-precision BMP388 barometer, SD Card socket, current sensor, 8 easily accessible motor outputs, OSD, Microphone, Audio output, and more.

소형에서 대형 쿼드, 평면, 옥토 콥터 및 고급 프레임에 쉽게 사용할 수 있습니다.
내장형 배전판(PDB)이 특징이므로 별도의 ESC와 함께 사용하는 것이 가장 좋습니다.
4in1 ESC 배선도 쉽습니다.

4 개의 추가 모터 출력, SPI 및 UART 연결을 제공하는 12핀 스택 커넥터도 있습니다.

![SPRacingH7EXTREME PCB Top](../../assets/flight_controller/spracingh7extreme/spracingh7extreme-top.jpg)

![SPRacingH7EXTREME PCB Bottom](../../assets/flight_controller/spracingh7extreme/spracingh7extreme-bottom.jpg)

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## 주요 특징

- Main System-on-Chip: [STM32H750VBT6 rev.y/v](https://www.st.com/en/microcontrollers-microprocessors/stm32h750vb.html)
  - CPU: 400/480Mhz (+) ARM Cortex M7 (단정밀도 FPU 포함). (Rev V CPU 사용시 + 480Mhz)
  - RAM: 1MB
  - 16MB External Flash 4-bit QuadSPI in Memory Mapped mode for code _and_ config.
- 내장 센서 :
  - 듀얼 자이로(각각 1xSPI, 별도의 인터럽트 신호 포함, 32khz 가능, fsync 가능)
  - 고정밀 BMP388 기압계(I2C + 인터럽트)
  - 110A 전류 센서
- 외부 8핀 IO 포트를 통한 GPS.
- 시청각
  - 온스크린 디스플레이 OSD(전용 SPI, 문자 기반, MAX7456)
  - 마이크 센서
  - CPU DAC에서 오디오 출력.
  - 마이크/DAC 출력용 오디오 믹서.
- 인터페이스
  - SD 카드(1 비트 SPI가 아닌 4 비트 SDIO)
  - IR 트랜스폰더(iLAP 호환)
  - 부저 회로
  - RSSI (아날로그/PWM)
  - 12개의 모터 출력(모터 패드에 의해 4개, 중간에 4개, 스태킹 커넥터에 4개).
  - 스태킹 커넥터에 대한 1x SPI 브레이크아웃
  - 6 Serial Ports (5x TX & RX, 1x TX-only bi-directional for telemetry)
  - 부팅 버튼 (측면 누름)
  - 바인딩/사용자 버튼 (측면 누름)
  - 수신기 포트 (모든 일반적인 프로토콜, 인버터 필요 없음)
  - CAM 소켓의 CAM OSD 제어 및 비디오 입력.
  - SWD 디버깅 포트.
- VTX 소켓의 비디오 출력 + 오디오 출력.
- OTG 기능이 있는 USB (CPU에 연결된 ID 및 VBUS)
- 전원 시스템
  - 통합 PDB.
  - 2-6S BEC
  - TVS 보호 다이오드
  - 자이로 노이즈 필터 커패시터가 있는 자이로 전용 500ma VREG.
  - CPU, Baro, 마이크 등을 위한 두 번째 500ma VREG
- 기타 기능
  - 상태 LED
  - LED 스트립 지지대 (잘 배치된 연결 패드 포함).
  - SD 카드나 외장 플래시에서 부팅 가능.
  - SD 카드에서 플래시 가능.
  - 상단 납땜 설계.
  - 배터리 와이어용 PCB 컷아웃.
  - 나침반 없음, GPS IO 포트에 연결된 자력계/나침반 센서가있는 외부 GPS를 사용하십시오.
  - Betaflight 4.x+, Cleanflight 4.x+도 실행합니다.
  - Cleanflight를 만든 사람인 Dominic Clifton이 디자인했습니다.
- 크기
  - 36x36mm with 30.5\*30.5 mouting pattern, M4 holes.
  - 소프트 마운트 M4 ~ M3 그로밋이 제공됩니다.

## 구매처

The SPRacingH7EXTREME is available from the [Seriously Pro shop](https://shop.seriouslypro.com/sp-racing-h7-extreme).

:::info
Select the PX4 edition when purchasing!
:::

## 매뉴얼, 핀배열 및 연결 다이어그램

The manual with pinouts can be downloaded from [here](http://seriouslypro.com/files/SPRacingH7EXTREME-Manual-latest.pdf).
See the [SPRacingH7EXTREME website](http://seriouslypro.com/spracingh7extreme) for other diagrams.

## Credits

This design was created by [Dominic Clifton](https://github.com/hydra)
Initial PX4 support by [Igor-Misic](https://github.com/Igor-Misic)
