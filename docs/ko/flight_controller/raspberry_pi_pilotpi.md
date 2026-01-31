# 라즈베리파이 PilotPi 실드

<LinkedBadge type="warning" text="Experimental" url="../flight_controller/autopilot_experimental.html"/>

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](mailto:lhf2613@gmail.com) for hardware support or compliance issues.
:::

The _PilotPi_ shield is a fully functional solution to run PX4 autopilot directly on Raspberry Pi.
리눅스와 PX4 측면에서 지속적으로 업데이트되고 있는, 저비용 고확장성의 플랫폼입니다.
모든 구성 요소가 라즈베리파이와 PX4 커뮤니티의 업스트림 지원을 제공하므로, 독점 드라이버가 필요하지 않습니다.
PCB와 회로도도 오픈 소스입니다.

![PilotPi with RPi 4B](../../assets/flight_controller/pilotpi/hardware-pilotpi4b.png)

## 요약

- 지원 라즈베리파이 보드:
  - 라즈베리파이 2B/3B/3B+/4B
- 지원 운영체제:
  - 라즈베리파이 OS
  - 우분투 서버 (armhf/arm64)
- 가속도계/자이로
  - ICM42688P
- 자력계:
  - IST8310
- 기압계:
  - MS5611
- PWM:
  - PCA9685
- ADC:
  - ADS1115
- 전원:
  - 전압감지기능이 내장 3~6S 배터리.
  - USB 케이블 전원 공급
- Availability: _preparing for shipping_

## 연결성

Shield는 다음을 제공합니다.

- PWM 출력 채널 16개
- GPS 커넥터
- 텔레메트리 커넥터
- External I2C bus connector (**Note:** conflicts with CSI camera)
- RC 입력 포트(SBUS)
- ADC 채널 범위 0 ~ 5V 3개
- 2\*8 2.54mm 미사용 GPIO 커넥터

라즈베리파이 직접 액세스

- USB 커넥터 4개
- CSI connector(**Note:** conflict with external I2C bus)
- 기타

## 권장 배선

![PilotPi PowerPart wiring](../../assets/flight_controller/pilotpi/pilotpi_pwr_wiring.png)

![PilotPi SensorPart wiring](../../assets/flight_controller/pilotpi/pilotpi_sens_wiring.png)

## 핀배열

:::warning
It still uses old GH1.25 connectors.
Wiring is compatible with Pixhawk 2.4.8
:::

### 커넥터

#### GPS 커넥터

Mapped to `/dev/ttySC0`

| 핀 | 신호  | 전압   |
| - | --- | ---- |
| 1 | VCC | +5V  |
| 2 | TX  | +3V3 |
| 3 | RX  | +3V3 |
| 4 | NC  | +3V3 |
| 5 | NC  | +3V3 |
| 6 | GND | GND  |

#### 텔레메트리 커넥터

Mapped to `/dev/ttySC1`

| 핀 | 신호  | 전압   |
| - | --- | ---- |
| 1 | VCC | +5V  |
| 2 | TX  | +3V3 |
| 3 | RX  | +3V3 |
| 4 | CTS | +3V3 |
| 5 | RTS | +3V3 |
| 6 | GND | GND  |

#### 외부 I2C 커넥터

Mapped to `/dev/i2c-0`

| 핀 | 신호  | 전압                          |
| - | --- | --------------------------- |
| 1 | VCC | +5V                         |
| 2 | SCL | +3v3(풀업) |
| 3 | SDA | +3v3(풀업) |
| 4 | GND | GND                         |

#### RC & ADC2/3/4

RC is mapped to `/dev/ttyAMA0` with signal inverter switch on RX line.

| 핀 | 신호  | 전압                         |
| - | --- | -------------------------- |
| 1 | RC  | +3V3 ~ +5V |
| 2 | VCC | +5V                        |
| 3 | GND | GND                        |

- ADC1은 배터리 전압 모니터링을 위해 전압 분배기에 내부적으로 연결됩니다.
- ADC2는 사용되지 않습니다.
- ADC3는 아날로그 속도 센서에 연결할 수 있습니다.
- ADC4에는 시스템 전압 레벨을 모니터링하기 위하여 ADC와 VCC 사이에 점퍼 캡이 있습니다.

| 핀 | 신호   | 전압                       |
| - | ---- | ------------------------ |
| 1 | ADCx | 0V ~ +5V |
| 2 | VCC  | +5V                      |
| 3 | GND  | GND                      |

:::info
ADC3 & 4 have an alternative VCC source
When 'Vref' switch is on, 'VCC' pin is driven by REF5050.
:::

#### 보드 상단의 미사용 GPIO

| 실드 핀 | BCM | WiringPi | RPi 핀 |
| ---- | --- | -------- | ----- |
| 1    | 3V3 | 3V3      | 3V3   |
| 2    | 5V  | 5V       | 5V    |
| 3    | 4   | 7        | 7     |
| 4    | 14  | 15       | 8     |
| 5    | 17  | 0        | 11    |
| 6    | 27  | 2        | 13    |
| 7    | 22  | 3        | 15    |
| 8    | 23  | 4        | 16    |
| 9    | 7   | 11       | 26    |
| 10   | 5   | 21       | 29    |
| 11   | 6   | 22       | 31    |
| 12   | 12  | 26       | 32    |
| 13   | 13  | 23       | 33    |
| 14   | 16  | 27       | 36    |
| 15   | 26  | 25       | 37    |
| 16   | GND | GND      | GND   |

### 스위치

#### RC 인버터

This switch will decide the signal polarity of RX line: `UART_RX = SW xor RC_INPUT`

- 켜짐: SBUS (신호 반전)에 적합
- 꺼짐: 보존됨

#### Vref

ADC 3 & 4 will have VCC driven by:

- 켜진 경우 REF5050에서 Vref 출력
- 꺼져있는 경우 라즈베리파이에서 직접 5V 핀

#### 부팅 모드

이 스위치는 Pin22(BCM25)에 연결됩니다.
시스템 rc 스크립트는 해당 값을 확인하고, PX4가 시스템 부팅시 시작 여부를 결정합니다.

- 켜짐: 자동으로 PX4 시작
- 꺼짐: PX4를 시작하지 않습니다.

## 개발자 가이드

라즈베리파이 OS에 대한 특정 지침을 참고하십시오.

- [Raspberry Pi OS Lite (armhf)](raspberry_pi_pilotpi_rpios.md)
- [Ubuntu Server (arm64 & armhf)](raspberry_pi_pilotpi_ubuntu_server.md)
