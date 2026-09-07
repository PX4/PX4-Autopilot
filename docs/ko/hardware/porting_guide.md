# 비행 콘트롤러 포팅 가이드

This topic is for developers who want to port PX4 to work with _new_ flight controller hardware.

## PX4 아키텍쳐

PX4 consists of two main layers: The [board support and middleware layer](../middleware/index.md) on top of the host OS (NuttX, Linux or any other POSIX platform like Mac OS), and the applications (Flight Stack in [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules)\).
Please reference the [PX4 Architectural Overview](../concept/architecture.md) for more information.

애플리케이션/플라이트 스택이 모든 보드 대상에서 실행되므로, 이 가이드에서는 호스트 OS와 미들웨어에만 초점을 맞추어 설명합니다.

## 비행 콘트롤러 설정  파일 구조

Board startup and configuration files are located under [/boards](https://github.com/PX4/PX4-Autopilot/tree/main/boards/) in each board's vendor-specific directory (i.e. **boards/_VENDOR_/_MODEL_/**).

예 FMUv5 :

- (All) Board-specific files: [/boards/px4/fmu-v5](https://github.com/PX4/PX4-Autopilot/tree/main/boards/px4/fmu-v5).<!-- NEED px4_version -->
- Build configuration: [/boards/px4/fmu-v5/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/default.px4board).<!-- NEED px4_version -->
- Board-specific initialisation file: [/boards/px4/fmu-v5/init/rc.board_defaults](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/init/rc.board_defaults) <!-- NEED px4_version -->
  - A board-specific initialisation file is automatically included in startup scripts if found under the boards directory at **init/rc.board**.
  - 이 파일은 특정 보드에만 존재하는 센서(및 기타 항목)를 시작하는 데 사용됩니다.
    또한 보드의 기본 매개변수, UART 매핑 및 기타 특수한 경우를 설정하는 데 사용할 수 있습니다.
  - FMUv5은 시작되는 모든 Pixhawk 4 센서를 볼 수 있으며, 더 큰 LOGGER_BUF도 설정합니다.

## 호스트 운영 체제 설정

이 섹션에서는 지원되는 호스트 운영 체제에서 새 비행 콘트롤러 이식에 필요한 설정 파일의 목적과 위치에 대하여 설명합니다.

### NuttX

See [NuttX Board Porting Guide](porting_guide_nuttx.md).

### Linux

Linux 보드에는 OS와 커널 설정이 포함되어 있지 않습니다.
Linux에는 이미 보드에 사용할 수 있는 Linux 이미지에 의해 제공됩니다(즉시 관성 센서를 지원해야 함).

- [boards/px4/raspberrypi/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/raspberrypi/default.px4board) - RPi cross-compilation. <!-- NEED px4_version -->

## 미들웨어 구성 요소 및 설정

이 섹션에서는 다양한 미들웨어 구성 요소와 이를 새로운 비행 콘트롤러로 이식에 필요한 설정 파일 업데이트에 대하여 설명합니다.

### QuRT / Hexagon

- The start script is located in [posix-configs/](https://github.com/PX4/PX4-Autopilot/tree/main/posix-configs). <!-- NEED px4_version -->
- OS 구성은 기본 Linux 이미지의 일부입니다(TODO: LINUX IMAGE 및 플래시 지침의 위치 제공).
- The PX4 middleware configuration is located in [src/boards](https://github.com/PX4/PX4-Autopilot/tree/main/boards). <!-- NEED px4_version --> TODO: ADD BUS CONFIG

## RC UART 배선 권장 사항

일반적으로 별도의 RX 및 TX 핀을 통해 RC를 마이크로 콘트롤러에 연결하는 것이 좋습니다.
그러나 RX와 TX가 함께 연결된 경우에는 UART의 경합 방지를 위하여, 단일 와이어 모드로 전환되어야 합니다.
이것은 보드 설정과 매니페스트 파일을 통하여 수행됩니다.
One example is [px4fmu-v5](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/src/manifest.c). <!-- NEED px4_version -->

## Getting Your Board Supported

This page covers the _technical_ work of porting PX4 to new hardware.
The _process_ for getting that port reviewed, merged, and listed on the PX4 website, including board IDs, USB VID/PID, flight-test evidence, and maintenance expectations, is documented separately:

- [Manufacturer's PX4 Board Support Guide](../hardware/board_support_guide.md)

In short: build your own firmware target based on PX4, demonstrate stable flight on the current release, and open a pull request with your board support code, documentation, and flight logs.
The board support guide explains each step.

The PX4 project supports and maintains the [FMU standard reference hardware](../hardware/reference_design.md) and any boards compatible with the standard, including the [Pixhawk series](../flight_controller/pixhawk_series.md) (see the [full list of supported hardware](../flight_controller/index.md)). Boards merged into PX4 benefit from a port in the repository, firmware builds accessible from _QGroundControl_, compatibility with the rest of the ecosystem, and automated CI checks.

:::tip
The cost of maintaining a port is proportional to how far it diverges from the standard.
Consider that cost before deviating: staying close to the reference design lets you benefit from day-to-day PX4 development with minimal maintenance burden.
:::

Manufacturers are responsible for keeping their port up to date and working across PX4 releases.
The PX4 project reserves the right to refuse or remove ports that do not meet the project's requirements, and all contributors are expected to follow the [Code of Conduct](https://github.com/PX4/PX4-Autopilot/blob/main/CODE_OF_CONDUCT.md).

## 관련 정보

- [Device Drivers](../middleware/drivers.md) - How to support new peripheral hardware (device drivers)
- [Building the Code](../dev_setup/building_px4.md) - How to build source and upload firmware
- 지원 비행 콘트롤러:
  - [Autopilot Hardware](../flight_controller/index.md)
  - [Supported boards list](https://github.com/PX4/PX4-Autopilot/#supported-hardware) (Github) - Boards for which PX4-Autopilot has specific code
- [Supported Peripherals](../peripherals/index.md)
