# 비행 콘트롤러 포팅 가이드

This topic is for developers who want to port PX4 to work with _new_ flight controller hardware.

## PX4 아키텍쳐

PX4 consists of two main layers: The [board support and middleware layer](../middleware/index.md) on top of the host OS (NuttX, Linux or any other POSIX platform like Mac OS), and the applications (Flight Stack in [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules)\). Please reference the [PX4 Architectural Overview](../concept/architecture.md) for more information.

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

## 공식 지원 하드웨어

The PX4 project supports and maintains the [FMU standard reference hardware](../hardware/reference_design.md) and any boards that are compatible with the standard.
This includes the [Pixhawk-series](../flight_controller/pixhawk_series.md) (see the user guide for a [full list of officially supported hardware](../flight_controller/index.md)).

공식 지원 보드의 이점은 다음과 같습니다.

- PX4 저장소에서 PX4 포트 사용 가능
- Automatic firmware builds that are accessible from _QGroundControl_
- 나머지 생태계와의 호환성
- CI를 통한 자동 검사 - 이 커뮤니티에서 가장 중요한 것은 안전입니다.
- [Flight testing](../test_and_ci/test_flights.md)

We encourage board manufacturers to aim for full compatibility with the [FMU spec](https://pixhawk.org/).
완전한 호환성을 통하여 PX4의 지속적인 일일 개발의 이점을 얻을 수 있지만, 사양에서 벗어난 지원으로 인한 유지 관리 비용은 없습니다.

:::tip
Manufacturers should carefully consider the cost of maintenance before deviating from the specification (the cost to the manufacturer is proportional to the level of divergence).
:::

We welcome any individual or company to submit their port for inclusion in our supported hardware, provided they are willing to follow our [Code of Conduct](https://github.com/PX4/PX4-Autopilot/blob/main/CODE_OF_CONDUCT.md) and work with the Dev Team to provide a safe and fulfilling PX4 experience to their customers.

또한 PX4 개발 팀은 안전한 소프트웨어를 출시할 책임이 있으므로, 모든 보드 제조업체는 포트를 최신 상태 유지에 필요한 리소스를 투입하여야 합니다.

보드를 PX4에서 공식적으로 지원하려면:

- 하드웨어는 시장에서 제한없이 구매할 수 있어야 합니다.
- Hardware must be made available to the PX4 Dev Team so that they can validate the port (contact [lorenz@px4.io](mailto:lorenz@px4.io) for guidance on where to ship hardware for testing).
- The board must pass full [test suite](../test_and_ci/index.md) and [flight testing](../test_and_ci/test_flights.md).

**The PX4 project reserves the right to refuse acceptance of new ports (or remove current ports) for failure to meet the requirements set by the project.**

You can reach out to the core developer team and community on the [official support channels](../contribute/support.md).

## 관련 정보

- [Device Drivers](../middleware/drivers.md) - How to support new peripheral hardware (device drivers)
- [Building the Code](../dev_setup/building_px4.md) - How to build source and upload firmware
- 지원 비행 콘트롤러:
  - [Autopilot Hardware](../flight_controller/index.md)
  - [Supported boards list](https://github.com/PX4/PX4-Autopilot/#supported-hardware) (Github) - Boards for which PX4-Autopilot has specific code
- [Supported Peripherals](../peripherals/index.md)
