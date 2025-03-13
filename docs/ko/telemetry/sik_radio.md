# SiK 라디오

[SiK radio](https://github.com/LorenzMeier/SiK) is a collection of firmware and tools for telemetry radios.

PX4 is protocol-compatible with radios that use _SiK_.
SiK Radios often come with appropriate connectors/cables allowing them to be directly connected to [Pixhawk Series](../flight_controller/pixhawk_series.md) controllers
(in some cases you may need to obtain an appropriate cable/connector).
보통 기체와 지상통제장치 간에 한 쌍의 장비가 필요합니다.

SiK 라디오는 다양한 범위와 폼 팩터를 지원하는 다양한 제조업체/가게에서 구매가능 합니다.

![SiK Radio](../../assets/hardware/telemetry/holybro_sik_radio.jpg)

## 공급 업체

- [RFD900 Telemetry Radio](../telemetry/rfd900_telemetry.md)
- [Holybro Telemetry Radio](../telemetry/holybro_sik_radio.md)
- <del>_HKPilot Telemetry Radio_</del> (Discontinued)
- <del>_3DR Telemetry Radio_</del> (Discontinued)

## Setup/Configuration

지상통제장치에서는 무선통신장치는 플러그앤플레이 방식의 USB로 연결합니다.

The vehicle-based radio is connected to the flight-controller's `TELEM1` port, and typically requires no further configuration.

## 펌웨어 업데이트

Hardware sourced from most [vendors](#vendors) should come pre-configured with the latest firmware.
구형 하드웨어에서 MAVLink 2 추가하기 위하여, 새 펌웨어로 업데이트하여야 합니다.

You can update the radio firmware using _QGroundControl_: [QGroundControl User Guide > Loading Firmware](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/firmware.html).

## 고급 설정

The Development section has [additional information](../data_links/sik_radio.md) about building firmware and AT-command based configuration.
개발자가 아닌 경우에는 이 작업을 수행하지 않아도 됩니다.
