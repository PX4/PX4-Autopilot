# 텔레메트리 라디오/모뎀 통합

Telemetry Radios can (optionally) be used to provide a wireless MAVLink connection between a ground control station like _QGroundControl_ and a vehicle running PX4.
지원되는 라디오의 고급 사용법과 새로운 텔레메트리와 PX4 통합 방법을 설명합니다.

:::tip
[Peripheral Hardware > Telemetry Radios](../telemetry/index.md) contains information about telemetry radio systems already supported by PX4.
This includes radios that use the _SiK Radio_ firmware and _3DR WiFi Telemetry Radios_.
:::

## 텔레메트리 통합

PX4는 Pixhawk 비행 콘트롤러의 텔레메트리를 통하여 MAVLink 기반의 원격 측정이 가능합니다.
텔레메트리가 MAVLink를 지원하고, 호환 전압 레벨/커넥터가 있는 UART 인터페이스가 있으면, 통합 작업이 필요하지 않습니다.

다른 프로토콜의 원격 측정 시스템은 잠재적으로 소프트웨어(예: 장치 드라이버)와 하드웨어(커넥터 등)를 포함하는 광범위한 통합 작업이 요구됩니다.
While this has been done for specific cases (e.g. [FrSky Telemetry](../peripherals/frsky_telemetry.md) enables sending vehicle status to an RC controller via an FrSky receiver) providing general advice is difficult.
We recommend you start by [discussing with the development team](../contribute/support.md).
