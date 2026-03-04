# 无线数传

Telemetry Radios can (optionally) be used to provide a wireless MAVLink connection between a ground control station like _QGroundControl_ and a vehicle running PX4.
本节包含两个主题：已经支持的无线数传 和 在PX4系统中集成新的数传。

:::tip
[Peripheral Hardware > Telemetry Radios](../telemetry/index.md) contains information about telemetry radio systems already supported by PX4.
This includes radios that use the _SiK Radio_ firmware and _3DR WiFi Telemetry Radios_.
:::

## 已支持的无线数传

PX4支持通过数传端口将一个基于 MAVLink 的数传连接到Pixhawk飞控。
只需要一个支持MAVLink的数传和一个兼容UART电平/连接器的端口，无需更多。

Telemetry systems that communicate using some other protocol will need more extensive integration, potentially covering both software (e.g. device drivers) and hardware (connectors etc.).
While this has been done for specific cases (e.g. [FrSky Telemetry](../peripherals/frsky_telemetry.md) enables sending vehicle status to an RC controller via an FrSky receiver) providing general advice is difficult.
We recommend you start by [discussing with the development team](../contribute/support.md).
