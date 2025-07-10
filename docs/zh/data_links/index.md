# 数据链路

数据链路是从飞控到地面站（到遥控系统），从地面站发送指令到机体的无线电评到，用于机体遥测通信（位置，速度，电池状态等）。
这些链路通常使用不同的无线电来创建，这些无线电用于手动遥控机体。
PX4 使用 [MAVLink](https://mavlink.io/en/) 协议在无线电频道上传送串行数据。

本节提供关于您可以使用的各种无线电系统的信息，以及如何配置它们与 PX4 一起使用。

- [MAVLink Telemetry (OSD/GCS)](../peripherals/mavlink_peripherals.md) - 配置自驾仪遥测输出
- [Telemetry Radios](../telemetry/index.md) — 受欢迎的数据链路协议和无线电系统
- [FrSky Telemetry](../peripherals/frsky_telemetry.md) — 睿思凯接收机上的遥测
- [TBS Crossfire (CRSF) Telemetry](../telemetry/crsf_telemetry.md) — TBS Crossfire 接收机上的遥测
- [Satellite Comms (Iridium/RockBlock)](../advanced_features/satcom_roadblock.md) — 高延迟卫星通信

## See Also

- [安全配置 > 数据连接丢失的失效保护](../config/safety.md#data-link-loss-failsafe)
