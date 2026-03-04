# Data Links

Data links are radio channels that are used to communicate vehicle telemetry (position, velocity, battery status, and so on) from the flight controller to ground control stations (and to some RC systems), and commands from ground stations to the vehicle.
These links are usually created using different radios than those used for manual RC control of the vehicle.
PX4 uses the [MAVLink](https://mavlink.io/en/) protocol for communicating serial data over the radio channel.

This section provides information about various radio systems that you can use, and how to configure them for use with PX4.

- [MAVLink Telemetry (OSD/GCS)](../peripherals/mavlink_peripherals.md) — Configuring autopilot outputs for telemetry
- [Telemetry Radios](../telemetry/index.md) — Popular protocols and radio systems for data links
- [FrSky Telemetry](../peripherals/frsky_telemetry.md) — Telemetry on your (FRSky) RC Receiver
- [TBS Crossfire (CRSF) Telemetry](../telemetry/crsf_telemetry.md) — Telemetry on your (TBS Crossfire) RC Receiver
- [Satellite Comms (Iridium/RockBlock)](../advanced_features/satcom_roadblock.md) — High-latency comms via satellite

## See Also

- [Safety Configuration > Data Link Loss Failsafe](../config/safety.md#data-link-loss-failsafe)
