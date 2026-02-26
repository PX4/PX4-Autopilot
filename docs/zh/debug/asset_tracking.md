# Asset Tracking

<Badge type="tip" text="main (planned for: PX4 v1.18)" />

PX4 can track and log detailed information about external hardware devices connected to the flight controller.
This enables unique identification of vehicle parts throughout their operational lifetime using device IDs, serial numbers, and version information.

:::info
Asset tracking is currently implemented for [DroneCAN](../dronecan/index.md) devices only.
:::

## 综述

Asset tracking allows you to determine exactly which hardware is installed on a vehicle, providing serial number, version, and other information.
This makes it easier to track and maintain specific vehicle parts across multiple vehicles, to quickly see what versions you're running when debugging, and log component information for regulatory audits.

Asset tracking automatically collects and logs the following metadata from external devices:

- **Device identification**: Vendor name, model name, device type
- **Version information**: Firmware version, hardware version
- **Unique identifiers**: Serial number, device ID
- **Device capabilities**: ESC, GPS, magnetometer, barometer, etc.

This information is published via the [`device_information`](../msg_docs/DeviceInformation.md) uORB topic and logged to flight logs.
This enables fleet management, maintenance tracking, and troubleshooting.

## Viewing Device Information

### Real-Time Monitoring

You can view device information in real-time using the [MAVLink Shell](../debug/mavlink_shell.md) or console:

```sh
listener device_information
```

Example output for a CAN GPS module:

```plain
TOPIC: device_information
 device_information
    timestamp: 16258961403 (0.216525 seconds ago)
    device_id: 8944643 (Type: 0x88, UAVCAN:0 (0x7C))
    device_type: 5
    vendor_name: "cubepilot"
    model_name: "here4"
    firmware_version: "1.14.3006590"
    hardware_version: "4.19"
    serial_number: "1c00410018513331"
```

Device information is published in a round-robin fashion for each detected device, at a rate of approximately 1 Hz.

### Multi-Capability Devices

Devices with multiple sensors (e.g., a CAN GPS/magnetometer combo module like the HERE4) register separate device information entries for each capability.
Each entry shares the same serial number and base metadata but has a different `device_id` corresponding to the specific sensor capability.

## 飞行日志分析

Device information is automatically logged to flight logs.
You can extract it using [pyulog](../log/flight_log_analysis.md#pyulog), though note that fields like vendor name, model name, and serial number are stored as `char` arrays and require additional parsing.

## 另见

- [CAN (DroneCAN & Cyphal)](../can/index.md) — CAN bus configuration and setup
- [DroneCAN](../dronecan/index.md) — DroneCAN-specific documentation
- [Flight Log Analysis](../log/flight_log_analysis.md) — Flight log analysis
