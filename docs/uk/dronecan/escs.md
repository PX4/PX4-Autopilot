# DroneCAN ESCs

PX4 підтримує ESCs, які відповідають стандарту DroneCAN.
Для отримання додаткової інформації дивіться наступні статті для конкретного обладнання/прошивки:

- [PX4 Sapog ESC Firmware](sapog.md)
  - [Holybro Kotleta 20](holybro_kotleta.md)
- [Zubax Telega](zubax_telega.md)
- [Vertiq](../peripherals/vertiq.md) (larger modules)
- [VESC Project](../peripherals/vesc.md)
- [RaccoonLab Cyphal and DroneCAN PWM nodes](raccoonlab_nodes.md)

## Конфігурація апаратного забезпечення

General DroneCAN hardware configuration is covered in [DroneCAN > Hardware Setup](../dronecan/index.md#hardware-setup).

DroneCAN ESCs should be on their own dedicated CAN interface(s) because ESC messages can saturate the bus and starve other nodes of bandwidth.

## Конфігурація PX4

DroneCAN peripherals are configured by following the procedure outlined in [DroneCAN](../dronecan/index.md).

In addition to the general setup, such as setting `UAVCAN_ENABLE` to `3`:

- Select the specific CAN interface(s) used for ESC data output using the [UAVCAN_ESC_IFACE](../advanced_config/parameter_reference.md#UAVCAN_ESC_IFACE) parameter (all that all interfaces are selected by default).
- Configure the [motor order and servo outputs](../config/actuators.md).
