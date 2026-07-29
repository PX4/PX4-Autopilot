# DroneCAN ESCs

PX4 supports DroneCAN compliant ESCs.

## Supported ESC

:::info
[Supported ESCs](../peripherals/esc_motors.md#supported-esc) in _ESCs & Motors_ may include additional devices that are not listed below.
:::

The following articles have specific hardware/firmware information:

- [PX4 Sapog ESC Firmware](sapog.md)
  - [Holybro Kotleta 20](holybro_kotleta.md)
- [Zubax Telega](zubax_telega.md)
- [Vertiq](../peripherals/vertiq.md) (larger modules)
- [VESC Project](../peripherals/vesc.md)
- [RaccoonLab Cyphal and DroneCAN PWM nodes](raccoonlab_nodes.md)

## Hardware Configuration

General DroneCAN hardware configuration is covered in [DroneCAN > Hardware Setup](../dronecan/index.md#hardware-setup).

DroneCAN ESCs should be on their own dedicated CAN interface(s) because ESC messages can saturate the bus and starve other nodes of bandwidth.

## PX4 Configuration

DroneCAN peripherals are configured by following the procedure outlined in [DroneCAN](../dronecan/index.md).

In addition to the general setup, such as setting `UAVCAN_ENABLE` to `3`:

- Select the specific CAN interface(s) used for ESC data output using the [UAVCAN_ESC_IFACE](../advanced_config/parameter_reference.md#UAVCAN_ESC_IFACE) parameter (all that all interfaces are selected by default).
- Configure the [motor order and servo outputs](../config/actuators.md).

## Reversible Motors {#reversible-motors}

<Badge type="tip" text="PX4 v1.18" />

Motors that are configured as [bidirectional](../config/actuators.md#bidirectional-motors) are commanded with a _signed_ `uavcan.equipment.esc.RawCommand` value, as defined by the DroneCAN ESC message: zero is stop, positive is forward and negative is reverse.
Motors that are not configured as bidirectional are commanded with positive values only, as before.

The ESC must support bidirectional operation and interpret negative commands as reverse.
Check the sign of the RPM reported in the ESC telemetry to confirm that the motor really turns the other way.

Reverse is also used by [Motor Failure Recovery](../config/motor_failure_recovery.md) to keep a hexarotor controllable after a motor failure.
