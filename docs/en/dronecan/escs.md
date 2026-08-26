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

When ESC output is enabled, PX4 publishes [ArmingStatus](https://dronecan.github.io/Specification/7._List_of_standard_data_types/#armingstatus) (`STATUS_FULLY_ARMED` while armed or during QGC actuator tests).
ESCs that require that message work for both flight and the Actuators page without setting [UAVCAN_PUB_ARM](../advanced_config/parameter_reference.md#UAVCAN_PUB_ARM).

## Reversible Motors {#reversible-motors}

<Badge type="tip" text="main (PX4 v2.0)" />

Motors can be reversible "on the fly" (bidirectional) if the motor hardware supports reversal, the ESC firmware is configured for 3D/bidirectional operation, and the motor is set as [bidirectional](../config/actuators.md#bidirectional-motors) in the actuator configuration.

When configured as bidirectional, PX4 commands the motor with a _signed_ `uavcan.equipment.esc.RawCommand` value, as defined by the DroneCAN ESC message protocol:

- **Zero:** Motor stop / neutral
- **Positive values:** Forward thrust
- **Negative values:** Reverse thrust

Motors that are not configured as bidirectional continue to receive positive values only.

::: tip Verifying Operation
Check the sign of the RPM reported in the ESC telemetry to confirm that the ESC is interpreting negative commands correctly and actively spinning the motor in reverse.
:::

Reversible motors may also be used in [Motor Failure Recovery](../config/motor_failure_recovery.md) to keep a hexarotor controllable after a single motor failure.
That case still needs ESCs configured for reversal.
However it is not necessary to set the motor as bidirectional in the actuator configuration, because PX4 makes the recovery motor reversible as it handles the failure.
