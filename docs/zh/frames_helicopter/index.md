# Helicopters

<LinkedBadge type="warning" text="Experimental" url="../airframes/#experimental-vehicles"/>

:::warning
Support for helicopters is [experimental](../airframes/index.md#experimental-vehicles).
Maintainer volunteers, [contribution](../contribute/index.md) of new features, new frame configurations, or other improvements would all be very welcome!

Issues include:

- Limited support for different types of helicopters.
  For example, PX4 does not support helicopters with coaxial or dual rotor types, and features such as RPM governor and autorotation.

:::

<!-- image here please of PX4 helicopter -->

## Helicopter Types

PX4 supports helicopters with a single main rotor with a swash-plate controlled by up to 4 swash-plate servos, and:

- a mechanically uncoupled tail rotor driven by an ESC, or
- a mechanically coupled tail controlled by a servo on the tail motor.

The allowed flight operations and [flight modes](../flight_modes_mc/index.md) are the same as for multicopter.
Note however that (at the time of writing) 3D flying with negative thrust is not supported in autonomous/guided modes.

## 组装

Assembly of the core autopilot components are similar for all frames.
This is covered in [Basic Assembly](../assembly/index.md).

Helicopter-specific assembly consists mostly of connecting and powering the motors and swash plate servos.

:::info
Note that the flight controller cannot power motors and servos (only GPS module, RC receiver, and low power telemetry modules can be powered from Pixhawk flight controllers).
Generally a power distribution board (PDB) is used to power motors, and a separate (or integrated) battery elimination circuit (BEC) is used to power each of the servos.
:::

## 配置

Helicopter configuration and setup is covered in:

- [Helicopter configuration](../config_heli/index.md): Vehicle frame selection, actuator configuration and testing, and tuning.
- [Standard Configuration](../config/index.md): The common configuration and calibration steps for most frames.
  This includes configuration/calibration of core components such as compass and gyroscope, setting flight mode mappings on a remote control, and safety settings.

## Frame Builds

None available.
