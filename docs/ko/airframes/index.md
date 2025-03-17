# Vehicle Types & Setup

PX4 supports numerous types of vehicles, including different configurations of multicopters, planes, VTOL vehicles, ground vehicles, and so on.

This section explains how to assemble, configure, and tune PX4-based autopilot systems for each type (much of this setup is common to all types).

:::info
[Basic Concepts > Drone Types](../getting_started/px4_basic_concepts.md#drone-types) provides high level information about the types of vehicles and the use cases for which they are best suited.
:::

## Supported Vehicles

The frame types that have a maintainer and are well tested and supported are:

- [Multicopters](../frames_multicopter/index.md) (tri-, quad-, hexa-, octa-, and even [omnicopter](../frames_multicopter/omnicopter.md) vehicles)
- [Planes (Fixed-Wing)](../frames_plane/index.md)
- [VTOL](../frames_vtol/index.md): [Standard VTOL](../frames_vtol/standardvtol.md), [Tailsitter VTOL](../frames_vtol/tailsitter.md), [Tiltrotor VTOL](../frames_vtol/tiltrotor.md)

## 실험 기체

Experimental frames are those vehicle types that:

- Do not have a maintainer.
- Are not regularly tested by the core development team.
- May not be tested in CI.
- May lack features required for production-ready vehicles.
- May not support some common vehicle configurations for the vehicle type.

The following vehicle types are considered experimental:

- [Airships](../frames_airship/index.md)
- [Autogyros](../frames_autogyro/index.md)
- [Balloons](../frames_balloon/index.md)
- [Helicopter](../frames_helicopter/index.md)
- [Rovers](../frames_rover/index.md)
- [Submarines](../frames_sub/index.md)

:::info
Maintainer volunteers, [contribution](../contribute/index.md) of new features, new frame configurations, or other improvements would all be very welcome!
:::

## Other Vehicles

The complete set of supported vehicle types and their configurations can be found in the [Airframes Reference](../airframes/airframe_reference.md).
