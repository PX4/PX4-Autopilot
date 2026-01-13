# 기체

PX4 has a flexible [control allocation system](../concept/control_allocation.md) that allows it to support almost any imaginable vehicle type/frame through a single codebase:

- **Planes:** Normal planes, flying wings, inverted V-tail planes, etc.
- **Multicopters:** Helicopters, tricopters, quadcopters, hexarotors, dodecarotors in many different geometries.
- **VTOL Airframes:** VTOL configurations including: Tailsitters, Tiltrotors, and QuadPlanes (plane + quad).
- **UGVs/Rovers:** Basic support has been added for Unmanned Ground Vehicles, enabling both manual and mission-based control.

You can find a list of all supported frame types and motor outputs in the [Airframes Reference](../airframes/airframe_reference.md).

이 섹션에서는 개발 중인 차량에 대한 빌드 로그를 포함하여, 신규 차량에 PX4를 지원하는 개발 정보를 제공합니다.

:::info
PX4 is also well-suited for use in other vehicle types and general robots, ranging from submarine, boats, or amphibious vehicles, through to experimental aircraft and rockets.
_Let us know_ if you have a new vehicle or frame-type you want to help support in PX4.
:::

:::info
Build logs for some of the supported airframes can be found in [Airframe/Vehicle Builds](../airframes/index.md).
:::
