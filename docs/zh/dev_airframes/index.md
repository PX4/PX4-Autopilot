# 机型

PX4 has a flexible [control allocation system](../concept/control_allocation.md) that allows it to support almost any imaginable vehicle type/frame through a single codebase:

- **Planes:** Normal planes, flying wings, inverted V-tail planes, etc.
- **Multicopters:** Helicopters, tricopters, quadcopters, hexarotors, dodecarotors in many different geometries.
- **VTOL Airframes:** VTOL configurations including: Tailsitters, Tiltrotors, and QuadPlanes (plane + quad).
- **UGVs/Rovers:** Basic support has been added for Unmanned Ground Vehicles, enabling both manual and mission-based control.

You can find a list of all supported frame types and motor outputs in the [Airframes Reference](../airframes/airframe_reference.md).

本节提供给希望向PX4添加对新车辆或车辆类型的支持的开发人员相关的信息, 包括仍在开发中的车辆的生成日志。

:::info
PX4 is also well-suited for use in other vehicle types and general robots, ranging from submarine, boats, or amphibious vehicles, through to experimental aircraft and rockets.
_Let us know_ if you have a new vehicle or frame-type you want to help support in PX4.
:::

:::info
Build logs for some of the supported airframes can be found in [Airframe/Vehicle Builds](../airframes/index.md).
:::
