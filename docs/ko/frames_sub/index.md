# Submarines (Unmanned Underwater Vehicles - UUV)

<LinkedBadge type="warning" text="Experimental" url="../airframes/#experimental-vehicles"/>

:::warning
Support for UUVs is [experimental](../airframes/index.md#experimental-vehicles).
Maintainer volunteers, [contribution](../contribute/index.md) of new features, new frame configurations, or other improvements would all be very welcome!

At time of writing manual and assisted manual modes are available for supported UUV frames, as well as ROS in offboard mode.
The following features have not been implemented:

- Autonomous mission-style underwater workflows are still limited compared to aerial vehicles.
- BlueRobotics gripper support.

:::

PX4 has basic support for UUVs. For BlueROV2 Heavy, PX4 currently supports Manual, Stabilized, Acro, Altitude and Position modes.

## Supported Frames

PX4 supports several unmanned underwater vehicle (UUV) frames.
The set of supported configurations can be seen in [Airframe Reference > Underwater Robots](../airframes/airframe_reference.md#underwater-robot).

### PX4 Compatible (Fully Assembled)

This section lists fully assembled vehicles where you can update the software to run PX4.

- [BlueROV2](../frames_sub/bluerov2.md): Vectored 6 DOF UUV

### Other Frames

- HippoCampus UUV: [Airframe Reference](../airframes/airframe_reference.md#underwater_robot_underwater_robot_hippocampus_uuv_%28unmanned_underwater_vehicle%29), [Gazebo Classic Simulation](../sim_gazebo_classic/vehicles.md#hippocampus-tuhh-uuv)

## 비디오

<lite-youtube videoid="1sUaURmlmT8" title="PX4 on BlueRov Demo"/>

---

<lite-youtube videoid="xSXSoUK-iBM" title="Hippocampus UUV in PX4 SITL Gazebo"/>
