# Submarines (Unmanned Underwater Vehicles - UUV)

<LinkedBadge type="warning" text="Experimental" url="../airframes/#experimental-vehicles"/>

:::warning
Support for UUVs is [experimental](../airframes/index.md#experimental-vehicles).
Maintainer volunteers, [contribution](../contribute/index.md) of new features, new frame configurations, or other improvements would all be very welcome!

At time of writing it has only been tested using ROS in offboard mode.
The following features have not been implemented:

- 임무, 뎁스 홀드, 안정화 수동 제어 등과 같은 모드
- BlueRobotics gripper support.

:::

<a href="https://youtu.be/1sUaURmlmT8">유투브</a>

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
