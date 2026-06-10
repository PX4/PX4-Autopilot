# Підводні човни (Безпілотна Підводна Техніка- UUV)

<LinkedBadge type="warning" text="Experimental" url="../airframes/#experimental-vehicles"/>

:::warning
Support for UUVs is [experimental](../airframes/index.md#experimental-vehicles).
Maintainer volunteers, [contribution](../contribute/index.md) of new features, new frame configurations, or other improvements would all be very welcome!

At time of writing manual and assisted manual modes are available for supported UUV frames, as well as ROS in offboard mode.
Наступні функції не були реалізовані:

- Autonomous mission-style underwater workflows are still limited compared to aerial vehicles.
- Підтримка BlueRobotics.

:::

PX4 has basic support for UUVs. For BlueROV2 Heavy, PX4 currently supports Manual, Stabilized, Acro, Altitude and Position modes.

## Підтримувані конструкції

PX4 підтримує кілька безпілотних підводних апаратів (UUV).
The set of supported configurations can be seen in [Airframe Reference > Underwater Robots](../airframes/airframe_reference.md#underwater-robot).

### Сумісний з PX4 (повна збірка)

У цьому розділі перераховані повністю зібрані транспортні засоби, де ви можете оновити програмне забезпечення для роботи з PX4.

- [BlueROV2](../frames_sub/bluerov2.md): Vectored 6 DOF UUV

### Інші ресурси

- HippoCampus UUV: [Airframe Reference](../airframes/airframe_reference.md#underwater_robot_underwater_robot_hippocampus_uuv_%28unmanned_underwater_vehicle%29), [Gazebo Classic Simulation](../sim_gazebo_classic/vehicles.md#hippocampus-tuhh-uuv)

## Відео

<lite-youtube videoid="1sUaURmlmT8" title="PX4 on BlueRov Demo"/>

---

<lite-youtube videoid="xSXSoUK-iBM" title="Hippocampus UUV in PX4 SITL Gazebo"/>
