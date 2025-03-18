# PX4-Autopilot Головна нотатка щодо релізу

<Badge type="danger" text="Alpha" />

This contains changes to PX4 `main` branch since the last major release ([PX v1.15](../releases/1.15.md)).

:::warning
The PX4 v1.15 release is in beta testing, pending release.
Update these notes with features that are going to be in `main` but not the PX4 v1.15 release.
:::

## Прочитайте перед оновленням

TBD …

Please continue reading for [upgrade instructions](#upgrade-guide).

## Основні зміни

- Уточнюється

## Інструкції для оновлення

## Інші зміни

### Підтримка обладнання

- Уточнюється

### Загальні

- [Battery level estimation improvements](../config/battery.md). ([PX4-Autopilot#23205](https://github.com/PX4/PX4-Autopilot/pull/23205)).
  - [Voltage-based estimation with load compensation](../config/battery.md#voltage-based-estimation-with-load-compensation) now uses a real-time estimate of the internal resistance of the battery to compensate voltage drops under load (with increased current), providing a better capacity estimate than with the raw measured voltage.
  - Thrust-based load compensation has been removed (along with the `BATn_V_LOAD_DROP` parameters, where `n` is the battery number).
- The [Position (GNSS) loss failsafe](../config/safety.md#position-gnss-loss-failsafe) configurable delay (`COM_POS_FS_DELAY`) has been removed.
  The failsafe will now trigger 1 second after position has been lost. ([PX4-Autopilot#24063](https://github.com/PX4/PX4-Autopilot/pull/24063)).
- [Log Encryption](../dev_log/log_encryption.md) now generates an encrypted log that contains the public-key-encrypted symmetric key that can be used to decrypt it, instead of putting the key into a separate file.
  This makes log decryption much easier, as there is no need to download or identify a separate key file.
  ([PX4-Autopilot#24024](https://github.com/PX4/PX4-Autopilot/pull/24024)).
- The generic mission command timeout [MIS_COMMAND_TOUT](../advanced_config/parameter_reference.md#MIS_COMMAND_TOUT) parameter replaces the delivery-specific `MIS_PD_TO` parameter.
  Mission commands that may take some time to complete, such as those for controlling gimbals, winches, and grippers, will progress to the next item when either feedback is received or the timeout expires.
  This is often used to provide a minimum delay for hardware that does not provide completion feedback, so that it can reach the commanded state before the mission progresses.
  ([PX4-Autopilot#23960](https://github.com/PX4/PX4-Autopilot/pull/23960)).
- **[uORB]** Introduce a [version field](../middleware/uorb.md#message-versioning) for a subset of uORB messages ([PX4-Autopilot#23850](https://github.com/PX4/PX4-Autopilot/pull/23850))
- [Compass calibration](../config/compass.md) disables internal compasses if an external compass is available.
  This typically reduces false warnings due to magnetometer inconsistencies.
  ([PX4-Autopilot#24316](https://github.com/PX4/PX4-Autopilot/pull/24316)).

### Управління

- Уточнюється

### Оцінки

- Уточнюється

### Датчики

- Уточнюється

### Симуляція

- [SIH]:
  - The SIH on SITL [custom takeoff location](../sim_sih/index.md#set-custom-takeoff-location) in now set using the normal unscaled GPS position values, where previously the value needed to be multiplied by 1E7.
    ([PX4-Autopilot#23363](https://github.com/PX4/PX4-Autopilot/pull/23363)).
  - SIH now supports the standard VTOL airframe
    ([PX4-Autopilot#24175](https://github.com/PX4/PX4-Autopilot/pull/24175)).
- [Gazebo]:
  - Gazebo Harmonic LTS release replaces Gazebo Garden as the version supported by PX4.
    The default installer scripts (used for CI) and documentation have been updated.
    This is required because Garden end-of-life is Nov 2024.
    ([PX4-Autopilot#23603](https://github.com/PX4/PX4-Autopilot/pull/23603))
  - New vehicle model `x500_lidar_2d` — [x500 Quadrotor with 2D Lidar](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-2d-lidar). ([PX4-Autopilot#22418](https://github.com/PX4/PX4-Autopilot/pull/22418), [PX4-gazebo-models#41](https://github.com/PX4/PX4-gazebo-models/pull/41)).
  - New vehicle model `x500_lidar_front` — [X500 Quadrotor with 1D LIDAR (Front-facing)](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-1d-lidar-front-facing). ([PX4-Autopilot#23879](https://github.com/PX4/PX4-Autopilot/pull/23879), [PX4-gazebo-models#62](https://github.com/PX4/PX4-gazebo-models/pull/62/files)).
  - New vehicle model `x500_lidar_down` — [X500 Quadrotor with 1D LIDAR (Down-facing)](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-1d-lidar-down-facing). ([PX4-Autopilot#23879](https://github.com/PX4/PX4-Autopilot/pull/23879), [PX4-gazebo-models#62](https://github.com/PX4/PX4-gazebo-models/pull/62/files)).
  - New vehicle model `r1_rover` — [Aion Robotics R1 Rover](../sim_gazebo_gz/vehicles.md#differential-rover) ([PX4-Autopilot#22402](https://github.com/PX4/PX4-Autopilot/pull/22402) and [PX4-gazebo-models#21](https://github.com/PX4/PX4-gazebo-models/pull/21)).
  - New vehicle model `rover_ackermann` — [Ackermann Rover](../sim_gazebo_gz/vehicles.md#ackermann-rover) ([PX4-Autopilot#23383](https://github.com/PX4/PX4-Autopilot/pull/23383) and [PX4-gazebo-models#46](https://github.com/PX4/PX4-gazebo-models/pull/46)).
  - New vehicle model `x500_gimbal` — [Quadrotor(x500) with gimbal (Front-facing) in Gazebo](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-gimbal-front-facing) ([PX4-Autopilot#23382](https://github.com/PX4/PX4-Autopilot/pull/23382) and [PX4-gazebo-models#47](https://github.com/PX4/PX4-gazebo-models/pull/47) and [PX4-gazebo-models#70](https://github.com/PX4/PX4-gazebo-models/pull/70)).
  - New vehicle model `quadtailsitter` — [Quad Tailsitter VTOL](../sim_gazebo_gz/vehicles.md#quad-tailsitter-vtol) ([PX4-Autopilot#23943](https://github.com/PX4/PX4-Autopilot/pull/23943) and [PX4-gazebo-models#65](https://github.com/PX4/PX4-gazebo-models/pull/65)).
  - New vehicle model `tiltrotor` — [Tiltrotor VTOL](../sim_gazebo_gz/vehicles.md#tiltrotor-vtol) ([PX4-Autopilot#24028](https://github.com/PX4/PX4-Autopilot/pull/24028) and [PX4-gazebo-models#66](https://github.com/PX4/PX4-gazebo-models/pull/66)).
  - [Faster than Real-time Simulation](../simulation/index.md#simulation_speed) ([PX4-Autopilot#24421](https://github.com/PX4/PX4-Autopilot/pull/24421), [PX4-Autopilot#23783](https://github.com/PX4/PX4-Autopilot/pull/23783))

### Ethernet

- Уточнюється

### uXRCE-DDS / ROS2

- **[Feature]** <Badge type="warning" text="Experimental"/> [ROS 2 Message Translation Node](../ros2/px4_ros2_msg_translation_node.md) to translate PX4 messages from one definition version to another dynamically ([PX4-Autopilot#24113](https://github.com/PX4/PX4-Autopilot/pull/24113))

### MAVLink

- Уточнюється

### Мульти-Ротор

- Allow system-default [multicopter orbit mode](../flight_modes_mc/orbit.md) yaw behaviour to be configured, using the parameter [MC_ORBIT_YAW_MOD](../advanced_config/parameter_reference.md#MC_ORBIT_YAW_MOD) ([PX4-Autopilot#23358](https://github.com/PX4/PX4-Autopilot/pull/23358))
- Adapted the [Collision Prevention](../computer_vision/collision_prevention.md) implementation to work in the default manual flight mode (Acceleration Based) [MPC_POS_MODE](../advanced_config/parameter_reference.md#MPC_POS_MODE). ([PX4-Autopilot#23507](https://github.com/PX4/PX4-Autopilot/pull/23507)

### VTOL

- Уточнюється

### Літак з фіксованим крилом

- Improvement: Fixed-wing auto takeoff: enable setting takeoff flaps for hand/catapult launch. [PX4-Autopilot#23460](https://github.com/PX4/PX4-Autopilot/pull/23460)

### Ровер

This release contains a major rework for the rover support in PX4:

- Complete restructure of the [rover related documentation](../frames_rover/index.md).
- New firmware build specifically for [rovers](../frames_rover/index.md#flashing-the-rover-build).
- New module dedicated to [Ackermann rovers](../frames_rover/ackermann.md):
  - The module currently supports [manual mode](../flight_modes_rover/ackermann.md#manual-mode), [acro mode](../flight_modes_rover/ackermann.md#acro-mode), [position mode](../flight_modes_rover/ackermann.md#position-mode) and [auto modes](../flight_modes_rover/ackermann.md#auto-modes).
- New module dedicated to [differential rovers](../frames_rover/differential.md):
  - The module currently supports [manual mode](../flight_modes_rover/differential.md#manual-mode), [acro mode](../flight_modes_rover/differential.md#acro-mode), [stabilized mode](../flight_modes_rover/differential.md#stabilized-mode), [position mode](../flight_modes_rover/differential.md#position-mode) and [auto modes](../flight_modes_rover/differential.md#auto-modes).
- New module dedicated to [mecanum rovers](../frames_rover/mecanum.md):
  - The module currently supports [manual mode](../flight_modes_rover/mecanum.md#manual-mode), [acro mode](../flight_modes_rover/mecanum.md#acro-mode), [stabilized mode](../flight_modes_rover/mecanum.md#stabilized-mode), [position mode](../flight_modes_rover/mecanum.md#position-mode) and [auto modes](../flight_modes_rover/mecanum.md#auto-modes).
- Restructure of the [rover airframe](../airframes/airframe_reference.md#rover) numbering convention ([PX4-Autopilot#23506](https://github.com/PX4/PX4-Autopilot/pull/23506)).
  This also introduces several [new rover airframes](../airframes/airframe_reference.md#rover):
  - Generic Differential Rover `50000`.
  - Generic Ackermann Rover `51000`.
  - Axial SCX10 2 Trail Honcho `51001`.
  - Generic Mecanum Rover `52000`.
- Library for the [pure pursuit guidance algorithm](../config_rover/differential.md#pure-pursuit-guidance-logic) that is shared by all the rover modules.
- [Simulation](../frames_rover/index.md#simulation) for differential-steering and Ackermann rovers in gazebo (for release notes see `r1_rover` and `rover_ackermann` in [simulation](#simulation)).
- Deprecation of the [rover position control](../frames_rover/rover_position_control.md) module: Note that the legacy rover module still exists but has been superseded by the new dedicated modules.

### ROS 2

- Уточнюється
