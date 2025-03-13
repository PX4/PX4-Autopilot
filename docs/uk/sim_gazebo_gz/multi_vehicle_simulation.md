# Симуляція кількох рухомих засобів з Gazebo

This topic explains how to simulate multiple UAV vehicles using [Gazebo (Gz)](../sim_gazebo_gz/index.md) and SITL.

:::info
Multi-Vehicle Simulation with Gazebo is only supported on Linux.
:::

У Gazebo дуже легко налаштувати сценарії з кількома рухомими засобами (у порівнянні з іншими симуляторами).

Спочатку зберіть код PX4 SITL:

```sh
make px4_sitl
```

Each instance of PX4 can then be launched in its own terminal, specifying a unique instance number and its desired combination of [environment variables](../sim_gazebo_gz/index.md#usage-configuration-options):

```sh
ARGS ./build/px4_sitl_default/bin/px4 [-i <instance>]
```

- `<instance>`:
  The instance number of the vehicle.
  - Кожен рухомий засіб повинен мати унікальний номер екземпляру.
    Якщо він не вказаний, номер екземпляру за замовчуванням - нуль.
  - When used with `PX4_SIM_MODEL`, Gazebo will automatically pick a unique model name in the form `${PX4_SIM_MODEL}_instance`.
- `ARGS`:
  A list of environmental variables, as described in [Gazebo Simulation > Usage/Configuration Options](../sim_gazebo_gz/index.md#usage-configuration-options).

Це дозволяє підвищити гнучкість та адаптивність.

## Кілька рухомих засобів з ROS 2 та Gazebo

[Multiple vehicles with ROS 2](../ros2/multi_vehicle.md) are possible.

- First follow the installation instructions for [Gazebo](../sim_gazebo_gz/index.md).

- Then configure your system for [ROS 2 / PX4 operations](../ros2/user_guide.md#installation-setup).

- В різних терміналах вручну запустіть симуляцію декількох рухомих засобів.
  Цей приклад відтворює 2 квадрокоптери X500 та літак з фіксованим крилом aFPX:

  ```sh
  PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1
  ```

  ```sh
  PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2
  ```

  ```sh
  PX4_SYS_AUTOSTART=4003 PX4_GZ_MODEL_POSE="0,2" PX4_SIM_MODEL=gz_rc_cessna ./build/px4_sitl_default/bin/px4 -i 3
  ```

- Запустіть агента:

  ```sh
  MicroXRCEAgent udp4 -p 8888
  ```

  Агент автоматично з'єднається з усіма клієнтами.

- Run `ros2 topic list` to see the topic list, which should look like this:

```sh
/parameter_events
/px4_1/fmu/in/obstacle_distance
/px4_1/fmu/in/offboard_control_mode
/px4_1/fmu/in/onboard_computer_status
/px4_1/fmu/in/sensor_optical_flow
/px4_1/fmu/in/telemetry_status
/px4_1/fmu/in/trajectory_setpoint
/px4_1/fmu/in/vehicle_attitude_setpoint
/px4_1/fmu/in/vehicle_command
/px4_1/fmu/in/vehicle_mocap_odometry
/px4_1/fmu/in/vehicle_rates_setpoint
/px4_1/fmu/in/vehicle_trajectory_bezier
/px4_1/fmu/in/vehicle_trajectory_waypoint
/px4_1/fmu/in/vehicle_visual_odometry
/px4_1/fmu/out/failsafe_flags
/px4_1/fmu/out/sensor_combined
/px4_1/fmu/out/timesync_status
/px4_1/fmu/out/vehicle_attitude
/px4_1/fmu/out/vehicle_control_mode
/px4_1/fmu/out/vehicle_global_position
/px4_1/fmu/out/vehicle_gps_position
/px4_1/fmu/out/vehicle_local_position
/px4_1/fmu/out/vehicle_odometry
/px4_1/fmu/out/vehicle_status
/px4_2/fmu/in/obstacle_distance
/px4_2/fmu/in/offboard_control_mode
/px4_2/fmu/in/onboard_computer_status
/px4_2/fmu/in/sensor_optical_flow
/px4_2/fmu/in/telemetry_status
/px4_2/fmu/in/trajectory_setpoint
/px4_2/fmu/in/vehicle_attitude_setpoint
/px4_2/fmu/in/vehicle_command
/px4_2/fmu/in/vehicle_mocap_odometry
/px4_2/fmu/in/vehicle_rates_setpoint
/px4_2/fmu/in/vehicle_trajectory_bezier
/px4_2/fmu/in/vehicle_trajectory_waypoint
/px4_2/fmu/in/vehicle_visual_odometry
/px4_2/fmu/out/failsafe_flags
/px4_2/fmu/out/sensor_combined
/px4_2/fmu/out/timesync_status
/px4_2/fmu/out/vehicle_attitude
/px4_2/fmu/out/vehicle_control_mode
/px4_2/fmu/out/vehicle_global_position
/px4_2/fmu/out/vehicle_gps_position
/px4_2/fmu/out/vehicle_local_position
/px4_2/fmu/out/vehicle_odometry
/px4_2/fmu/out/vehicle_status
/px4_3/fmu/in/obstacle_distance
/px4_3/fmu/in/offboard_control_mode
/px4_3/fmu/in/onboard_computer_status
/px4_3/fmu/in/sensor_optical_flow
/px4_3/fmu/in/telemetry_status
/px4_3/fmu/in/trajectory_setpoint
/px4_3/fmu/in/vehicle_attitude_setpoint
/px4_3/fmu/in/vehicle_command
/px4_3/fmu/in/vehicle_mocap_odometry
/px4_3/fmu/in/vehicle_rates_setpoint
/px4_3/fmu/in/vehicle_trajectory_bezier
/px4_3/fmu/in/vehicle_trajectory_waypoint
/px4_3/fmu/in/vehicle_visual_odometry
/px4_3/fmu/out/failsafe_flags
/px4_3/fmu/out/sensor_combined
/px4_3/fmu/out/timesync_status
/px4_3/fmu/out/vehicle_attitude
/px4_3/fmu/out/vehicle_control_mode
/px4_3/fmu/out/vehicle_global_position
/px4_3/fmu/out/vehicle_gps_position
/px4_3/fmu/out/vehicle_local_position
/px4_3/fmu/out/vehicle_odometry
/px4_3/fmu/out/vehicle_status
/rosout
```
