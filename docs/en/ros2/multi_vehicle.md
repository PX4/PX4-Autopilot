# Multi-Vehicle Simulation with ROS 2

[XRCE-DDS](../middleware/uxrce_dds.md) allows for multiple clients to be connected to the same agent over UDP.
This is particular useful in simulation as only one agent needs to be started.

## Setup and Requirements

The only requirements are

- To be able to run [multi-vehicle simulation](../simulation/multi-vehicle-simulation.md) without ROS 2 with the desired simulator ([Gazebo](../sim_gazebo_gz/multi_vehicle_simulation.md), [Gazebo Classic](../sim_gazebo_classic/multi_vehicle_simulation.md#multiple-vehicle-with-gazebo-classic), [FlightGear](../sim_flightgear/multi_vehicle.md) and [JMAVSim](../sim_jmavsim/multi_vehicle.md)).
- To be able to use [ROS 2](../ros2/user_guide.md) in a single vehicle simulation.

## Principle of Operation

In simulation each PX4 instance receives a unique `px4_instance` number starting from `0`.
This value is used to assign a unique value to [UXRCE_DDS_KEY](../advanced_config/parameter_reference.md#UXRCE_DDS_KEY):

```sh
param set UXRCE_DDS_KEY $((px4_instance+1))
```

::: info
By doing so, `UXRCE_DDS_KEY` will always coincide with [MAV_SYS_ID](../advanced_config/parameter_reference.md#MAV_SYS_ID).
:::

Moreover, when `px4_instance` is greater than zero, a unique ROS 2 [namespace prefix](../middleware/uxrce_dds.md#customizing-the-namespace) in the form `px4_$px4_instance` is added:

```sh
uxrce_dds_ns="-n px4_$px4_instance"
```

::: info
The environment variable `PX4_UXRCE_DDS_NS`, if set, will override the namespace behavior described above.
:::

The first instance (`px4_instance=0`) does not have an additional namespace in order to be consistent with the default behavior of the `xrce-dds` client on a real vehicle.
This mismatch can be fixed by manually using `PX4_UXRCE_DDS_NS` on the first instance or by starting adding vehicles from index `1` instead of `0` (this is the default behavior adopted by [sitl_multiple_run.sh](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/simulation/gazebo-classic/sitl_multiple_run.sh) for Gazebo Classic).

The default client configuration in simulation is summarized as follows:

| `PX4_UXRCE_DDS_NS` | `px4_instance` | `UXRCE_DDS_KEY`   | client namespace      |
|-------------------|----------------|------------------|-----------------------|
| not provided      | 0              | `px4_instance+1` | none                  |
| provided          | 0              | `px4_instance+1` | `PX4_UXRCE_DDS_NS`     |
| not provided      | >0             | `px4_instance+1` | `px4_${px4_instance}` |
| provided          | >0             | `px4_instance+1` | `PX4_UXRCE_DDS_NS`     |

## Adjusting the `target_system` value

PX4 accepts [VehicleCommand](../msg_docs/VehicleCommand.md) messages only if their `target_system` field is zero (broadcast communication) or coincides with `MAV_SYS_ID`.
In all other situations, the messages are ignored.
Therefore, when ROS 2 nodes want to send `VehicleCommand` to PX4, they must ensure that the messages are filled with the appropriate `target_system` value.

For example, if you want to send a command to your third vehicle, which has `px4_instance=2`, you need to set `target_system=3` in all your `VehicleCommand` messages.
