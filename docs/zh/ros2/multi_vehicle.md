# 使用 ROS 2 的多载具模拟

[XRCE-DDS](../middleware/uxrce_dds.md)支持多个客户端通过 UDP 协议连接到同一个代理。
这在模拟中特别有用，因为只有一个代理需要启动。

## 设置和要求

唯一的要求是

- 能够在不依赖 ROS 2 的情况下，使用所需的仿真器([Gazebo](../sim_gazebo_gz/multi_vehicle_simulation.md), [Gazebo Classic](../sim_gazebo_classic/multi_vehicle_simulation.md#multiple-vehicle-with-gazebo-classic), [FlightGear](../sim_flightgear/multi_vehicle.md) and [JMAVSim](../sim_jmavsim/multi_vehicle.md))运行多载具模拟 [multi-vehicle simulation](../simulation/multi-vehicle-simulation.md) 。
- 能够在单一载具模拟中使用 [ROS 2](../ros2/user_guide.md)

## 工作原理

在模拟中，每个 PX4 实例都会获得一个唯一的`px4_instance`编号，编号从`0`开始。
该值用于为 [UXRCE_DDS_KEY](../advanced_config/parameter_reference.md#UXRCE_DDS_KEY)分配一个唯一值：

```sh
参数设置UXRCE_DDS_KEY $((px4_instance+1))
```

:::info
通过这种方式， `UXRCE_DDS_KEY` 将始终与 [MAV_SYS_ID] 保持一致(../advanced_config/parameter_reference.md#MAV_SYS_ID)
:::

此外，当 `px4_instance` 大于 0 时，会添加一个格式为  `px4_$px4_instance` 的唯一 ROS 2[namespace prefix](../middleware/uxrce_dds.md#customizing-the-namespace)：

```sh
uxrce_dds_ns="-n px4_$px4_instance"
```

:::info
环境变量`PX4_UXRCE_DDS_NS` 若已设置，将覆盖上文所述的命名空间行为。
:::

第一个实例(`px4_instance=0`)没有额外的命名空间，此举是为了与真实载具上 xrce-dds 客户端的默认行为保持一致。
这种不匹配可以通过手动使用 `PX4_UXRCE_DDS_NS` 来修复，或者通过从索引 `1` 中添加车辆而不是 `0` (这是Gazebo Classic的  [sitl_multiple_run.sh](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/simulation/gazebo-classic/sitl_multiple_run.sh) 的默认行为)。

模拟中的默认客户端配置概述如下：

| `PX4_UXRCE_DDS_NS` | `px4_instance` | `UXRCE_DDS_KEY`  | client namespace      |
| ------------------ | -------------- | ---------------- | --------------------- |
| not provided       | 0              | `px4_instance+1` | 无                     |
| provided           | 0              | `px4_instance+1` | `PX4_UXRCE_DDS_NS`    |
| not provided       | > 0            | `px4_instance+1` | `px4_${px4_instance}` |
| provided           | > 0            | `px4_instance+1` | `PX4_UXRCE_DDS_NS`    |

## 调整 `target_system` 值

PX4 只在他们的 `target_system` 字段为 0`(路由通信) 或与`MAV_SYS_ID` 一致时，才接受[VehicleCommand](../msg_docs/VehicleCommand.md)。
在所有其他情况下，信息都被忽视。
因此，当 ROS 2 节点需向 PX4 发送`VehicleCommand`消息时，必须确保消息中填写了合适的`target_system\`字段值。

例如，若你想向 `px4_instance=2` 的第三台飞行器发送指令，则需要在所有`VehicleCommand`消息中设置 `target_system=3`。
