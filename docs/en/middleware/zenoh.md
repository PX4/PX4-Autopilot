# Zenoh (PX4 ROS 2 rmw_zenoh)

<Badge type="tip" text="PX4 v1.17" /> <Badge type="warning" text="Experimental" />

:::warning Experimental
At the time of writing, PX4 Zenoh-pico is experimental, and hence subject to change.
:::

PX4 now supports a new experimental integration with [ROS 2](../ros2/user_guide.md) using the [`rmw_zenoh`](https://github.com/ros2/rmw_zenoh) middleware.
This allows uORB messages to be published and subscribed on a companion computer as though they were ROS 2 topics.
It provides a fast and lightweight way to connect PX4 to ROS 2, making it easier for applications to access vehicle telemetry and send control commands.

PX4 uses a Zenoh-Pico based implementation that leverages [Eclipse Zenoh](https://zenoh.io/) a data-centric protocol designed for real-time, distributed, and resource-constrained environments.
The following guide describes the architecture and various options for setting up the Zenoh client and router.
In particular, it covers the options that are most important to PX4 users exploring Zenoh as an alternative communication layer for ROS 2.

## Architecture

The Zenoh-based middleware consists of a client running on PX4 and a Zenoh router running on the companion computer, with bi-directional data exchange between them over a UART, TCP, UDP, or multicast-UDP link.
The router acts as a broker and discovery service, enabling PX4 to publish and subscribe to topics in the global Zenoh data space.
This allows seamless integration with ROS 2 nodes using rmw_zenoh, and supports flexible deployment across distributed systems.

![Architecture PX4 Zenoh-Pico with ROS 2](../../assets/middleware/zenoh/architecture-px4-zenoh.svg)

:::info
UART is supported by Zenoh but has not yet implemented in the PX4 Zenoh-Pico node.
:::

## ROS 2 Zenoh Bring-up on Linux Companion

In order for PX4 uORB topics to be shared with ROS 2 applications, you will need the PX4 Zenoh-Pico Node client running on your FMU, connected to a Zenoh router (zenohd) running on the companion computer or elsewhere in the network.

On ROS 2 you've to make sure you've selected Zenoh as the `RMW_IMPLEMENTATION` you can set using
```
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

Then you've got to start the Zenoh router using the command below, for more information about the Zenoh Router see the [rmw_zenoh](https://github.com/ros2/rmw_zenoh?tab=readme-ov-file#start-the-zenoh-router) documentation
```
ros2 run rmw_zenoh_cpp rmw_zenohd
```

## PX4 Zenoh-Pico Node setup

Before setting up the Zenoh communication, make sure that your hardware and firmware supports running the PX4 Zenoh-Pico Node.
Refer to the table below for a list of supported PX4 targets with Zenoh integration.

| PX4 Target               | Zenoh Support            | Notes                                            |
|--------------------------|--------------------------|--------------------------------------------------|
| `px4_fmu-v6xrt`          | Built-in Zenoh support   | [FMUv6X-RT](../flight_controller/nxp_mr_vmu_rt1176.md) is the recommended hardware to use with Zenoh   |
| `nxp_tropic-community`   | Built-in Zenoh support   |                                                  |
| `nxp_mr-tropic`          | Built-in Zenoh support   |                                                  |
| `nxp_mr-canhubk344`      | Built-in Zenoh support   |                                                  |
| `px4_sitl_zenoh`         | Special build target     | Zenoh-enabled SITL for simulation                |
| `px4_fmu-v6x_zenoh`      | Special build target     | Zenoh-enabled firmware for FMUv6X                |

---

### 1. Enable Zenoh on PX4 Startup

Enable Zenoh on PX4 Startup you've to set the [ZENOH_ENABLE](../advanced_config/parameter_reference.md#ZENOH_ENABLE) parameter to 1.

### 2. Configure Zenoh Network

Set up PX4 to connect to the companion computer running zenohd. Replace `192.168.1.104` with your Zenoh daemon host IP if different.

```
zenoh config net client tcp/192.168.1.104:7447#iface=eth0
```

:::warning
Any changes to the network configuration require a PX4 system reboot to take effect.
:::

:::tip
px4_sitl_zenoh defaults to localhost, so no changes are needed for SITL.
:::

### 3. PX4 Zenoh-pico node configuration

The **default configuration** is auto-generated from the following [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/zenoh/dds_topics.yaml) file in the PX4 repository


To inspect the current Zenoh configuration, use:

```
zenoh config
```

The PX4 Zenoh-pico node stores its configuration on the **SD card** under the `zenoh` folder. This folder contains three key files:

- **`net.txt`** – Defines the **Zenoh network configuration**.
- **`pub.csv`** – Maps **uORB topics to ROS2 topics** (used for publishing).
- **`sub.csv`** – Maps **ROS2 topics to uORB topics** (used for subscribing).

### 4. Modifying Topic Mappings

Zenoh topic mappings define how data flows between PX4's internal uORB topics and external ROS2 topics via Zenoh. These mappings are stored in `pub.csv` and `sub.csv` on the SD card, and can be modified at runtime using the `zenoh config` CLI tool.

:::warning
Any changes to the topic mappings require a PX4 system reboot to take effect.
:::

There are two types of mappings you can modify:

- **Publisher mappings**: Forward data from a uORB topic to a Zenoh topic.
- **Subscriber mappings**: Receive data from a Zenoh topic and publish it to a uORB topic.

#### To publish a uORB topic to a Zenoh topic, use:

```
zenoh config add publisher <zenoh_topic> <uorb_topic> [uorb_instance]
```

#### To subscribe to a Zenoh topic and forward it to a uORB topic, use:

```
zenoh config add subscriber <zenoh_topic> <uorb_topic> [uorb_instance]
```

#### You can remove existing mappings using the following commands:

```
zenoh config delete publisher <zenoh_topic>
zenoh config delete subscriber <zenoh_topic>
```

After modifying the mappings, reboot PX4 to apply the changes. The updated configuration will be loaded from the SD card during startup.

## Communicating with PX4 from ROS 2 via Zenoh

Once your PX4 FMU is publishing data into ROS 2, you can inspect the available topics and their contents using standard ROS 2 CLI tools:

```
ros2 topic list
```

Check topic type and publishers/subscribers:

```
ros2 topic info -v /fmu/out/vehicle_status
Type: px4_msgs/msg/VehicleStatus

Publisher count: 1

Node name: px4_aabbcc00000000000000000000000000
Node namespace: /
Topic type: px4_msgs/msg/VehicleStatus
Topic type hash: RIHS01_828bddbb7d4c2aa6ad93757955f6893be1ec5d8f11885ec7715bcdd76b5226c9
Endpoint type: PUBLISHER
GID: 82.99.74.2c.b6.7d.93.44.91.4d.fe.14.93.58.40.16
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (7)
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 0
```

### PX4 ROS 2 Interface with Zenoh

The [PX4 ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md) works out of the box with Zenoh as a transport backend. This means you can publish and subscribe to PX4 topics over Zenoh without changing your ROS 2 nodes or dealing with DDS configuration.
For setup details and supported message types, refer to the [PX4 ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md).
