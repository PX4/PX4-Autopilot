# Plotting uORB Topic Data in Real Time using PlotJuggler

This topic shows how you can graph the "live" values of [uORB topics](../msg_docs/index.md) (in real time) using [PlotJuggler](../log/flight_log_analysis.md#plotjuggler) and the _uXRCE-DDS Agent_.

This technique uses PX4 [uXRCE-DDS](../middleware/uxrce_dds.md) middleware to export uORB topics as ROS2 topics, which can then be read and plotted by PlotJuggler as they change (PlotJuggler cannot directly read uORB topics, but the values of the corresponding ROS 2 topics are the same).

The video below demonstrates this for a simulated vehicle — the approach works equally well on real hardware.

<video src="../../assets/debug/realtime_debugging/realtime_debugging.mp4" width="720" controls></video>

## 준비 사항

Follow the [ROS 2 Installation & Setup](../ros2/user_guide.md#installation-setup) instructions in the _ROS2 user guide_ to install:

- ROS 2
- [Micro XRCE-DDS Agent](../ros2/user_guide.md#setup-micro-xrce-dds-agent-client)
- [PX4/px4_msgs](https://github.com/PX4/px4_msgs): PX4/ROS2 shared message definitions.
- PX4 source code and build the simulator.

  ::: tip
  If you're using real hardware instead of the simulator, you will only need PX4 source code if you need to change the set of topics that are published to ROS 2 (only a subset of uORB topics are published by default).

:::

You will also need to install:

- [PlotJuggler for ROS2](https://github.com/facontidavide/PlotJuggler)

  ::: tip
  Use the Debian packages (the snap files are not supported).

:::

## 사용법

First we need to build a ROS 2 workspace that includes the `px4_msgs` that correspond to the PX4 build to be monitored, and then launch PlotJuggler from within that workspace.
This allows ROS 2 and PlotJuggler to interpret the messages.
If you're using unmodified PX4, the definitions from [PX4/px4_msgs](https://github.com/PX4/px4_msgs) can be used.

:::info
This is the same process as covered in [Build ROS 2 Workspace](../ros2/user_guide.md#build-ros-2-workspace) in _ROS 2 Installation & Setup_.
:::

Assuming your ROS 2 workspace is named `~/ros2_ws/`, fetch and build the `px4_msgs` package in a terminal as shown:

```sh
cd ~/ros2_ws/src/
git clone https://github.com/PX4/px4_msgs.git
cd ..
colcon build
source install/setup.bash
```

Then run PlotJuggler by entering the following commands in a terminal:

```sh
ros2 run plotjuggler plotjuggler
```

To start sending ROS 2 topics from PX4, the uXRCE-DDS **client** has to be running on PX4, and the `MicroXRCEAgent` has to be running on the same computer as PlotJuggler.

### PX4 Simulator

Next we'll start the [Gazebo](../sim_gazebo_gz/index.md) simulator for a quadcopter.
Because we're using a PX4 simulator the client is started automatically, but we will still need to start the agent and connect to the client.

First open another terminal.
Then navigate to the root of the PX4 source code and start the simulator using the following commands:

```sh
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Open another terminal and start the `MicroXRCEAgent` to connect to the the simulator:

```sh
MicroXRCEAgent udp4 -p 8888; exec bash
```

That's all that should be needed for connecting to the simulator.

### PX4 on Hardware

If you're working with real hardware you'll need to explicitly start the client on PX4 and your agent connection command will be slightly different.
[Using flight controller hardware](../ros2/user_guide.md#using-flight-controller-hardware) in the _ROS 2 User Guide_ provides links to setup information.

## Unavailable/New Messages

All PX4 message definitions from `main` are exported to the [PX4/px4_msgs](https://github.com/PX4/px4_msgs) repository.
These must be imported into your ROS 2 workspace, allowing PlotJuggler to interpret messages from PX4.

:::info
Exporting the messages allows ROS 2 and the uXRCE-DDS agent to be independent of PX4, which is why you only need the PX4 source code if you need to build the simulator or modify the messages.
:::

While `px4_msgs` has messages for all uORB topics in PX4, not all messages in `px4_msgs` are available to ROS 2/PlotJuggler by default.
The set that are available must be built into the client running on PX4.
These are defined in [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml).

The instructions below explain the changes needed to monitor topics that are not available by default.

### Missing Topics

If a normal uORB topic is not available in PlotJuggler you will need to modify the [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml) to include the topic and rebuild PX4.

If working with real hardware you will need to build and [install](../config/firmware.md#installing-px4-main-beta-or-custom-firmware) custom firmware after changing the YAML file.

### Modified Messages

If you have modified any uORB messages you must update the ROS2 messages used by PlotJuggler.

You will need to rebuild PX4 with your new messages, and replace the `px4_msgs` (from the repository) in your workspace with the new ones.

Assuming that you have already built PX4 in the directory `~/PX4-Autopilot/`, and that `~/ros2_ws` is your ROS2 workspace, enter the following commands to copy the messages across and rebuild your workspace:

```sh
rm -f ~/ros2_ws/src/px4_msgs/msg/*.msg
cp ~/PX4-Autopilot/msg/*.msg ~/ros2_ws/src/px4_msgs/msg/
cd ~/ros2_ws/ && colcon build
```

### Custom Topics

After defining the topic, follow the instructions above to add the topic to [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml), and export the new message into your ROS 2 workspace.

## See also

[ROS 2 User Guide](../ros2/user_guide.md)
