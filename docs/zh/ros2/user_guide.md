# ROS 2 用户指南

ROS 2-PX4 架构在ROS 2和PX4之间进行了深度整合。 允许 ROS 2 订阅或发布节点直接使用 PX4 uORB 话题。

本指南介绍了系统架构和应用程序流程，并解释了如何与PX4一起安装和使用ROS2。

:::info
从 PX4 v1.14, ROS 2 使用 [uXRCE-DDS](../middleware/uxrce_dds.md) 中间件替换版本 1 中使用的 _FastRTPS_ 中间件. 3 (v1.13不支持uXRCE-DDS)。

[migration guide](../middleware/uxrce_dds.md#fast-rtps-to-uxrce-dds-migration-guidelines) 解释您需要做什么来将ROS2 应用程序从 PX4 v1.13 迁移到 PX4 v1.14。

如果您仍然在 PX4 v1.13 上工作，请按照[PX4 v1.13 文档](https://docs.px4.io/v1.13/en/ros/ros2_comm.html)中的说明操作。

<!-- remove this when there are PX4 v1.14 docs for some months -->

:::

## 综述

得益于 uXRCE-DDS(../middleware/uxrce_dds.md) 通信中间件的使用，ROS 2 的应用流程非常简单直接。

![Architecture uXRCE-DDS with ROS 2](../../assets/middleware/xrce_dds/architecture_xrce-dds_ros2.svg)

<!-- doc source: https://docs.google.com/drawings/d/1WcJOU-EcVOZRPQwNzMEKJecShii2G4U3yhA3U6C4EhE/edit?usp=sharing -->

uXRCE-DDS 中间件由两部分组成：一部分是运行在 PX4 上的客户端，另一部分是运行在伴飞计算机上的代理；二者之间通过串口、UDP、TCP或自定义链路进行双向数据交换。
代理充当客户端的代理角色，以便在全局 DDS 数据空间中发布和订阅主题。

PX4 [uxrce_dds_client](../modules/modules_system.md#uxrce-dds-client) 是在构建时生成，并且默认包含在 PX4 固件中。
它包含“通用”XRCE-DDS客户端代码和它用来发布到/来自uORB主题的 PX4 特定转换代码。
生成到客户端中的 uORB 消息子集在 [dds_topics.yaml](../middleware/dds_topics.md)中说明。
生成器使用源代码树中的 uORB 消息定义：[PX4-Autopilot/msg](https://github.com/PX4/PX4-Autopilot/tree/main/msg) 用于生成发送 ROS 2 消息的代码。

ROS 2 应用程序需要在一个工作空间中构建，该工作空间需包含与 PX4 固件中创建 uXRCE-DDS 客户端模块时所用完全相同的消息定义。
您可以通过克隆接口包[PX4/px4_msgs](https://github.com/PX4/px4_msgs)将这些内容纳入您的 ROS 2 工作空间(repo 中的范围与不同的 PX4 版本的消息相对应)。

从 PX4 v1.16 版本开始[message versioning](../middleware/uorb.md#message-versioning)，ROS 2 应用程序所使用的消息定义版本，可与构建 PX4 时所用的消息定义版本不同。
这需要[ROS 2 Message Translation Node](../ros2/px4_ros2_msg_translation_node.md)运行ROS 2 消息转换节点，以确保消息能够正确转换和交互。

需要注意的是，微型XRCE-DDS _agent_ 本身并不依赖客户端代码。
它可以从 [source](https://github.com/eProsima/Micro-XRCE-DDS-Agent）中单独构建，或者作为ROS构建的一部分，或者作为snap包安装。

在使用 ROS 2 时，您通常需要同时启动客户端和代理。
需要注意的是，uXRCE-DDS 客户端默认已内置到固件中，但除仿真器构建版本外，不会自动启动。

:::info
在 PX4 v1.13 及更早版本中，ROS 2 依赖于 [px4_ros_com](https://github.com/PX4/px4_ros_com) 中的消息定义。
该代码仓库已不再需要，但其中包含一些实用的示例。
:::

## 安装与设置

支持和推荐使用 PX4 的 ROS 2 平台是 Ubuntu 的 ROS 2 “简易” LTS 22.04。

:::tip
如果您在 Ubuntu 20.04 上工作，我们建议您更新到 Ubuntu 22.04。
同时，你可以在 Ubuntu 20.04 上使用 [Gazebo Class](../sim_gazebo_classic/index.md) 的 ROS 2 "Foxy" 。
请注意，第二号外空系统“Foxy”在2023年5月到达寿命终结，但在撰写本报告时仍然稳定并与PX4合作。
:::

安装使用 PX4 的 ROS 2：

- [Install PX4](#install-px4) (to use the PX4 simulator)
- [Install ROS 2](#install-ros-2)
- [Setup Micro XRCE-DDS Agent & Client](#setup-micro-xrce-dds-agent-client)
- [Build & Run ROS 2 Workspace](#build-ros-2-workspace)

该架构中会自动安装的其他依赖项（如 Fast DDS）未在此处提及。

### 安装PX4

若要使用该仿真器，你需要安装 PX4 开发工具链。

:::info
唯一依赖于ROS2的 PX4 是一组信息定义，它从 [px4_msgs](https://github.com/PX4/px4_msgs)获取。
您只需要安装 PX4 当您需要模拟器时(如我们在本指南中所做的那样)， 或者如果您正在创建一个发布自定义uORB主题的构建。
:::

通过以下方式在 Ubuntu 上配置一个 PX4 开发环境：

```sh
cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
```

请注意，上述命令将为您的Ubuntu版本安装推荐的模拟器。
如果您想要安装 PX4，但保留您现有的模拟器安装，请使用 "--no-sim-tools" 标志运行 `ubuntu.sh`。

欲了解更多信息和故障排除，请参阅：[Ubuntu 开发环境](../dev_setup/dev_env_linux_ubuntu.md) 和 [下载 PX4 源](../dev_setup/building_px4.md)。

### 安装 ROS 2

安装 ROS 2 及其依赖：

1. 安装 ROS 2.

   :::: tabs

   ::: tab humble
   To install ROS 2 "Humble" on Ubuntu 22.04:

   ```sh
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update && sudo apt upgrade -y
   sudo apt install ros-humble-desktop
   sudo apt install ros-dev-tools
   source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
   ```

   以上说明转载自官方安装指南：[Install ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)。
   您可以安装 _either_ the desktop (`ros-humble-desktop`) _or_ bare-bones versions (`ros-humble-ros-base`), _and_ the development tools (`ros-dev-tools`).

:::

   ::: tab foxy
   To install ROS 2 "Foxy" on Ubuntu 20.04:

   - 按照官方安装指南： [Install ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

   您可以安装 _either_ the desktop (`ros-foxy-desktop`) _or_ bare-bones versions (`ros-foxy-ros-base`), _and_ the development tools (`ros-dev-tools`).

:::

   ::::

2. 一些Python 依赖关系也必须安装 (使用 **`pip`** 或 **`apt`**):

   ```sh
   pip install --user -U empy==3.3.4 pyros-genmsg setuptools
   ```

### 配置微型 XRCE-DDS 代理与客户端

要实现 ROS 2 与 PX4 的通信，[uXRCE-DDS client](../modules/modules_system.md#uxrce-dds-client)必须在 PX4 上运行，且需与运行在机载计算机上的微型 XRCE-DDS 代理建立连接。

#### 设置代理(Agent)

代理可以安装在机载计算机上 [number of ways](../middleware/uxrce_dds.md#micro-xrce-dds-agent-installation)。
下文将介绍如何从源代码 “独立” 构建代理，并连接到运行在 PX4 仿真器上的客户端。

设置并启动代理：

1. 打开一个终端。

2. 输入以下命令从仓库获取源代码并构建代理(Agent)：

   ```sh
   git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
   cd Micro-XRCE-DDS-Agent
   mkdir build
   cd build
   cmake ..
   make
   sudo make install
   sudo ldconfig /usr/local/lib/
   ```

3. 启动代理并设置以连接运行在模拟器上的 uXRCE-DDS客户端(Client)：

   ```sh
   MicroXRCEAgent udp4 -p 8888
   ```

代理现已启动，但在我们启动 PX4（下一步） 之前，你不会看到太多。

:::info
你可以让代理在这个终端中保持运行状态！
需注意，每个连接通道仅允许运行一个代理
:::

#### 启动客户端(Client)

PX4 仿真器会自动启动 uXRCE-DDS 客户端，并连接到本地主机上的 UDP 8888 端口。

启动模拟器(和客户端)：

1. 在之前安装好的 PX4 自动驾驶仪 代码仓库的根目录下，打开一个新的终端。

   :::: tabs

   ::: tab humble

   - 使用 PX4 [Gazebo](../sim_gazebo_gz/index.md) 模拟：

     ```sh
     make px4_sitl gz_x500
     ```


:::

   ::: tab foxy

   - 使用 PX4 [Gazebo Classic](../sim_gazebo_classic/index.md) 模拟：

     ```sh
     make px4_sitl gazebo-classic
     ```


:::

   ::::

代理和客户端现已运行并二者应已建立连接。

PX4 终端会显示  [NuttShell/PX4 System Console](../debug/system_console.md) 系统控制台 的输出内容，该输出会在 PX4 启动和运行过程中实时呈现。
代理一建立连接，输出内容中就应包含 INFO 级别的消息，这些消息会显示数据撰写器的创建情况：

```sh
...
INFO  [uxrce_dds_client] synchronized with time offset 1675929429203524us
INFO  [uxrce_dds_client] successfully created rt/fmu/out/failsafe_flags data writer, topic id: 83
INFO  [uxrce_dds_client] successfully created rt/fmu/out/sensor_combined data writer, topic id: 168
INFO  [uxrce_dds_client] successfully created rt/fmu/out/timesync_status data writer, topic id: 188
```

微型 XRCE-DDS 代理终端也应开始显示输出内容，因为在 DDS 网络中会创建对应的主题：

```sh
...
[1675929445.268957] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x00000001, publisher_id: 0x0DA(3), participant_id: 0x001(1)
[1675929445.269521] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x00000001, datawriter_id: 0x0DA(5), publisher_id: 0x0DA(3)
[1675929445.270412] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x00000001, topic_id: 0x0DF(2), participant_id: 0x001(1)
```

### 构建ROS 2 工作空间

本节介绍如何在你的主目录中创建一个 ROS 2 工作空间（可根据需要修改命令，将源代码放置到其他位置）。

[px4_ros_com](https://github.com/PX4/px4_ros_com) 和 [px4_msgs](https://github.com/PX4/px4_msgs) 这两个功能包会被克隆到工作空间文件夹中，之后使用 colcon 工具对该工作空间进行构建
此示例使用 "ros2 launch" 运行。

您应该使用一个 px4_msgs 包的版本与 \_same_ 消息定义作为您已经安装在上面步骤中的 PX4 固件。
px4_msgs 代码仓库中的分支均以特定名称命名，这些名称与不同 PX4 版本的消息定义一一对应。
如果出于任何原因，您不能确保您的 PX4 固件和 ROS 2 px4_msgs 包之间具有相同的消息定义。 您还需要 [start the message translation node](#optional-starting-the-translation-node)，作为您设置过程的一部分。

:::info
该示例会构建 [ROS 2 Listener](#ros-2-listener) 示例应用程序，该程序位于  [px4_ros_com](https://github.com/PX4/px4_ros_com)中。
[px4_msgs](https://github.com/PX4/px4_msgs) 也是需要的，以便示例能够解释PX4 ROS 2 主题。
:::

#### 构建工作空间

要创建和构建工作空间：

1. 打开一个新的终端。

2. 使用以下方式创建并进入一个新的工作空间目录：

   ```sh
   mkdir -p ~/ws_sensor_combined/src/
   cd ~/ws_sensor_combined/src/
   ```

   ::: info
   一个为工作空间文件夹制定命名规范，有助于更轻松地管理工作空间。

:::

3. 将示例代码仓库和 [px4_msgs](https://github.com/PX4/px4_msgs) 克隆到 /src 目录下（默认克隆 main 分支，该分支与我们当前运行的 PX4 版本相对应）：

   ```sh
   git clone https://github.com/PX4/px4_msgs.git
   git clone https://github.com/PX4/px4_ros_com.git
   ```

4. 在当前终端中加载 ROS 2 开发环境，并使用 colcon 工具编译工作空间：

   :::: tabs

   ::: tab humble

   ```sh
   cd ..
   source /opt/ros/humble/setup.bash
   colcon build
   ```


:::

   ::: tab foxy

   ```sh
   cd ..
   source /opt/ros/foxy/setup.bash
   colcon build
   ```


:::

   ::::

   该操作会使用已加载的工具链对 /src 目录下的所有文件夹进行构建。

#### 运行示例

要运行你刚刚构建好的可执行文件，需加载local_setup.bash 。
这提供了当前工作空间的 "environment hooks"访问权限。
换句话说，它会让刚刚构建好的可执行文件在当前终端中可用。

:::info
[ROS2 beginner tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay)建议您 _open a new terminal_来运行您的可执行文件。
:::

在新终端中：

1. 进入工作空间目录的顶层，并加载 ROS 2 环境（本例中为 “Humble” 版本）：

   :::: tabs

   ::: tab humble

   ```sh
   cd ~/ws_sensor_combined/
   source /opt/ros/humble/setup.bash
   ```


:::

   ::: tab foxy

   ```sh
   cd ~/ws_sensor_combined/
   source /opt/ros/foxy/setup.bash
   ```


:::

   ::::

2. 加载 local_setup.bash

   ```sh
   source install/local_setup.bash
   ```

3. 现在启动示例。
   请注意，此处我们使用的是 ros2 launch，其相关说明如下。

   ```sh
   ros2 launch px4_ros_com sensor_combined_listener.launch.py
   ```

若此功能正常运行，你应能在启动 ROS 监听器的终端 / 控制台上看到数据正在打印输出

```sh
RECEIVED DATA FROM SENSOR COMBINED
================================
ts: 870938190
gyro_rad[0]: 0.00341645
gyro_rad[1]: 0.00626475
gyro_rad[2]: -0.000515705
gyro_integral_dt: 4739
accelerometer_timestamp_relative: 0
accelerometer_m_s2[0]: -0.273381
accelerometer_m_s2[1]: 0.0949186
accelerometer_m_s2[2]: -9.76044
accelerometer_integral_dt: 4739
```

#### (可选) 启动转化节点

<0/> <1/>

此示例由 PX4 和ROS 2 版本构建，它们使用相同的消息定义。
若你要使用不兼容的 [message versions](../middleware/uorb.md#message-versioning)，则在运行示例之前，还需要安装并运行[Message Translation Node](./px4_ros2_msg_translation_node.md)：

1. 通过运行以下脚本，将 [Message Translation Node](../ros2/px4_ros2_msg_translation_node.md) 纳入示例工作空间或单独的工作空间中

   ```sh
   cd /path/to/ros_ws
   /path/to/PX4-Autopilot/Tools/copy_to_ros_ws.sh .
   ```

2. 构建并运行转化节点：

   ```sh
   colcon build
   source install/local_setup.bash
   ros2 run translation_node translation_node_bin
   ```

## 控制机体

要控制应用，ROS 2 应用程序：

- 订阅 (聆听) PX4 发布的数传主题
- 发布到导致PX4执行某些操作的主题。

您可以使用的主题定义在[dds_topics.yaml](../middleware/dds_topics.md)， 并且您可以在  [uORB Message Reference](../msg_docs/index.md)获取更多关于他们数据的信息。
例如，[VehicleGlobalPosition](../msg_docs/VehicleGlobalPosition.md) 可以用来获得机体的全局位置。 [VehicleCommand](../msg_docs/VehicleCommand.md) 可以用于命令诸如起飞和降落等操作。

下面的 [ROS 2 Example applications](#ros-2-example-applications)  示例提供了如何使用这些主题的具体例子。

## 兼容性问题

本节包含的信息可能会影响你编写 ROS 代码的方式。

### ROS 2 订阅者QoS 设置

用于订阅 PX4 发布的话题的 ROS 2 代码，必须指定合适（兼容）的 QoS（服务质量）设置，才能监听这些话题。
具体而言，节点应使用 ROS 2 预定义的 QoS 传感器数据（可参考[listener example source code](#ros-2-listener)）进行订阅：

```cpp
...
rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

subscription_ = this->create_subscription<0>("/fmu/out/sensor_combined", qos,
...
```

需要这样做的原因是，ROS 2 的默认 [Quality of Service (QoS) settings](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html#qos-profiles)与 PX4 所使用的设置不同。
并非所有发布者 - 订阅者的 [Qos settings are possible](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html#qos-compatibilities)，而事实证明，ROS 2 默认的订阅者设置就属于不可行的情况！需注意，ROS 代码在发布时无需设置 QoS参数（在此场景下，PX4 的 QoS 设置与 ROS 的默认 QoS 设置是兼容的）。

<!-- From https://github.com/PX4/PX4-user_guide/pull/2259#discussion_r1099788316 -->

### ROS 2 & PX4 坐标系公约

ROS与 PX4所使用的本地 / 世界坐标系和机体坐标系存在差异。

| 框架    | ROS                                                                 | ROS                                                              |
| ----- | ------------------------------------------------------------------- | ---------------------------------------------------------------- |
| 机体    | FRD (X **F**orward, Y **R**ight, Z **D**own)     | FLU (X **F**orward, Y **L**eft, Z **U**p)     |
| 世界坐标系 | FRD or NED (X **N**orth, Y **E**ast, Z **D**own) | FLU 或 ENU (X **E**ast, Y **N**orth, Z **U**p) |

:::tip
See [REP105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html) for more information about ROS frames.
:::

如果你把机体命名为 <code>robot1</code>，你会得到一个主题，比如 <code>/vrpn_client_node/robot1/pose</code>

![Reference frames](../../assets/lpe/ref_frames.png)

The FRD (NED) conventions are adopted on **all** PX4 topics unless explicitly specified in the associated message definition.
Therefore, ROS 2 nodes that want to interface with PX4 must take care of the frames conventions.

- To rotate a vector from ENU to NED two basic rotations must be performed:
  - first a pi/2 rotation around the `Z`-axis (up),
  - then a pi rotation around the `X`-axis (old East/new North).

- To rotate a vector from NED to ENU two basic rotations must be performed:

- - first a pi/2 rotation around the `Z`-axis (down),
  - then a pi rotation around the `X`-axis (old North/new East). Note that the two resulting operations are mathematically equivalent.

- To rotate a vector from FLU to FRD a pi rotation around the `X`-axis (front) is sufficient.

- To rotate a vector from FRD to FLU a pi rotation around the `X`-axis (front) is sufficient.

Examples of vectors that require rotation are:

- all fields in [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md) message; ENU to NED conversion is required before sending them.
- all fields in [VehicleThrustSetpoint](../msg_docs/VehicleThrustSetpoint.md) message; FLU to FRD conversion is required before sending them.

Similarly to vectors, also quaternions representing the attitude of the vehicle (body frame) w.r.t. the world frame require conversion.

[PX4/px4_ros_com](https://github.com/PX4/px4_ros_com) provides the shared library [frame_transforms](https://github.com/PX4/px4_ros_com/blob/main/include/px4_ros_com/frame_transforms.h) to easily perform such conversions.

### ROS, Gazebo and PX4 time synchronization

By default, time synchronization between ROS 2 and PX4 is automatically managed by the [uXRCE-DDS middleware](https://micro-xrce-dds.docs.eprosima.com/en/latest/time_sync.html) and time synchronization statistics are available listening to the bridged topic `/fmu/out/timesync_status`.
When the uXRCE-DDS client runs on a flight controller and the agent runs on a companion computer this is the desired behavior as time offsets, time drift, and communication latency, are computed and automatically compensated.

For Gazebo simulations the GZBridge sets the PX4 time on every sim step (see [Change simulation speed](../sim_gazebo_gz/index.md#change-simulation-speed)).
Note that this is different from the [simulation lockstep](../sim_gazebo_classic/index.md#lockstep) procedure adopted with Gazebo Classic.

ROS2 users have then two possibilities regarding the [time source](https://design.ros2.org/articles/clock_and_time.html) of their nodes.

#### ROS2 nodes use the OS clock as time source

This scenario, which is the one considered in this page and in the [offboard_control](./offboard_control.md) guide, is also the standard behaviour of the ROS2 nodes.
The OS clock acts as time source and therefore it can be used only when the simulation real time factor is very close to one.
The time synchronizer of the uXRCE-DDS client then bridges the OS clock on the ROS2 side with the Gazebo clock on the PX4 side.
No further action is required by the user.

#### ROS2 nodes use the Gazebo clock as time source

In this scenario, ROS2 also uses the Gazebo `/clock` topic as time source.
This approach makes sense if the Gazebo simulation is running with real time factor different from one, or if ROS2 needs to directly interact with Gazebo.
On the ROS2 side, direct interaction with Gazebo is achieved by the [ros_gz_bridge](https://github.com/gazebosim/ros_gz) package of the [ros_gz](https://github.com/gazebosim/ros_gz) repository.

Use the following commands to install the correct ROS 2/gz interface packages (not just the bridge) for the ROS2 and Gazebo version(s) supported by PX4.

:::: tabs

:::tab humble
To install the bridge for use with ROS 2 "Humble" and Gazebo Harmonic (on Ubuntu 22.04):

```sh
sudo apt install ros-humble-ros-gzharmonic
```

:::

:::tab foxy
First you will need to [install Gazebo Garden](../sim_gazebo_gz/index.md#installation-ubuntu-linux), as by default Foxy comes with Gazebo Classic 11. <!-- note, garden is EOL Nov 2024 -->

Then to install the interface packages for use with ROS 2 "Foxy" and Gazebo (Ubuntu 20.04):

```sh
sudo apt install ros-foxy-ros-gzgarden
```

:::

::::

:::info
The [repo](https://github.com/gazebosim/ros_gz#readme) and [package](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge#readme) READMEs show the package versions that need to be installed depending on your ROS2 and Gazebo versions.
:::

Once the packages are installed and sourced, the node `parameter_bridge` provides the bridging capabilities and can be used to create an unidirectional `/clock` bridge:

```sh
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

At this point, every ROS2 node must be instructed to use the newly bridged `/clock` topic as time source instead of the OS one, this is done by setting the parameter `use_sim_time` (of _each_ node) to `true` (see [ROS clock and Time design](https://design.ros2.org/articles/clock_and_time.html)).

This concludes the modifications required on the ROS2 side. On the PX4 side, you are only required to stop the uXRCE-DDS time synchronization, setting the parameter [UXRCE_DDS_SYNCT](../advanced_config/parameter_reference.md#UXRCE_DDS_SYNCT) to `false`.
By doing so, Gazebo will act as main and only time source for both ROS2 and PX4.

## ROS 2 Example Applications

### ROS 2 Listener

The ROS 2 [listener examples](https://github.com/PX4/px4_ros_com/tree/main/src/examples/listeners) in the [px4_ros_com](https://github.com/PX4/px4_ros_com) repo demonstrate how to write ROS nodes to listen to topics published by PX4.

Here we consider the [sensor_combined_listener.cpp](https://github.com/PX4/px4_ros_com/blob/main/src/examples/listeners/sensor_combined_listener.cpp) node under `px4_ros_com/src/examples/listeners`, which subscribes to the [SensorCombined](../msg_docs/SensorCombined.md) message.

:::info
[Build ROS 2 Workspace](#build-ros-2-workspace) shows how to build and run this example.
:::

The code first imports the C++ libraries needed to interface with the ROS 2 middleware and the header file for the `SensorCombined` message to which the node subscribes:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
```

Then it creates a `SensorCombinedListener` class that subclasses the generic `rclcpp::Node` base class.

```cpp
/**
 * @brief Sensor Combined uORB topic data callback
 */
class SensorCombinedListener : public rclcpp::Node
{
```

This creates a callback function for when the `SensorCombined` uORB messages are received (now as micro XRCE-DDS messages), and outputs the content of the message fields each time the message is received.

```cpp
public:
  explicit SensorCombinedListener() : Node("sensor_combined_listener")
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
    [this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
      std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
      std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;
      std::cout << "============================="   << std::endl;
      std::cout << "ts: "          << msg->timestamp    << std::endl;
      std::cout << "gyro_rad[0]: " << msg->gyro_rad[0]  << std::endl;
      std::cout << "gyro_rad[1]: " << msg->gyro_rad[1]  << std::endl;
      std::cout << "gyro_rad[2]: " << msg->gyro_rad[2]  << std::endl;
      std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
      std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative << std::endl;
      std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
      std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
      std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
      std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;
    });
  }
```

:::info
The subscription sets a QoS profile based on `rmw_qos_profile_sensor_data`.
This is needed because the default ROS 2 QoS profile for subscribers is incompatible with the PX4 profile for publishers.
For more information see: [ROS 2 Subscriber QoS Settings](#ros-2-subscriber-qos-settings),
:::

The lines below create a publisher to the `SensorCombined` uORB topic, which can be matched with one or more compatible ROS 2 subscribers to the `fmu/sensor_combined/out` ROS 2 topic.

````cpp
private:
 rclcpp::Subscription<px4_msgs::msg::DebugVect>::SharedPtr subscription_;
};
```s

The instantiation of the `SensorCombinedListener` class as a ROS node is done on the `main` function.

```cpp
int main(int argc, char *argv[])
{
  std::cout << "Starting sensor_combined listener node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<std::chrono::microseconds>());

  rclcpp::shutdown();
  return 0;
}
````

This particular example has an associated launch file at [launch/sensor_combined_listener.launch.py](https://github.com/PX4/px4_ros_com/blob/main/launch/sensor_combined_listener.launch.py).
This allows it to be launched using the [`ros2 launch`](#ros2-launch) command.

### ROS 2 Advertiser

A ROS 2 advertiser node publishes data into the DDS/RTPS network (and hence to the PX4 Autopilot).

Taking as an example the `debug_vect_advertiser.cpp` under `px4_ros_com/src/advertisers`, first we import required headers, including the `debug_vect` msg header.

```cpp
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/debug_vect.hpp>

using namespace std::chrono_literals;
```

Then the code creates a `DebugVectAdvertiser` class that subclasses the generic `rclcpp::Node` base class.

```cpp
class DebugVectAdvertiser : public rclcpp::Node
{
```

The code below creates a function for when messages are to be sent.
The messages are sent based on a timed callback, which sends two messages per second based on a timer.

```cpp
public:
  DebugVectAdvertiser() : Node("debug_vect_advertiser") {
    publisher_ = this->create_publisher<px4_msgs::msg::DebugVect>("fmu/debug_vect/in", 10);
    auto timer_callback =
    [this]()->void {
      auto debug_vect = px4_msgs::msg::DebugVect();
      debug_vect.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
      std::string name = "test";
      std::copy(name.begin(), name.end(), debug_vect.name.begin());
      debug_vect.x = 1.0;
      debug_vect.y = 2.0;
      debug_vect.z = 3.0;
      RCLCPP_INFO(this->get_logger(), "\033[97m Publishing debug_vect: time: %llu x: %f y: %f z: %f \033[0m",
                                    debug_vect.timestamp, debug_vect.x, debug_vect.y, debug_vect.z);
      this->publisher_->publish(debug_vect);
    };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::DebugVect>::SharedPtr publisher_;
};
```

The instantiation of the `DebugVectAdvertiser` class as a ROS node is done on the `main` function.

```cpp
int main(int argc, char *argv[])
{
  std::cout << "Starting debug_vect advertiser node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DebugVectAdvertiser>());

  rclcpp::shutdown();
  return 0;
}
```

### Offboard控制

[ROS 2 Offboard control example](../ros2/offboard_control.md) provides a complete C++ reference example of how to use [offboard control](../flight_modes/offboard.md) of PX4 with ROS2.

[Python ROS2 offboard examples with PX4](https://github.com/Jaeyoung-Lim/px4-offboard) (Jaeyoung-Lim/px4-offboard) provides a similar example for Python, and includes the scripts:

- `offboard_control.py`: Example of offboard position control using position setpoints
- `visualizer.py`: Used for visualizing vehicle states in Rviz

## Using Flight Controller Hardware

ROS 2 with PX4 running on a flight controller is almost the same as working with PX4 on the simulator.
The only difference is that you need to start both the agent _and the client_, with settings appropriate for the communication channel.

For more information see [Starting uXRCE-DDS](../middleware/uxrce_dds.md#starting-agent-and-client).

## Custom uORB Topics

ROS 2 needs to have the _same_ message definitions that were used to create the uXRCE-DDS client module in the PX4 Firmware in order to interpret the messages.
The definition are stored in the ROS 2 interface package [PX4/px4_msgs](https://github.com/PX4/px4_msgs) and they are automatically synchronized by CI on the `main` and release branches.
Note that all the messages from PX4 source code are present in the repository, but only those listed in `dds_topics.yaml` will be available as ROS 2 topics.
Therefore,

- If you're using a main or release version of PX4 you can get the message definitions by cloning the interface package [PX4/px4_msgs](https://github.com/PX4/px4_msgs) into your workspace.
- If you're creating or modifying uORB messages you must manually update the messages in your workspace from your PX4 source tree.
  Generally this means that you would update [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml), clone the interface package, and then manually synchronize it by copying the new/modified message definitions from [PX4-Autopilot/msg](https://github.com/PX4/PX4-Autopilot/tree/main/msg) to its `msg` folders.
  Assuming that PX4-Autopilot is in your home directory `~`, while `px4_msgs` is in `~/ros2_ws/src/`, then the command might be:

  ```sh
  rm ~/ros2_ws/src/px4_msgs/msg/*.msg
  cp ~/PX4-Autopilot/msg/*.msg ~/ros2_ws/src/px4_msgs/msg/
  ```

  ::: info
  Technically, [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml) completely defines the relationship between PX4 uORB topics and ROS 2 messages.
  For more information see [uXRCE-DDS > DDS Topics YAML](../middleware/uxrce_dds.md#dds-topics-yaml).

:::

## Customizing the Namespace

Custom topic and service namespaces can be applied at build time (changing [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml)) or at runtime (useful for multi vehicle operations):

- One possibility is to use the `-n` option when starting the [uxrce_dds_client](../modules/modules_system.md#uxrce-dds-client) from command line.
  This technique can be used both in simulation and real vehicles.
- A custom namespace can be provided for simulations (only) by setting the environment variable `PX4_UXRCE_DDS_NS` before starting the simulation.

:::info
Changing the namespace at runtime will append the desired namespace as a prefix to all `topic` fields in [dds_topics.yaml](../middleware/dds_topics.md) and all [service servers](#px4-ros-2-service-servers).
Therefore, commands like:

```sh
uxrce_dds_client start -n uav_1
```

或

```sh
PX4_UXRCE_DDS_NS=uav_1 make px4_sitl gz_x500
```

will generate topics under the namespaces:

```sh
/uav_1/fmu/in/  # for subscribers
/uav_1/fmu/out/ # for publishers
```

:::

## PX4 ROS 2 Service Servers

<Badge type="tip" text="PX4 v1.15" />

PX4 uXRCE-DDS middleware supports [ROS 2 services](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Services.html).
Services are remote procedure calls, from one node to another, that return a result.

A service server is the entity that will accept a remote procedure request, perform some computation on it, and return the result.
They simplify communication between ROS 2 nodes and PX4 by grouping the request and response behaviour, and ensuring that replies are only returned to the specific requesting user.
This is much easier that publishing the request, subscribing to the reply, and filtering out any unwanted responses.

The service servers that are built into the PX4 [uxrce_dds_client](../modules/modules_system.md#uxrce-dds-client) module include:

- `/fmu/vehicle_command` (definition: [`px4_msgs::srv::VehicleCommand`](https://github.com/PX4/px4_msgs/blob/main/srv/VehicleCommand.srv).)

  This service can be called by ROS 2 applications to send PX4 [VehicleCommand](../msg_docs/VehicleCommand.md) uORB messages and receive PX4 [VehicleCommandAck](../msg_docs/VehicleCommandAck.md) uORB messages in response.

All PX4 service names follow the convention `{extra_namespace}/fmu/{server_specific_name}` where `{extra_namespace}` is the same [custom namespace](#customizing-the-namespace) that can be given to the PX4 topics.

Details and specific examples are provided in the following sections.

### VehicleCommand service

This can be used to send commands to the vehicle, such as "take off", "land", change mode, and "orbit", and receive a response.

The service type is defined in [`px4_msgs::srv::VehicleCommand`](https://github.com/PX4/px4_msgs/blob/main/srv/VehicleCommand.srv) as:

```txt
VehicleCommand request
---
VehicleCommandAck reply
```

Users can make service requests by sending [VehicleCommand](../msg_docs/VehicleCommand.md) messages, and receive a [VehicleCommandAck](../msg_docs/VehicleCommandAck.md) message in response.
The service ensures that only the `VehicleCommandAck` reply generated for the specific request made by the user is sent back.

#### VehicleCommand Service Offboard Control Example

A complete _offboard control_ example using the VehicleCommand service is provided by the [offboard_control_srv](https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control_srv.cpp) node available in the `px4_ros_com` package.

The example closely follows the _offboard control_ example described in [ROS 2 Offboard Control Example](../ros2/offboard_control.md) but uses the `VehicleCommand` service to request mode changes, vehicle arming and vehicle disarming.

First the ROS 2 application declares a service client of type `px4_msgs::srv::VehicleCommand` using `rclcpp::Client()` as shown (this is the same approach used for all ROS2 service clients):

```cpp
rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
```

Then the client is initialized to the right ROS 2 service (`/fmu/vehicle_command`).
As the application assumes the standard PX4 namespace is used, the code to do this looks like this:

```cpp
vehicle_command_client_{this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command")}
```

After that, the client can be used to send any vehicle command request.
For example, the `arm()` function is used to request the vehicle to arm:

```cpp
void OffboardControl::arm()
{
  RCLCPP_INFO(this->get_logger(), "requesting arm");
  request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}
```

where `request_vehicle_command` handles formatting the request and sending it over in _asynchronous_ [mode](https://docs.ros.org/en/humble/How-To-Guides/Sync-Vs-Async.html#asynchronous-calls):

```cpp
void OffboardControl::request_vehicle_command(uint16_t command, float param1, float param2)
{
  auto request = std::make_shared<px4_msgs::srv::VehicleCommand>();

  VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  request->request = msg;

  service_done_ = false;
  auto result = vehicle_command_client_->async_send_request(request, std::bind(&OffboardControl::response_callback, this,
                           std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Command send");
}
```

The response is finally captured asynchronously by the `response_callback` method which checks for the request result:

```cpp
void OffboardControl::response_callback(
      rclcpp::Client<0>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      auto reply = future.get()->reply;
      service_result_ = reply.result;
      // make decision based on service_result_
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }
```

## ros2 CLI

The [ros2 CLI](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html) is a useful tool for working with ROS.
You can use it, for example, to quickly check whether topics are being published, and also inspect them in detail if you have `px4_msg` in the workspace.
The command also lets you launch more complex ROS systems via a launch file.
A few possibilities are demonstrated below.

### ros2 topic list

Use `ros2 topic list` to list the topics visible to ROS 2:

```sh
ros2 topic list
```

If PX4 is connected to the agent, the result will be a list of topic types:

```sh
/fmu/in/obstacle_distance
/fmu/in/offboard_control_mode
/fmu/in/onboard_computer_status
...
```

Note that the workspace does not need to build with `px4_msgs` for this to succeed; topic type information is part of the message payload.

### ros2 topic echo

Use `ros2 topic echo` to show the details of a particular topic.

Unlike with `ros2 topic list`, for this to work you must be in a workspace has built the `px4_msgs` and sourced `local_setup.bash` so that ROS can interpret the messages.

```sh
ros2 topic echo /fmu/out/vehicle_status
```

The command will echo the topic details as they update.

```sh
---
timestamp: 1675931593364359
armed_time: 0
takeoff_time: 0
arming_state: 1
latest_arming_reason: 0
latest_disarming_reason: 0
nav_state_timestamp: 3296000
nav_state_user_intention: 4
nav_state: 4
failure_detector_status: 0
hil_state: 0
...
---
```

### ros2 topic hz

You can get statistics about the rates of messages using `ros2 topic hz`.
For example, to get the rates for `SensorCombined`:

```sh
ros2 topic hz /fmu/out/sensor_combined
```

The output will look something like:

```sh
average rate: 248.187
min: 0.000s max: 0.012s std dev: 0.00147s window: 2724
average rate: 248.006
min: 0.000s max: 0.012s std dev: 0.00147s window: 2972
average rate: 247.330
min: 0.000s max: 0.012s std dev: 0.00148s window: 3212
average rate: 247.497
min: 0.000s max: 0.012s std dev: 0.00149s window: 3464
average rate: 247.458
min: 0.000s max: 0.012s std dev: 0.00149s window: 3712
average rate: 247.485
min: 0.000s max: 0.012s std dev: 0.00148s window: 3960
```

### ros2 launch

The `ros2 launch` command is used to start a ROS 2 launch file.
For example, above we used `ros2 launch px4_ros_com sensor_combined_listener.launch.py` to start the listener example.

You don't need to have a launch file, but they are very useful if you have a complex ROS 2 system that needs to start several components.

For information about launch files see [ROS 2 Tutorials > Creating launch files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)

## 故障处理

### Missing dependencies

The standard installation should include all the tools needed by ROS 2.

If any are missing, they can be added separately:

- **`colcon`** build tools should be in the development tools.
  It can be installed using:

  ```sh
  sudo apt install python3-colcon-common-extensions
  ```

- The Eigen3 library used by the transforms library should be in the both the desktop and base packages.
  It should be installed as shown:

  :::: tabs

  ::: tab humble

  ```sh
  sudo apt install ros-humble-eigen3-cmake-module
  ```


:::

  ::: tab foxy

  ```sh
  sudo apt install ros-foxy-eigen3-cmake-module
  ```


:::

  ::::

### ros_gz_bridge not publishing on the \clock topic

If your [ROS2 nodes use the Gazebo clock as time source](../ros2/user_guide.md#ros2-nodes-use-the-gazebo-clock-as-time-source) but the `ros_gz_bridge` node doesn't publish anything on the `/clock` topic, you may have the wrong version installed.
This might happen if you install ROS 2 Humble with the default "Ignition Fortress" packages, rather than using those for PX4, which uses "Gazebo Harmonic".

The following commands uninstall the default Ignition Fortress topics and install the correct bridge and other interface topics for **Gazebo Harmonic** with ROS2 **Humble**:

```bash
# Remove the wrong version (for Ignition Fortress)
sudo apt remove ros-humble-ros-gz

# Install the version for Gazebo Garden
sudo apt install ros-humble-ros-gzharmonic
```

## Additional information

- [ROS 2 in PX4: Technical Details of a Seamless Transition to XRCE-DDS](https://www.youtube.com/watch?v=F5oelooT67E) - Pablo Garrido & Nuno Marques (youtube)
- [DDS and ROS middleware implementations](https://github.com/ros2/ros2/wiki/DDS-and-ROS-middleware-implementations)
