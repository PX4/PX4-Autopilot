# ROS 2 用户指南

ROS 2-PX4 架构在ROS 2和PX4之间进行了深度整合。 允许 ROS 2 订阅或发布节点直接使用 PX4 uORB 话题。

本指南介绍了系统架构和应用程序流程，并解释了如何与PX4一起安装和使用ROS2。

:::info
从 PX4 v1.14, ROS 2 使用 [uXRCE-DDS](../middleware/uxrce_dds.md) 中间件替换版本 1 中使用的 _FastRTPS_ 中间件. 3 (v1.13不支持uXRCE-DDS)。

[migration guide](../middleware/uxrce_dds.md#fast-rtps-to-uxrce-dds-migration-guidelines) 解释您需要做什么来将ROS2 应用程序从 PX4 v1.13 迁移到 PX4 v1.14。

如果您仍然在 PX4 v1.13 上工作，请按照[PX4 v1.13 Docs](https://docs.px4.io/v1.13/en/ros/ros2_comm.html)中的说明操作。

<!-- remove this when there are PX4 v1.14 docs for some months -->

:::

## 综述

得益于 [uXRCE-DDS](../middleware/uxrce_dds.md) 通信中间件的使用，ROS 2 的应用流程非常简单直接。

![Architecture uXRCE-DDS with ROS 2](../../assets/middleware/xrce_dds/architecture_xrce-dds_ros2.svg)

<!-- doc source: https://docs.google.com/drawings/d/1WcJOU-EcVOZRPQwNzMEKJecShii2G4U3yhA3U6C4EhE/edit?usp=sharing -->

uXRCE-DDS 中间件由两部分组成：一部分是运行在 PX4 上的客户端，另一部分是运行在机载计算机上的代理；二者之间通过串口、UDP、TCP或自定义链路进行双向数据交换。
代理充当客户端的代理角色，以便在全局 DDS 数据空间中发布和订阅主题。

PX4 [uxrce_dds_client](../modules/modules_system.md#uxrce-dds-client) 是在构建时生成，并且默认包含在 PX4 固件中。
它包含“通用”XRCE-DDS客户端代码和它用来发布到来自uORB主题的 PX4 特定转换代码。
生成到客户端中的 uORB 消息子集在 [dds_topics.yaml](../middleware/dds_topics.md)中说明。
生成器使用源代码树中的 uORB 消息定义：[PX4-Autopilot/msg](https://github.com/PX4/PX4-Autopilot/tree/main/msg) 用于生成发送 ROS 2 消息的代码。

ROS 2 应用程序需要在一个工作空间中构建，该工作空间需包含与 PX4 固件中创建 uXRCE-DDS 客户端模块时所用完全相同的消息定义。
您可以通过克隆接口包[PX4/px4_msgs](https://github.com/PX4/px4_msgs)将这些内容纳入您的 ROS 2 工作空间(repo 中的范围与不同的 PX4 版本的消息相对应)。

从 PX4 v1.16 版本开始[message versioning](../middleware/uorb.md#message-versioning)，ROS 2 应用程序所使用的消息定义版本，可与构建 PX4 时所用的消息定义版本不同。
这需要[ROS 2 Message Translation Node](../ros2/px4_ros2_msg_translation_node.md)运行ROS 2 消息转换节点，以确保消息能够正确转换和交互。

需要注意的是，微型XRCE-DDS _agent_ 本身并不依赖客户端代码。
它可以从 [source](https://github.com/eProsima/Micro-XRCE-DDS-Agent) 中单独构建，或者作为ROS构建的一部分，或者作为snap包安装。

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
唯一依赖于ROS2的 PX4 是一组信息定义，它从[px4_msgs](https://github.com/PX4/px4_msgs)获取。
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
[ROS2 初学者教程](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay)建议您_打开一个新的终端来运行您的可执行文件。
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

<Badge type="tip" text="PX4 v1.16" /> <Badge type="warning" text="Experimental" />

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

除非在相关消息定义中明确指定，否则所有PX4 话题均采用 FRD（即 NED）坐标系约定。
因此，想要与 PX4 进行交互的 ROS 2 节点，必须妥善处理坐标系约定问题。

- 要将一个向量从ENU坐标系旋转到NED坐标系，必须执行两个基本旋转操作：
  - 首先是绕 Z 轴（朝上方向）旋转 π/2 弧度。
  - 然后是绕 X 轴（原东向 / 新北向）旋转 π 弧度

- 要将一个向量从NED坐标系旋转到ENU坐标系，必须执行两个基本旋转操作：

- - 首先是绕 Z 轴（朝下方向）旋转 π/2 弧度。
  - 然后是绕 X 轴（原北向 / 新东向）旋转 π 弧度。 需注意，这两种最终得到的操作在数学上是等效的

- 将向量从 FLU坐标系旋转到 FRD坐标系，仅需绕 X 轴（朝前方向）旋转 π 弧度即可。

- 将向量从 FRD坐标系旋转到 FLU坐标系，仅需绕 X 轴（朝前方向）旋转 π 弧度即可。

需要进行旋转处理的向量示例包括：

- [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md)消息中的所有字段；发送这些字段前，需先将其从 ENU坐标系转换为 NED坐标系。
- [VehicleThrustSetpoint](../msg_docs/VehicleThrustSetpoint.md)消息中的所有字段；发送这些字段前，需先将其从 FLU坐标系转换为 FRD坐标系。

与向量类似，用于表示飞行器（机体坐标系）相对于（w.r.t.）姿态的四元数也是如此。 （相对于）世界坐标系（的四元数）需要进行转换。

[PX4/px4_ros_com](https://github.com/PX4/px4_ros_com) 提供了名为  [frame_transforms](https://github.com/PX4/px4_ros_com/blob/main/include/px4_ros_com/frame_transforms.h)的共享库，可便捷地执行此类转换操作。

### ROS, Gazebo 和 PX4 时间同步

默认情况下，ROS 2 与 PX4 之间的时间同步由[uXRCE-DDS middleware](https://micro-xrce-dds.docs.eprosima.com/en/latest/time_sync.html) 自动管理；若需查看时间同步统计信息，可监听已桥接的话题 /fmu/out/timesync_status。
当 uXRCE-DDS 客户端运行在飞控器上，且代理运行在机载计算机上时，这便是理想的运行状态 —— 此时时间偏移、时间漂移以及通信延迟会被自动计算并补偿。

在 Gazebo 仿真中，GZBridge 会在每个仿真步长（sim step）为 PX4 设置时间[Change simulation speed](../sim_gazebo_gz/index.md#change-simulation-speed)。
需注意，这与 Gazebo Classic所采用的仿真锁步[simulation lockstep](../sim_gazebo_classic/index.md#lockstep)流程不同。

对于 ROS 2 用户而言，其节点的[time source](https://design.ros2.org/articles/clock_and_time.html)有两种选择。

#### ROS2 节点使用操作系统时钟作为时间源

本文档以及[offboard_control](./offboard_control.md)指南中所采用的便是此场景，同时，该场景也是 ROS 2 节点的标准行为
操作系统时钟作为时间来源，因此它只能在模拟实时系数非常接近时才能使用。
uXRCE-DDS 客户端的时间同步器随后会将 ROS 2 端的操作系统时钟（OS clock）与 PX4 端的 Gazebo 时钟进行桥接同步。
用户不需要进一步操作。

#### ROS2 节点使用 Gazebo 时钟作为时间源

在这种情况下，ROS2还使用Gazebo\`/时钟主题作为时间来源。
若 Gazebo 仿真的实时因子不为 1，或 ROS 2 需直接与 Gazebo 交互，则该方法具有合理性。
在 ROS 2 端，可通过[ros_gz](https://github.com/gazebosim/ros_gz)代码仓库中的[ros_gz_bridge](https://github.com/gazebosim/ros_gz) 功能包，实现与 Gazebo 的直接交互。

请使用以下命令，为 PX4 所支持的 ROS 2 和 Gazebo 版本安装正确的 ROS 2/gz 接口功能包（不仅限于桥接功能包）。

:::: tabs

:::tab humble
在 Ubuntu 22.04 系统上，若需安装用于搭配 ROS 2 “Humble”与 Gazebo Harmonic的桥接功能包，可执行以下操作：

```sh
sudo apt install ros-humble-ros-gzharmonic
```

:::

:::tab foxy
首先，您需要 [install Gazebo Garden](../sim_gazebo_gz/index.md#installation-ubuntu-linux)，因为默认情况下，Foxy预装的是 Gazebo Classic 11 <!-- note, garden is EOL Nov 2024 -->

接下来，若要在 Ubuntu 20.04 系统上安装用于搭配 ROS 2 "Foxy"与 Gazebo的桥接功能包，操作如下：

```sh
sudo apt install ros-foxy-ros-gzgarden
```

:::

::::

:::info
[repo](https://github.com/gazebosim/ros_gz#readme) 和 [package](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge#readme) README显示了需要安装的软件包版本，取决于您的 ROS2 和 Gazebo 版本。
:::

功能包安装并完成环境配置后，parameter_bridge节点会提供桥接能力，可用于创建一个单向的/clock桥接。

```sh
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

此时，必须指示每个 ROS 2 节点使用新桥接的/clock话题作为时间源，而非操作系统时钟（OS clock）；要实现这一点，需将（每个节点的）use_sim_time参数设置为true（详见[ROS clock and Time design](https://design.ros2.org/articles/clock_and_time.html)）。

至此，ROS 2 端所需的修改已全部完成。 在 PX4 端，你只需停止 uXRCE-DDS 时间同步功能，将参数[UXRCE_DDS_SYNCT](../advanced_config/parameter_reference.md#UXRCE_DDS_SYNCT)设置为false即可。
通过此操作，Gazebo 将成为 ROS 2 和 PX4 两者共同的、唯一的主时间源。

## ROS 2 示例应用程序

### ROS 2 Listener

[px4_ros_com](https://github.com/PX4/px4_ros_com中的 ROS 2  [listener examples](https://github.com/PX4/px4_ros_com/tree/main/src/examples/listeners)  repo展示了如何编写 ROS 节点，以监听由 PX4 发布的话题

此处我们以 px4_ros_com/src/examples/listeners 路径下的 [sensor_combined_listener.cpp](https://github.com/PX4/px4_ros_com/blob/main/src/examples/listeners/sensor_combined_listener.cpp) 节点为例，该节点会订阅 [SensorCombined](../msg_docs/SensorCombined.md) 消息。

:::info
[Build ROS 2 Workspace](#build-ros-2-workspace) 显示如何构建和运行这个例子。
:::

代码首先导入了与 ROS 2 中间件进行交互所需的 C++ 库，以及该节点所订阅的SensorCombined消息对应的头部文件：

```cpp
#include <0>
#include <1>
```

随后，代码创建了一个 SensorCombinedListener 类，该类继承自通用的 rclcpp::Node 基类。

```cpp
/**
 * @brief Sensor Combined uORB topic data callback
 */
class SensorCombinedListener : public rclcpp::Node
{
```

这会创建一个回调函数，用于处理SensorCombined uORB 消息（当前以微型 XRCE-DDS 消息格式传输）的接收事件；每当接收到该消息时，该函数会输出消息字段的内容

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
该订阅会基于 rmw_qos_profile_sensor_data 设置一个 QoS 配置文件。
之所以需要这样做，是因为 ROS 2 订阅者的默认 QoS（服务质量）配置文件，与 PX4 发布者的配置文件不兼容。
欲了解更多信息，请参阅：[ROS 2 Subscriber QoS Settings](#ros-2-subscriber-qos-settings),
:::

以下代码行创建了一个发布者，用于向 SensorCombined uORB 话题发布数据；该发布者可与一个或多个兼容的 ROS 2 订阅者匹配，这些订阅者监听的是 fmu/sensor_combined/out ROS 2 话题。

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

此特殊示例在[launch/sensor_combined_listener.launch.py](https://github.com/PX4/px4_ros_com/blob/main/launch/sensor_combined_listener.launch.py).有一个相关的启动文件。
这使得它可以通过 [`ros2 launch`](#ros2-launch)  命令启动

### ROS 2 发布者

一个 ROS 2 发布者节点会将数据发布到 DDS/RTPS 网络中（进而传递给 PX4 自动驾驶仪）。

以 px4_ros_com/src/advertisers 路径下的 debug_vect_advertiser.cpp（文件）为例，首先我们会导入所需的headers，其中包括 `debug_vect` msg header。

```cpp
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/debug_vect.hpp>

using namespace std::chrono_literals;
```

随后，代码创建了一个 DebugVectAdvertiser 类，该类继承自通用的 rclcpp::Node 基类。

```cpp
class DebugVectAdvertiser : public rclcpp::Node
{
```

这段代码创建了一个用来发送消息的回调函数。
发送消息的回调函数由定时器触发的，每秒钟发送两次消息。

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

这段代码在 main 函数中将 DebugVectAdvertiser 类实例化成一个ROS节点。

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

[ROS 2 Offboard control example](../ros2/offboard_control.md)提供了一个完整的 C++ 参考示例，说明如何使用 PX4 的  [offboard control](../flight_modes/offboard.md) 与 ROS 2。

[Python ROS2 offboard examples with PX4](https://github.com/Jaeyoung-Lim/px4-offboard) (Jaeyoung-Lim/px4-offboard) 为Python 提供了一个类似的示例，并包含脚本：

- `offboard_control.py`: 使用位置设定值进行离板位置控制的示例
- “visualizer.py\`：用于可视化载体状态的 Rviz

## 使用飞行控制器硬件

在飞行控制器上运行的 PX4 号ROS2与在模拟器上运行的 PX4 几乎相同。
唯一的区别是您需要同时启动agent  _and the client_，并设置适合通信频道。

更多信息详见[Starting uXRCE-DDS](../middleware/uxrce_dds.md#starting-agent-and-client)

## 自定义 uORB 主题

ROS 2需要有用于在 PX4 固件中创建 uXRCE-DDS客户端模块的 _sam_message 定义，以便解释消息。
这些定义存储在 ROS 2 接口包[PX4/px4_msgs](https://github.com/PX4/px4_msgs)中，并且会通过CI在 main（主）分支和发布分支上自动同步。
需要注意的是，PX4 源代码中的所有消息均存在于该代码仓库中，但只有在dds_topics.yaml文件中列出的消息，才会作为 ROS 2 话题可用。
因此

- 如果您正在使用 PX4 的主要版本或发布版本，您可以通过克隆接口包[PX4/px4_msgs](https://github.com/PX4/px4_msgs)获得消息定义。
- 如果您要创建或修改 uORB 消息，必须从 PX4 源代码树中手动更新工作空间中的消息。
  一般来说，这意味着您将更新 [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml)，克隆接口包。 然后手动同步，将新的/修改的消息定义从 [PX4-Autopilot/msg](https://github.com/PX4/PX4-Autopilot/tree/main/msg)复制到它的 `msg` 文件夹。
  假定PX4-Autopilot在你的主目录`~`中，而`px4_msgs`则在`~/ros2_ws/src/`中，命令可能是：

  ```sh
  rm ~/ros2_ws/src/px4_msgs/msg/*.msg
  cp ~/PX4-Autopilot/msg/*.msg ~/ros2_ws/src/px4_msgs/msg/
  ```

  ::: info
  从技术角度而言，[dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml) 这个文件完整定义了 PX4 uORB 话题与 ROS 2 消息之间的对应关系。
  欲了解更多信息，请参阅[uXRCE-DDS > DDS Topics YAML](../middleware/uxrce_dds.md#dds-topics-yaml)。

:::

## Customizing the Namespace

自定义主题和服务命名空间可以在构建时间(更改 [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml))或运行时间(对多载体操作有用)：

- 一种可能性是在从命令行启动[uxrce_dds_client](../modules/modules_system.md#uxrce-dds-client)时使用 "-n" 选项。
  这种技术既可用于模拟，也可用于实际机体。
- 在开始模拟前，可以通过设置环境变量 `PX4_UXRCE_DDS_NS`来提供自定义命名空间 (仅限)

:::info
更改运行时的命名空间将会将所需的命名空间作为一个前缀附加到 [dds_topics.yaml](../middleware/dds_topics.md) 中所有的 "topic " 字段和所有 [service servers](#px4-ros-2-service-servers)。
因此，命令如下：

```sh
uxrce_dds_client start -n uav_1
```

或

```sh
PX4_UXRCE_DDS_NS=uav_1 make px4_sitl gz_x500
```

将在以下命名空间下生成话题：

```sh
/uav_1/fmu/in/  # for subscribers
/uav_1/fmu/out/ # for publishers
```

:::

## PX4 ROS 2 Service Servers

<Badge type="tip" text="PX4 v1.15" />

PX4 uXRCE-DDS middleware supports [ROS 2 services](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Services.html).
服务（Services）是一种远程过程调用（remote procedure calls），由一个节点发起，向另一个节点请求调用，最终会返回一个结果。

A service server is the entity that will accept a remote procedure request, perform some computation on it, and return the result.
They simplify communication between ROS 2 nodes and PX4 by grouping the request and response behaviour, and ensuring that replies are only returned to the specific requesting user.
这比发布请求、订阅回复并过滤掉所有不需要的响应要容易得多。

构建在 PX4 [uxrce_dds_client](../modules/modules_system.md#uxrce-dds-client) 模块中的服务服务器包括：

- `/fmu/vehicle_command` (definition: [`px4_msgs::srv::VehicleCommand`](https://github.com/PX4/px4_msgs/blob/main/srv/VehicleCommand.srv).)

  此服务可以被 ROS 2 应用程序调用来发送 PX4[VehicleCommand](../msg_docs/VehicleCommand.md) uORB 消息，并相应接收 PX4  [VehicleCommandAck](../msg_docs/VehicleCommandAck.md)uORB 消息。

所有 PX4 服务名称均遵循 `{extra_namespace}/fmu/{server_specific_name}` 这一约定，其中 {extra_namespace} 与可分配给 PX4 话题的 [custom namespace](#customizing-the-namespace)相同。

具体细节和示例将在以下章节中提供。

### 载体指挥服务

这可用于向飞行器发送指令（例如 “起飞”“着陆”“切换模式” 和 “盘旋”），并接收响应。

服务类型在 [`px4_msgs::srv::VehicleCommand`](https://github.com/PX4/px4_msgs/blob/main/srv/VehicleCommand.srv)  中定义如下：

```txt
VehicleCommand request
---
VehicleCommandAck reply
```

用户可通过发送  [VehicleCommand](../msg_docs/VehicleCommand.md)消息发起服务请求，并会收到一条[VehicleCommandAck](../msg_docs/VehicleCommandAck.md) 消息作为响应。
该服务可确保仅将针对用户发起的特定请求所生成的 VehicleCommandAck回复返回。

#### 载体指挥服务离板控制示例

在 px4_ros_com 功能包中，有一个[offboard_control_srv](https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control_srv.cpp) 节点，该节点提供了一个完整的、使用 VehicleCommand 服务实现离板控制的示例。

该示例与[ROS 2 Offboard Control Example](../ros2/offboard_control.md) 中描述的离板控制示例高度相似，但使用 VehicleCommand 服务来请求模式切换、飞行器上锁和飞行器解锁。

首先，ROS 2 应用程序会使用 rclcpp::Client() 声明一个类型为 px4_msgs::srv::VehicleCommand 的服务客户端，具体如下（所有 ROS 2 服务客户端均采用此方法）

```cpp
rclcpp::Client<0>::SharedPtr vehicle_command_client_;
```

然后客户端初始化到正确的 ROS 2 服务 (`/fmu/vehicle_command` )。
当应用程序假设使用标准的 PX4 命名空间时，这样做的代码看起来就像这样：

```cpp
vehicle_command_client_{this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command")}
```

此后，客户可以用来发送任何机体命令请求。
例如，`arm()`函数用于请求机体放置：

```cpp
void OffboardControl::arm()
{
  RCLCPP_INFO(this->get_logger(), "requesting arm");
  request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}
```

`request_vehicle_command`处理请求格式化并在_asynchronous_ [mode](https://docs.ros.org/en/humble/How-To-Guides/Sync-Vs-Async.html#asynchronous-calls):

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

最终，响应由 response_callback 方法以异步方式捕获，该方法会检查请求结果：

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

[ros2 CLI](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)是一个有用的工具来处理ROS。
例如，您可以使用它快速检查话题是否正在发布；如果您的工作空间中包含 px4_msg，还可以详细查看这些话题的内容。
该命令还允许您通过启动文件（launch file）启动更复杂的 ROS 系统。
下文显示了几种可能性。

### ros2 topic list（ROS 2 话题列表命令）

使用 ros2 topic list 命令列出 ROS 2 可识别的话题：

```sh
ros2 topic list（ROS 2 话题列表命令）
```

若 PX4 已连接至代理，输出结果将是一份话题类型列表：

```sh
/fmu/in/obstacle_distance
/fmu/in/offboard_control_mode
/fmu/in/onboard_computer_status
...
```

请注意，工作区不需要使用 px4_msgs 构建才能成功；主题类型信息是消息有效载荷的一部分。

### ros2 topic echo

使用  `ros2 topic echo`"来显示特定主题的详细信息。

与 ros2 topic list 命令不同，要让该功能正常工作，你必须处于一个已编译 px4_msgs且已执行 local_setup.bash 脚本的工作空间中，这样 ROS 才能解析相关消息

```sh
ros2 topic echo /fmu/out/vehicle_status
```

该命令将在主题细节更新时响应它们的详细信息。

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

你可以使用 ros2 topic hz 命令获取消息速率相关的统计信息。
例如，获取`SensorCombined`速率：

```sh
ros2 topic hz /fmu/out/sensor_combined
```

输出会看起来像这样：

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

ros2 launch 命令用于启动一个 ROS 2 启动文件
例如，前面我们使用 ros2 launch px4_ros_com sensor_combined_listener.launch.py 命令启动了监听器示例。

你并非必须使用启动文件，但如果你的 ROS 2 系统较为复杂，需要启动多个组件，那么启动文件会非常实用。

关于启动文件的信息，请参阅 [ROS 2 Tutorials > Creating launch files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)

## 故障处理

### 缺少依赖项

标准安装应包含 ROS 2 所需的所有工具。

如果有任何缺失，可以单独添加：

- **`colcon`** 构建工具应该在开发工具中。
  可以使用以下方式安装它：

  ```sh
  sudo apt install python3-colcon-common-extensions
  ```

- 变换库（transforms library）所使用的 Eigen3 库，应同时存在于桌面版（desktop）功能包和基础版（base）功能包中。
  它应该安装在显示中：

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

如果你的[ROS2 nodes use the Gazebo clock as time source](../ros2/user_guide.md#ros2-nodes-use-the-gazebo-clock-as-time-source) 但`ros_gz_bridge` 节点没有发布任何关于\`/时钟' 主题的内容。 您可能安装了错误的版本。
若你在安装 ROS 2 Humble 时，使用的是默认的 “Ignition Fortress” 功能包，而非 PX4 所使用的、适配 “Gazebo Harmonic” 的功能包，就可能出现这种情况。

以下命令会卸载默认的 Ignition Fortress 功能包，并为搭配 ROS 2 Humble 版本的 Gazebo Harmonic 安装正确的桥接功能包及其他接口功能包：

```bash
# Remove the wrong version (for Ignition Fortress)
sudo apt remove ros-humble-ros-gz

# Install the version for Gazebo Garden
sudo apt install ros-humble-ros-gzharmonic
```

## 更多信息

- [ROS 2 in PX4: Technical Details of a Seamless Transition to XRCE-DDS](https://www.youtube.com/watch?v=F5oelooT67E) - Pablo Garrido & Nuno Marques (youtube)
- [DDS and ROS middleware implementations](https://github.com/ros2/ros2/wiki/DDS-and-ROS-middleware-implementations)
