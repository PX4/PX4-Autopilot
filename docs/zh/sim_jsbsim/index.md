# JSBSim 仿真

:::warning
This simulator is [community supported and maintained](../simulation/community_supported_simulators.md).
It may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

[JSBSim](https://jsbsim.sourceforge.net/index.html)是一款开源飞行仿真器（“飞行动力学模型”，FDM），可运行于 Microsoft Windows、Apple Macintosh、Linux、IRIX、Cygwin（Windows 上的 Unix 环境）等平台。
其功能包括完全可配置的空气动力学和推进系统，可对飞行器的复杂飞行动力学进行建模。
动力学模型还考虑了地球自转效应。

支持的飞行器：固定翼飞机、四旋翼飞行器、六旋翼飞行器

<lite-youtube videoid="y5azVNmIVyw" title="JSBSim with APX4 Software-In-The-Loop Simulation"/>

:::info
See [Simulation](../simulation/index.md) for general information about simulators, the simulation environment, and simulation configuration (e.g. supported vehicles).
:::

## Installation (Ubuntu Linux)

:::info
These instructions were tested on Ubuntu 18.04
:::

1. Install the usual [Development Environment on Ubuntu LTS / Debian Linux](../dev_setup/dev_env_linux_ubuntu.md).

2. 从 发布页面 (https://github.com/JSBSim-Team/jsbsim/releases/tag/Linux)安装JSBSim 发行版：

   ```sh
   dpkg -i JSBSim-devel_1.1.0.dev1-<release-number>.bionic.amd64.deb
   ```

3. （可选）可使用 FlightGear 进行可视化。
   如需安装 FlightGear，请参阅 FlightGear 安装说明(../sim_flightgear/index.md))。

## Running the Simulation

如下所示，可以通过 make 命令便捷地运行 JSBSim SITL 仿真：

```sh
cd /path/to/PX4-Autopilot
make px4_sitl jsbsim
```

这将同时运行 PX4 SITL 实例和 FlightGear 用户界面（用于可视化）。
如果希望在不启动 FlightGear 用户界面的情况下运行，可在 make 命令前添加 HEADLESS=1。

The supported vehicles and `make` commands are listed below (click on the links to see the vehicle images).

| Vehicle        | 通信                                 |
| -------------- | ---------------------------------- |
| Standard Plane | `make px4_sitl jsbsim_rascal`      |
| Quadrotor      | `make px4_sitl jsbsim_quadrotor_x` |
| 六旋翼飞行器         | `make px4_sitl jsbsim_hexarotor_x` |

The commands above launch a single vehicle with the full UI.
_QGroundControl_ should be able to automatically connect to the simulated vehicle.

## 使用 ROS 运行 JSBSim

要使用 ROS 运行 JSBSim：

1. 将 px4-jsbsim-bridge 软件包克隆到 catkin 工作空间中：

   ```sh
   cd <path_to_catkin_ws>/src
   git clone https://github.com/Auterion/px4-jsbsim-bridge.git
   ```

2. 构建 jsbsim_bridge catkin 软件包：

   ```sh
   catkin build jsbsim_bridge
   ```

   ::: 信息
   必须已在工作空间中配置好 MAVROS（若尚未配置，请按照 MAVROS 安装指南操作）(../ros/mavros_installation.md)。

:::

3. 然后，使用如下 launch 文件通过 ROS 启动 JSBSim：

   ```sh
   roslaunched jsbsim_bridge px4_jsbsim_bridge.launch
   ```

## 更多信息

- [px4-jsbsim-bridge readme](https://github.com/Auterion/px4-jsbsim-bridge)
