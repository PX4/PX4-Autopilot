# 安装文件和代码

The _supported platforms_ for PX4 development are:

- [Ubuntu Linux (22.04/20.04/18.04)](../dev_setup/dev_env_linux_ubuntu.md) — Recommended
- [Windows (10/11)](../dev_setup/dev_env_windows_wsl.md) — via WSL2
- [Mac OS](../dev_setup/dev_env_mac.md)

## 支持的编译目标

下表显示了您可以在每个操作系统上构建何种 PX平台的固件编译。

| 平台                                                                                                                                                     | Linux (Ubuntu) |             Mac             |           Windows           |
| ------------------------------------------------------------------------------------------------------------------------------------------------------ | :-------------------------------: | :-------------------------: | :-------------------------: |
| **NuttX based hardware:** [Pixhawk Series](../flight_controller/pixhawk_series.md), [Crazyflie](../complete_vehicles_mc/crazyflie2.md) |    &check;    | &check; | &check; |
| **Linux-based hardware:** [Raspberry Pi 2/3](../flight_controller/raspberry_pi_navio2.md)                                              |    &check;    |                             |                             |
| **Simulation:** [Gazebo SITL](../sim_gazebo_gz/index.md)                                                                               |    &check;    | &check; | &check; |
| **Simulation:** [Gazebo Classic SITL](../sim_gazebo_classic/index.md)                                                                  |    &check;    | &check; | &check; |
| **Simulation:** [ROS with Gazebo Classic](../simulation/ros_interface.md)                                                              |    &check;    |                             | &check; |
| **Simulation:** ROS 2 with Gazebo                                                                                                      |    &check;    |                             | &check; |

Experienced Docker users can also build with the containers used by our continuous integration system: [Docker Containers](../test_and_ci/docker.md)

## Gazebo dependencies

如果你对 Docker 比较熟悉的话你也可以使用预先构建好的容器作为开发环境：<a href="../test_and_ci/docker.md">Docker 容器</a>。

- Install [VSCode](../dev_setup/vscode.md) (if you prefer using an IDE to the command line).
- Install the [QGroundControl Daily Build](../dev_setup/qgc_daily_build.md)
- Continue to [Building PX4 Software](../dev_setup/building_px4.md).
