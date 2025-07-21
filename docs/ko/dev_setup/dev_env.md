# 파일 및 코드 설치개발자 환경 설정 (툴체인)

The _supported platforms_ for PX4 development are:

- [Ubuntu Linux (22.04/20.04/18.04)](../dev_setup/dev_env_linux_ubuntu.md) — Recommended
- [Windows (10/11)](../dev_setup/dev_env_windows_wsl.md) — via WSL2
- [Mac OS](../dev_setup/dev_env_mac.md)

## 지원 대상

아래 표는 각 OS에서 구축 가능한 PX 대상을 보여줍니다.

| 대상                                                                                                                                                     | Linux (Ubuntu) | Mac | 윈도우 |
| ------------------------------------------------------------------------------------------------------------------------------------------------------ | :-------------------------------: | :-: | :-: |
| **NuttX based hardware:** [Pixhawk Series](../flight_controller/pixhawk_series.md), [Crazyflie](../complete_vehicles_mc/crazyflie2.md) |                 ✓                 |  ✓  |  ✓  |
| **Linux-based hardware:** [Raspberry Pi 2/3](../flight_controller/raspberry_pi_navio2.md)                                              |                 ✓                 |     |     |
| **Simulation:** [Gazebo SITL](../sim_gazebo_gz/index.md)                                                                               |                 ✓                 |  ✓  |  ✓  |
| **Simulation:** [Gazebo Classic SITL](../sim_gazebo_classic/index.md)                                                                  |                 ✓                 |  ✓  |  ✓  |
| **Simulation:** [ROS with Gazebo Classic](../simulation/ros_interface.md)                                                              |                 ✓                 |     |  ✓  |
| **Simulation:** ROS 2 with Gazebo                                                                                                      |                 ✓                 |     |  ✓  |

Experienced Docker users can also build with the containers used by our continuous integration system: [Docker Containers](../test_and_ci/docker.md)

## 다음 단계

위의 명령줄 도구 모음 중 하나를 설정하고, 다음 단계를 실행합니다.

- Install [VSCode](../dev_setup/vscode.md) (if you prefer using an IDE to the command line).
- Install the [QGroundControl Daily Build](../dev_setup/qgc_daily_build.md)
- Continue to [Building PX4 Software](../dev_setup/building_px4.md).
