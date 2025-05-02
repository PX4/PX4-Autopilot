# 우분투 개발 환경

The following instructions use a bash script to set up the PX4 development environment on the [Ubuntu Linux LTS](https://wiki.ubuntu.com/LTS) versions supported by PX4: Ubuntu 24.04 (Nimble Numbat) and Ubuntu 22.04 (Jammy Jellyfish).

The environment includes:

- [Gazebo Simulator](../sim_gazebo_gz/index.md) ("Harmonic")
- [Build toolchain for Pixhawk (and other NuttX-based hardware)](../dev_setup/building_px4.md#nuttx-pixhawk-based-boards).

On Ubuntu 22.04:

- [Gazebo Classic Simulator](../sim_gazebo_classic/index.md) can be used instead of Gazebo.
  Gazebo is nearing feature-parity with Gazebo-Classic on PX4, and will soon replace it for all use cases.

The build toolchain for other flight controllers, simulators, and working with ROS are discussed in the [Other Targets](#other-targets) section below.

:::details
Can I use an older version of Ubuntu?
PX4 supports the current and last Ubuntu LTS release where possible.
Older releases are not supported (so you can't raise defects against them), but may still work.
For example, Gazebo Classic setup is included in our standard build instructions for macOS, Ubuntu 18.04 and 20.04, and Windows on WSL2 for the same hosts.
:::

## Simulation and NuttX (Pixhawk) Targets

Use the [ubuntu.sh](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/setup/ubuntu.sh) script to set up a development environment that allows you to build for simulators and/or the [NuttX/Pixhawk](../dev_setup/building_px4.md#nuttx-pixhawk-based-boards) toolchain.

:::tip
The script is intended to be run on _clean_ Ubuntu LTS installations, and may not work if run "on top" of an existing system, or on a different Ubuntu release.
:::

툴체인을 설치하려면:

1. [Download PX4 Source Code](../dev_setup/building_px4.md):

  ```sh
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive
  ```

  ::: info
  The environment setup scripts in the source usually work for recent PX4 releases.
  If working with an older version of PX4 you may need to [get the source code specific to your release](../contribute/git_examples.md#get-a-specific-release).

:::

2. Run the **ubuntu.sh** with no arguments (in a bash shell) to install everything:

  ```sh
  bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
  ```

  - 스크립트가 진행되는 동안 모든 프롬프트를 확인합니다.
  - You can use the `--no-nuttx` and `--no-sim-tools` options to omit the NuttX and/or simulation tools.

3. If you need Gazebo Classic (Ubuntu 22.04 only) then you can manually remove Gazebo and install it by following the instructions in [Gazebo Classic > Installation](../sim_gazebo_classic/index.md#installation).

4. 완료되면 컴퓨터를 재부팅합니다.

:::details
Additional notes
These notes are provided "for information only":

- This setup is supported by the PX4 Dev Team.
  The instructions may also work on other Debian Linux based systems.

- You can verify the NuttX installation by confirming the `gcc` version as shown:

  ```sh
  $arm-none-eabi-gcc --version

  arm-none-eabi-gcc (15:13.2.rel1-2) 13.2.1 20231009
  Copyright (C) 2023 Free Software Foundation, Inc.
  This is free software; see the source for copying conditions.  There is NO
  warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  ```

- 어쨌든 PX4 소스 코드가 필요합니다.
  But if you just wanted to set up the development environment without getting all the source code you could instead just download [ubuntu.sh](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/setup/ubuntu.sh) and [requirements.txt](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/setup/requirements.txt) and then run **ubuntu.sh**:

  ```sh
  wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/ubuntu.sh
  wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/requirements.txt
  bash ubuntu.sh
  ```


:::

## Other Targets

The Ubuntu development environment for ROS, other simulators, and other hardware targets, is covered in their respective documentation.
A subset of the relevant topics are linked below.

라즈베리파이

- [Raspberry Pi 2/3 Navio2 Autopilot > PX4 Development Environment](../flight_controller/raspberry_pi_navio2.md#px4-development-environment)
- [Raspberry Pi 2/3/4 PilotPi Shield](../flight_controller/raspberry_pi_pilotpi.md).

ROS

- ROS 2: [ROS 2 User Guide > Installation & Setup](../ros2/user_guide.md#installation-setup).
- ROS (1): [ROS (1) Installation Guide](../ros/mavros_installation.md)

## 다음 단계

명령줄 도구 모음 설정후, 다음을 수행합니다.

- Install [VSCode](../dev_setup/vscode.md) (if you prefer using an IDE to the command line).

- Install the [QGroundControl Daily Build](../dev_setup/qgc_daily_build.md)

  :::tip
  The _daily build_ includes development tools that hidden in release builds.
  또한, 릴리스 빌드에서 아직 지원되지 않는 새로운 PX4 기능에 대한 액세스를 제공할 수도 있습니다.

:::

- Continue to the [build instructions](../dev_setup/building_px4.md).
