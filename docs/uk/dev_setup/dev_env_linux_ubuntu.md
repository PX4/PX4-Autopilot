# Середовище розробки Ubuntu

The following instructions use a bash script to set up the PX4 development environment on the [Ubuntu Linux LTS](https://wiki.ubuntu.com/LTS) versions supported by PX4: Ubuntu 22.04 (Jammy Jellyfish), 20.04 (Focal Fossa), and 18.04 (Bionic Beaver).

The environment includes:

- [Gazebo Simulator](../sim_gazebo_gz/index.md) ("Harmonic") on Ubuntu 22.04
- [Gazebo Classic Simulator](../sim_gazebo_classic/index.md) on Ubuntu 20.04 and Ubuntu 18.04
- [Build toolchain for Pixhawk (and other NuttX-based hardware)](../dev_setup/building_px4.md#nuttx-pixhawk-based-boards).

:::info
The build toolchain for other flight controllers, simulators, and working with ROS are discussed in the [Other Targets](#other-targets) section below.
:::

:::tip
if you need to use Gazebo on Ubuntu 20.04 you can [manually install Gazebo "Garden"](../sim_gazebo_gz/index.md#installation-ubuntu-linux), with the caveat that this is end-of-life in November 2024.
If you want to use Gazebo Classic on Ubuntu 22.04 (say) then you can manually install it by following the instructions in [Gazebo Classic > Installation](../sim_gazebo_classic/index.md#installation).
:::

## Симуляція та NuttX (Pixhawk)

Use the [ubuntu.sh](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/setup/ubuntu.sh) script to set up a development environment that allows you to build for simulators and/or the [NuttX/Pixhawk](../dev_setup/building_px4.md#nuttx-pixhawk-based-boards) toolchain.

:::tip
The script is intended to be run on _clean_ Ubuntu LTS installations, and may not work if run "on top" of an existing system, or on a different Ubuntu release.
:::

Щоб встановити інструментарій:

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

  - При появі підказки по ходу виконання скрипту підтвердить вибір.
  - You can use the `--no-nuttx` and `--no-sim-tools` options to omit the NuttX and/or simulation tools.

3. Перезавантажте комп'ютер при завершенні.

:::details
Additional notes
These notes are provided "for information only":

- This setup is supported by the PX4 Dev Team.
  Інструкції також можуть працювати на інших системах заснованих на Debian Linux.

- You can verify the NuttX installation by confirming the `gcc` version as shown:

  ```sh
  $arm-none-eabi-gcc --version

  arm-none-eabi-gcc (GNU Arm Embedded Toolchain 9-2020-q2-update) 9.3.1 20200408 (release)
  Copyright (C) 2019 Free Software Foundation, Inc.
  This is free software; see the source for copying conditions.  There is NO
  warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  ```

- Вам все одно потрібен вихідний код PX4.
  But if you just wanted to set up the development environment without getting all the source code you could instead just download [ubuntu.sh](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/setup/ubuntu.sh) and [requirements.txt](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/setup/requirements.txt) and then run **ubuntu.sh**:

  ```sh
  wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/ubuntu.sh
  wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/requirements.txt
  bash ubuntu.sh
  ```


:::

## Відеоінструкція

This video shows how to install the toolchain for NuttX and simulation targets ([as covered below](#simulation-and-nuttx-pixhawk-targets)) along with the basic testing covered in [Building PX4 Software](../dev_setup/building_px4.md).

:::warning
The video suggests that you build source using JMAVSim, entering the command: `make px4_sitl jmavsim`.
As JMAVSim is now community-supported, you should instead build using Gazebo or Gazebo Classic, as shown in [Building the Code](../dev_setup/building_px4.md#first-build-using-a-simulator)
:::

<lite-youtube videoid="OtValQdAdrU" title=" Setting up your PX4 development environment on Linux"/>

## Other Targets

The Ubuntu development environment for ROS, other simulators, and other hardware targets, is covered in their respective documentation.
A subset of the relevant topics are linked below.

Raspberry Pi

- [Raspberry Pi 2/3 Navio2 Autopilot > PX4 Development Environment](../flight_controller/raspberry_pi_navio2.md#px4-development-environment)
- [Raspberry Pi 2/3/4 PilotPi Shield](../flight_controller/raspberry_pi_pilotpi.md).

ROS

- ROS 2: [ROS 2 User Guide > Installation & Setup](../ros2/user_guide.md#installation-setup).
- ROS (1): [ROS (1) Installation Guide](../ros/mavros_installation.md)

## Наступні кроки

Після того, як ви закінчите налаштування інструментів командного рядка:

- Install [VSCode](../dev_setup/vscode.md) (if you prefer using an IDE to the command line).

- Install the [QGroundControl Daily Build](../dev_setup/qgc_daily_build.md)

  :::tip
  The _daily build_ includes development tools that hidden in release builds.
  Вона також може надати доступ до нових функцій PX4, які ще не підтримуються в релізних збірках.

:::

- Continue to the [build instructions](../dev_setup/building_px4.md).
