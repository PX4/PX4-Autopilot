# ROS (1) with MAVROS Installation Guide

:::warning
The PX4 development team recommend that all users [upgrade to ROS 2](../ros2/index.md).
이 문서는 "이전 접근 방식"을 설명합니다.
:::

This documentation explains how to set up communication between the PX4 Autopilot and a ROS 1 enabled companion computer using MAVROS.

[MAVROS](http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.sys_status) is a ROS 1 package that enables MAVLink extendable communication between computers running ROS 1 for any MAVLink enabled autopilot, ground station, or peripheral.
_MAVROS_ is the "official" supported bridge between ROS 1 and the MAVLink protocol.

First we install PX4 and ROS, and then MAVROS.

## Install ROS and PX4

This section explains how to install [ROS 1](../ros/index.md) with PX4.
ROS 1 full desktop builds come with Gazebo Classic, so normally you will not install the simulator dependencies yourself!

:::tip
These instructions are a simplified version of the [official installation guide](https://github.com/mavlink/mavros/tree/master/mavros#installation).
They cover the _ROS Melodic and Noetic_ releases.
:::

:::: tabs

:::tab ROS Noetic (Ubuntu 20.04)

If you're working with [ROS Noetic](http://wiki.ros.org/noetic) on Ubuntu 20.04:

1. Install PX4 without the simulator toolchain:

   1. [Download PX4 Source Code](../dev_setup/building_px4.md):

      ```sh
      git clone https://github.com/PX4/PX4-Autopilot.git --recursive
      ```

   2. Run the **ubuntu.sh** the `--no-sim-tools` (and optionally `--no-nuttx`):

      ```sh
      bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
      ```

      - 스크립트가 진행되는 동안 모든 프롬프트를 확인합니다.

   3. 완료되면 컴퓨터를 재부팅합니다.

2. You _may_ need to install the following additional dependencies:

   ```sh
   sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
   ```

3. Follow the [Noetic Installation instructions](http://wiki.ros.org/noetic/Installation/Ubuntu#Installation) (ros-noetic-desktop-full is recommended).

:::

:::tab ROS Melodic (Ubuntu 18.04)

If you're working with ROS "Melodic on Ubuntu 18.04:

1. Download the [ubuntu_sim_ros_melodic.sh](https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh) script in a bash shell:

   ```sh
   wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh
   ```

2. Run the script:

   ```sh
   bash ubuntu_sim_ros_melodic.sh
   ```

   You may need to acknowledge some prompts as the script progresses.

   ::: tip
   You don't need to install MAVROS (as shown below), as this is included by the script

   Also note:

   - ROS Melodic is installed with Gazebo (Classic) 9 by default.
   - Your catkin (ROS build system) workspace is created at **~/catkin_ws/**.
   - The script uses instructions from the ROS Wiki "Melodic" [Ubuntu page](http://wiki.ros.org/melodic/Installation/Ubuntu).

:::

::::

## Install MAVROS

Then MAVROS can be installed either from source or binary.
개발자는 소스로 설치하는 것이 좋습니다.

#### 바이너리 설치(Debian/Ubuntu)

The ROS repository has binary packages for Ubuntu x86, amd64 (x86_64) and armhf (ARMv7).
Kinetic은 Debian Jessie amd64 및 arm64(ARMv8)도 지원합니다.

Use `apt-get` for installation, where `${ROS_DISTRO}` below should resolve to `kinetic` or `noetic`, depending on your version of ROS:

```sh
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
```

Then install [GeographicLib](https://geographiclib.sourceforge.io/) datasets by running the `install_geographiclib_datasets.sh` script:

```sh
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

#### 소스 설치

This installation assumes you have a catkin workspace located at `~/catkin_ws` If you don't create one with:

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src
```

You will be using the ROS Python tools: _wstool_ (for retrieving sources), _rosinstall_, and _catkin_tools_ (building) for this installation. 다음 명령어를 사용하여 ROS를 설치할 수 있습니다.

```sh
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
```

:::tip
While the package can be built using **catkin_make** the preferred method is using **catkin_tools** as it is a more versatile and "friendly" build tool.
:::

wstool을 처음 사용하는 경우 다음을 사용하여 소스 공간을 초기화합니다.

```sh
$ wstool init ~/catkin_ws/src
```

Now you are ready to do the build:

1. MAVLink를 설치합니다.

   ```sh
   # We use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
   rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
   ```

2. 릴리스 또는 최신 버전을 사용하여 소스에서 MAVROS를 설치합니다.

   - 출시/안정

      ```sh
      rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
      ```

   - 최신 소스

      ```sh
      rosinstall_generator --upstream-development mavros | tee -a /tmp/mavros.rosinstall
      ```

      ```sh
      # For fetching all the dependencies into your catkin_ws,
      # just add '--deps' to the above scripts, E.g.:
      #   rosinstall_generator --upstream mavros --deps | tee -a /tmp/mavros.rosinstall
      ```

3. Create workspace & deps

   ```sh
   wstool merge -t src /tmp/mavros.rosinstall
   wstool update -t src -j4
   rosdep install --from-paths src --ignore-src -y
   ```

4. Install [GeographicLib](https://geographiclib.sourceforge.io/) datasets:

   ```sh
   ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
   ```

5. 소스를 빌드합니다.

   ```sh
   catkin build
   ```

6. 작업 공간에서 setup.bash 또는 setup.zsh를 사용하는지 확인하십시오.

   ```sh
   #Needed or rosrun can't find nodes from this workspace.
   source devel/setup.bash
   ```

In the case of error, there are addition installation and troubleshooting notes in the [mavros repo](https://github.com/mavlink/mavros/tree/master/mavros#installation).

## MAVROS 예제

The [MAVROS Offboard Example (C++)](../ros/mavros_offboard_cpp.md), will show you the basics of MAVROS, from reading telemetry, checking the drone state, changing flight modes and controlling the drone.

:::info
If you have an example app using the PX4 Autopilot and MAVROS, we can help you get it on our docs.
:::

## See Also

- [mavros ROS Package Summary](http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.sys_status)
- [mavros source](https://github.com/mavlink/mavros/)
- [ROS Melodic installation instructions](http://wiki.ros.org/melodic/Installation)
