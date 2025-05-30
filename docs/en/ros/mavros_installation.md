# ROS (1) with MAVROS Installation Guide

:::warning
The PX4 development team recommend that all users [upgrade to ROS 2](../ros2/index.md).
This documentation reflects the "old approach".
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

::: tab ROS Noetic (Ubuntu 20.04)

If you're working with [ROS Noetic](http://wiki.ros.org/noetic) on Ubuntu 20.04:

1. Install PX4 without the simulator toolchain:

   1. [Download PX4 Source Code](../dev_setup/building_px4.md):

      ```sh
      git clone https://github.com/PX4/PX4-Autopilot.git --recursive
      ```

   1. Run the **ubuntu.sh** the `--no-sim-tools` (and optionally `--no-nuttx`):

      ```sh
      bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
      ```

      - Acknowledge any prompts as the script progress.

   1. Restart the computer on completion.

1. You _may_ need to install the following additional dependencies:

   ```sh
   sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
   ```

1. Follow the [Noetic Installation instructions](http://wiki.ros.org/noetic/Installation/Ubuntu#Installation) (ros-noetic-desktop-full is recommended).

:::

::: tab ROS Melodic (Ubuntu 18.04)

If you're working with ROS "Melodic on Ubuntu 18.04:

1. Download the [ubuntu_sim_ros_melodic.sh](https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh) script in a bash shell:

   ```sh
   wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh
   ```

1. Run the script:

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
We recommend that developers use the source installation.

#### Binary Installation (Debian / Ubuntu)

The ROS repository has binary packages for Ubuntu x86, amd64 (x86_64) and armhf (ARMv7).
Kinetic also supports Debian Jessie amd64 and arm64 (ARMv8).

Use `apt-get` for installation, where `${ROS_DISTRO}` below should resolve to `kinetic` or `noetic`, depending on your version of ROS:

```sh
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
```

Then install [GeographicLib](https://geographiclib.sourceforge.io/) datasets by running the `install_geographiclib_datasets.sh` script:

```sh
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

#### Source Installation

This installation assumes you have a catkin workspace located at `~/catkin_ws` If you don't create one with:

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src
```

You will be using the ROS Python tools: _wstool_ (for retrieving sources), _rosinstall_, and _catkin_tools_ (building) for this installation. While they may have been installed during your installation of ROS you can also install them with:

```sh
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
```

:::tip
While the package can be built using **catkin_make** the preferred method is using **catkin_tools** as it is a more versatile and "friendly" build tool.
:::

If this is your first time using wstool you will need to initialize your source space with:

```sh
$ wstool init ~/catkin_ws/src
```

Now you are ready to do the build:

1. Install MAVLink:

   ```sh
   # We use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
   rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
   ```

1. Install MAVROS from source using either released or latest version:

   - Released/stable

     ```sh
     rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
     ```

   - Latest source

     ```sh
     rosinstall_generator --upstream-development mavros | tee -a /tmp/mavros.rosinstall
     ```

     ```sh
     # For fetching all the dependencies into your catkin_ws,
     # just add '--deps' to the above scripts, E.g.:
     #   rosinstall_generator --upstream mavros --deps | tee -a /tmp/mavros.rosinstall
     ```

1. Create workspace & deps

   ```sh
   wstool merge -t src /tmp/mavros.rosinstall
   wstool update -t src -j4
   rosdep install --from-paths src --ignore-src -y
   ```

1. Install [GeographicLib](https://geographiclib.sourceforge.io/) datasets:

   ```sh
   ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
   ```

1. Build source

   ```sh
   catkin build
   ```

1. Make sure that you use setup.bash or setup.zsh from workspace.

   ```sh
   #Needed or rosrun can't find nodes from this workspace.
   source devel/setup.bash
   ```

In the case of error, there are addition installation and troubleshooting notes in the [mavros repo](https://github.com/mavlink/mavros/tree/master/mavros#installation).

## MAVROS Examples

The [MAVROS Offboard Example (C++)](../ros/mavros_offboard_cpp.md), will show you the basics of MAVROS, from reading telemetry, checking the drone state, changing flight modes and controlling the drone.

::: info
If you have an example app using the PX4 Autopilot and MAVROS, we can help you get it on our docs.
:::

## See Also

- [mavros ROS Package Summary](http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.sys_status)
- [mavros source](https://github.com/mavlink/mavros/)
- [ROS Melodic installation instructions](http://wiki.ros.org/melodic/Installation)
