# JSBSim Simulation

:::warning
This simulator is [community supported and maintained](../simulation/community_supported_simulators.md).
It may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

[JSBSim](http://jsbsim.sourceforge.net/index.html) is a open source flight simulator ("flight dynamics model (FDM)") that runs on Microsoft Windows, Apple Macintosh, Linux, IRIX, Cygwin (Unix on Windows), etc.
Its features include: fully configurable aerodynamics and a propulsion system that can model complex flight dynamics of an aircraft.
Rotational earth effects are also modeled into the dynamics.

**Supported Vehicles:** Plane, Quadrotor, Hexarotor

<lite-youtube videoid="y5azVNmIVyw" title="JSBSim with APX4 Software-In-The-Loop Simulation"/>

:::info
See [Simulation](../simulation/index.md) for general information about simulators, the simulation environment, and simulation configuration (e.g. supported vehicles).
:::

## Installation (Ubuntu Linux)

:::info
These instructions were tested on Ubuntu 18.04
:::

1. Install the usual [Development Environment on Ubuntu LTS / Debian Linux](../dev_setup/dev_env_linux_ubuntu.md).

2. Install a JSBSim release from the [release page](https://github.com/JSBSim-Team/jsbsim/releases/tag/Linux):

   ```sh
   dpkg -i JSBSim-devel_1.1.0.dev1-<release-number>.bionic.amd64.deb
   ```

3. (Optional) FlightGear may (optionally) be used for visualisation.
   To install FlightGear, refer to the [FlightGear installation instructions](../sim_flightgear/index.md)).

## Running the Simulation

JSBSim SITL simulation can be conveniently run through a `make` command as shown below:

```sh
cd /path/to/PX4-Autopilot
make px4_sitl jsbsim
```

This will run both the PX4 SITL instance and the FlightGear UI (for visualization).
If you want to run without the FlightGear UI, you can add `HEADLESS=1` to the front of the `make` command.

The supported vehicles and `make` commands are listed below (click on the links to see the vehicle images).

| Vehicle   | 통신                                 |
| --------- | ---------------------------------- |
| 표준 항공기    | `make px4_sitl jsbsim_rascal`      |
| Quadrotor | `make px4_sitl jsbsim_quadrotor_x` |
| Hexarotor | `make px4_sitl jsbsim_hexarotor_x` |

The commands above launch a single vehicle with the full UI.
_QGroundControl_ should be able to automatically connect to the simulated vehicle.

## Running JSBSim with ROS

To run JSBSim with ROS:

1. Clone the `px4-jsbsim-bridge` package into your catkin workspace:

   ```sh
   cd <path_to_catkin_ws>/src
   git clone https://github.com/Auterion/px4-jsbsim-bridge.git
   ```

2. Build the `jsbsim_bridge` catkin package:

   ```sh
   catkin build jsbsim_bridge
   ```

   ::: info
   You must have already set MAVROS in your workspace (if not, follow the instructions in the [MAVROS installation guide](../ros/mavros_installation.md)).

:::

3. So start JSBSim through ROS using the launch file as shown:

   ```sh
   roslaunch jsbsim_bridge px4_jsbsim_bridge.launch
   ```

## 추가 정보

- [px4-jsbsim-bridge readme](https://github.com/Auterion/px4-jsbsim-bridge)
