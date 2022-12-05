# PX4 Drone Autopilot

This is the custom simulator for RAPTOR using PX4 gazebo SITL.

## Installation

install dependencies by running

```
sudo ./Tools/setup/ubuntu.sh
```

also install ROS2 humble. see: [https://docs.ros.org/en/humble/Installation.html)](https://docs.ros.org/en/humble/Installation.html)

## Run the Simulator

source the workspace

```
source /opt/ros/humble/setup.bash
```

run the simulator

```
make px4_sitl gazebo_raptor__baylands
```

the first (here raptor) is the model that is loaded and the second (here baylands) is the world.
