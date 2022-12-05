# PX4 Drone Autopilot

This is the custom simulator for RAPTOR using PX4 gazebo SITL.

## Installation

clone repository
```
git clone --recursive https://github.com/aurelappius/PX4-Autopilot
```

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


##Modify the model.

All parameters of the model can be modified in the file:

```
Tools/simulation/gazebo/sitl_
gazebo/models/raptor/raptor.sdf
```

##ROS2 Publishers/Subscribers

The current model publishes the following topics and message types:

/realsense_down_d
/realsense_down_ir1
/realsense_down_ir2
/realsense_down_rgb
/realsense_front_d
/realsense_front_ir1
/realsense_front_ir2
/realsense_front_rgb


```cpp
sensor_msgs::msg::Image realsense_down_rgb
sensor_msgs::msg::Image realsense_down_ir1
sensor_msgs::msg::Image realsense_down_ir2
sensor_msgs::msg::Image realsense_down_d

sensor_msgs::msg::Image realsense_front_rgb
sensor_msgs::msg::Image realsense_front_ir1
sensor_msgs::msg::Image realsense_front_ir2
sensor_msgs::msg::Image realsense_front_d
```

and subscribes to the following

```cpp
std_msgs::msg::Float64 leftGripper_deg
std_msgs::msg::Float64 rightGripper_deg
```
