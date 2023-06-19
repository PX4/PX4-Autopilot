# PX4 Space Systems
This repository is a fork of [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) intended to add modules and functionality for space systems. Currently, this repository is a work in progress at KTH Space Robotics Lab. For more information, reach out the DISCOWER organization members.

## Setup for Air Carriages:
1. Make sure this repository is in the external/PX4 folder of your Astrobee source code
2. Run the following commands:
```
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh 
make px4_sitl gazebo-classic 
```
At this point, your simulation should run with the the default Iris quadcopter.

Then you can proceed with the Astrobee setup.