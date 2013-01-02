# Description

This branch currently has a switch to go between 
sensors HIL and state HIL. This is located in apps/mavlink/mavlink.c.
Currently the HIL mode is set to HIL_MODE_SENSORS.

# Usage

During the boot process you must type this or put it in your startup script:
```
kalman_demo start
control_demo start
```
## QGroundControl

The udp packets used by the python script called px4Start.sh are the same as the FlightGear xml descriptions in qgroundcontrol for the fixed wing.

Currently I have not modified qgroundcontrol to have a new JSBSim mode, but you can disable the FlightGear process start and use this temporarily.

# TODO:

* Magnetometer measurement model doesn't depend on lat/lot yet.
* Add noise.
* Initialization routines for EKF.

# Source

This is currently a modification of the code for ardupilotmega SITL written by Andrew Tridgell.
