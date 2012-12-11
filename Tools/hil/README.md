# Description

This branch currently has a switch to go between 
sensors HIL and state HIL. This is located in apps/mavlink/mavlink.c.
Currently the HIL mode is set to HIL_MODE_SENSORS.

# Usage

During the boot process you must type this or put it in your startup script:
```
attiute_estimator_ekf start
position_estimator start &
control_demo start
```
## QGroundControl

The udp packets used by the python script called px4Start.sh are the same as the FlightGear xml descriptions in qgroundcontrol for the fixed wing.

Currently I have not modified qgroundcontrol to have a new JSBSim mode, but you can disable the FlightGear process start and use this temporarily.

# TODO:

* Magnetometer measurement model from attitude needs to be improved.
* Add noise.

# BUGS:

* jsbsim/runsim.py not starting with JSBSim in air.
* Controller not stable with ekf navigator.

# Source

This is currently a modification of the code for ardupilotmega SITL written by Andrew Tridgell.
