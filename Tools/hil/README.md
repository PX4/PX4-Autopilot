# Usage

## NSH Startup Script

You must use the startup script found in data/rc. The two main lines added to normal startup are:
```
kalman_demo start
control_demo start
```

## Python Script

This python script runhil.py is used for conducting HIL. Both sensor-level and state-level HIL are supported. You can view the runhil.py usage with:
```
runhil.py -h
```

A call to runhil.py might look like this:
```
./runhil.py --waypoints data/sf_waypoints.txt --master /dev/ttyUSB1 --gcs localhost:14550 --mode sensor
```

This says:
* load the give waypoints
* connect to the px4 autopilot on usb port 1
* setup external ground station communication of localhost:14550 udp.
* mode sensor says do sensor-level hardware-in-the-loop (HIL), this can be set to state as well

HILmodes
* State-level HIL tests the control and guidance systems.
* Sensor-level HIL tests the navigation system in addition to the control, and guidance systems. This requires a lot of data to be sent to the vehicle and a high baudrate.
* It is no longer necessary to change the firmware to change between state and sensor level HIL.

## GroundControl Interface
Note that the script defaults to opening a mavlink slave port on udp:14550, this is the default udp port for QGroundControl. If you start QGC, it should start communicating with runhil.py automatically.

# Notes

## TODO:

* Magnetometer measurement model doesn't depend on lat/lot yet.
* Add noise.
* Initialization routines for EKF.

## Source

This uses pymavlink and is based off of MAVProxy/ ardupilotemga SITL code.
