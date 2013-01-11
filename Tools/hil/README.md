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

## GroundControl Interface
Note that the script defaults to opening a mavlink slave port on udp:14550, this is the default udp port for QGroundControl. If you start QGC, it should start communicating with runhil.py automatically.

# Notes

## TODO:

* Magnetometer measurement model doesn't depend on lat/lot yet.
* Add noise.
* Initialization routines for EKF.

## Source

This uses pymavlink and is based off of MAVProxy/ ardupilotemga SITL code.
