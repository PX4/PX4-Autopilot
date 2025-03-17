# Modules Reference: Estimator

## AttitudeEstimatorQ
Source: [modules/attitude_estimator_q](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/attitude_estimator_q)


### Description
Attitude estimator q.


<a id="AttitudeEstimatorQ_usage"></a>
### Usage
```
AttitudeEstimatorQ <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
## airspeed_estimator
Source: [modules/airspeed_selector](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/airspeed_selector)


### Description
This module provides a single airspeed_validated topic, containing indicated (IAS),
calibrated (CAS), true airspeed (TAS) and the information if the estimation currently
is invalid and if based sensor readings or on groundspeed minus windspeed.
Supporting the input of multiple "raw" airspeed inputs, this module automatically switches
to a valid sensor in case of failure detection. For failure detection as well as for
the estimation of a scale factor from IAS to CAS, it runs several wind estimators
and also publishes those.


<a id="airspeed_estimator_usage"></a>
### Usage
```
airspeed_estimator <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
## ekf2
Source: [modules/ekf2](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/ekf2)


### Description
Attitude and position estimator using an Extended Kalman Filter. It is used for Multirotors and Fixed-Wing.

The documentation can be found on the [ECL/EKF Overview & Tuning](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html) page.

ekf2 can be started in replay mode (`-r`): in this mode, it does not access the system time, but only uses the
timestamps from the sensor topics.


<a id="ekf2_usage"></a>
### Usage
```
ekf2 <command> [arguments...]
 Commands:
   start
     [-r]        Enable replay mode

   stop

   status        print status info
     [-v]        verbose (print all states and full covariance matrix)

   select_instance Request switch to new estimator instance
     <instance>  Specify desired estimator instance
```
## local_position_estimator
Source: [modules/local_position_estimator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/local_position_estimator)


### Description
Attitude and position estimator using an Extended Kalman Filter.


<a id="local_position_estimator_usage"></a>
### Usage
```
local_position_estimator <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
## mc_hover_thrust_estimator
Source: [modules/mc_hover_thrust_estimator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_hover_thrust_estimator)


### Description


<a id="mc_hover_thrust_estimator_usage"></a>
### Usage
```
mc_hover_thrust_estimator <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
