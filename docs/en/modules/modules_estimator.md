# Modules Reference: Estimator

## AttitudeEstimatorQ

Source: [modules/attitude_estimator_q](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/attitude_estimator_q)

### Description

Attitude estimator q.

### Usage {#AttitudeEstimatorQ_usage}

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

### Usage {#airspeed_estimator_usage}

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

The documentation can be found on the [ECL/EKF Overview & Tuning](../advanced_config/tuning_the_ecl_ekf.md) page.

ekf2 can be started in replay mode (`-r`): in this mode, it does not access the system time, but only uses the
timestamps from the sensor topics.

### Usage {#ekf2_usage}

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

### Usage {#local_position_estimator_usage}

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

### Usage {#mc_hover_thrust_estimator_usage}

```
mc_hover_thrust_estimator <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## wind_estimator_3d

Source: [modules/wind_estimator_3d](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/wind_estimator_3d)

### Description

wind_estimator_3d is a 3D wind estimator for fixed-wing vehicles, based on a simple aerodynamic model of the
airframe. Unlike the existing 2D wind estimators (which only estimate horizontal wind), this model requires the
following airframe-specific parameters:

- `FW_W_MASS`: Vehicle mass
- `FW_W_AREA`: Vehicle reference (wing) area
- `FW_W_CY_B`: Sideslip force coefficient (side slip)
- `FW_W_CL_0`: Lift coefficient (zero angle of attack)
- `FW_W_CL_A`: Lift coefficient (angle of attack)

There is currently no automated identification/calibration pipeline in PX4 to derive these coefficients for a
given airframe. They must currently be obtained externally (for example from wind-tunnel testing, CFD, or manual
system identification) and entered by hand.

### Usage {#wind_estimator_3d_usage}

```
wind_estimator_3d <command> [arguments...]
 Commands:
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```
