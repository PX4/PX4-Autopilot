# Multicopter Racer Setup

This page describes how to setup and configure a multicopter racer for optimal performance (in particular for [Acro mode](../flight_modes_mc/acro.md)).

Keep in mind that racers are fast vehicles, specifically designed to be overpowered!
You should already have some experience, or let someone with experience help you.

::: tip
Many things described here can also be applied to improve the flight performance of other types of multicopters.
:::

::: info
A racer usually omits some sensors (e.g. GPS).
As a result, fewer failsafe options are available.
:::

## Build Options

A racer usually omits some sensors.

The minimal configuration is to use only a gyro and accelerometer sensor.

::: info
If the board has an internal magnetometer, it should not be used (small racers are particularly prone to strong electromagnetic interference).
:::

Racers typically do not have a GPS as it adds some weight and is prone to damage during crashes (a GPS + external magnetometer must be placed on a GPS mast away from high currents to avoid magnetic interference, which unfortunately means that it is easy to break).

There are however some benefits in adding GPS, particularly for beginners:

- You can go into position hold and the vehicle will just stay in one place.
  This is handy if you lose the orientation or need a brake.
  It can also be used to land safely.
- [Return mode](../flight_modes_mc/return.md) can be used, either on a switch or as RC loss/low battery failsafe.
- You will have the last position when it crashes.
- The log contains the flight track, which means you can review the flight (in 3D).
  This can help to improve your acrobatic flight skills.

::: info
During aggressive acrobatic maneuvers the GPS can lose its position fix for a short time.
If you switch into [position mode](../flight_modes_mc/position.md) during that time, [altitude mode](../flight_modes_mc/altitude.md) will be used instead until the position becomes valid again.
:::

## Hardware Setup

The following paragraphs describe a few important points when building the vehicle.
If you need complete build instructions, you can follow the [QAV-R 5" KISS ESC Racer](../frames_multicopter/qav_r_5_kiss_esc_racer.md) build log.

### Vibration Setup

There are various mounting approaches to reduce vibrations.
For example, the flight controller can be mounted with vibration dampening foam, or using [O-rings](../frames_multicopter/qav_r_5_kiss_esc_racer.md#mounting).

While there is no single best method, you will typically have fewer problems with vibrations if you use high-quality components (frame, motors, props) as for example used in the [QAV-R 5" KISS ESC Racer](../frames_multicopter/qav_r_5_kiss_esc_racer.md).

Make sure to use **balanced props**.

### Center of Gravity

Make sure that the center of gravity is as close as possible to the center of thrust.
Left-right balance is usually not a problem, but front-back balance may be.
You can move the battery until it is correct and mark it on the frame so you will always place it correctly.

::: info
The integral term can account for an imbalanced setup, and a custom mixer can do that even better.
However it is best to fix any imbalance as part of the vehicle setup.
:::

## Software Setup

After having built the racer, you will need to configure the software.

Go through the [Basic Configuration Guide](../config/index.md).
In particular, set the [Airframe](../config/airframe.md) that most closely matches your frame (typically you will choose the [Generic 250 Racer](../airframes/airframe_reference.md#copter_quadrotor_x_generic_250_racer) airframe, which sets some racer-specific parameters by default).

These parameters are important:

- Enable One-Shot or DShot by selecting the protocol for a group of outputs during [Actuator Configuration](../config/actuators.md).
- Set the maximum roll-, pitch- and yaw rates for Stabilized mode as desired: [MC_ROLLRATE_MAX](../advanced_config/parameter_reference.md#MC_ROLLRATE_MAX), [MC_PITCHRATE_MAX](../advanced_config/parameter_reference.md#MC_PITCHRATE_MAX) and [MC_YAWRATE_MAX](../advanced_config/parameter_reference.md#MC_YAWRATE_MAX).
  The maximum tilt angle is configured with [MPC_MAN_TILT_MAX](../advanced_config/parameter_reference.md#MPC_MAN_TILT_MAX).
- The minimum thrust [MPC_MANTHR_MIN](../advanced_config/parameter_reference.md#MPC_MANTHR_MIN) should be set to 0.

### Estimator

If you use a GPS you can skip this section and use the default estimator.
Otherwise you should switch to the Q attitude estimator, which works without a magnetometer or barometer.

To enable it set [ATT_EN = 1](../advanced_config/parameter_reference.md#ATT_EN), [EKF2_EN =0 ](../advanced_config/parameter_reference.md#EKF2_EN) and [LPE_EN = 0](../advanced_config/parameter_reference.md#LPE_EN) (for more information see [Switching State Estimators](../advanced/switching_state_estimators.md#how-to-enable-different-estimators)).

Then change the following parameters:

- Set [SYS_HAS_MAG](../advanced_config/parameter_reference.md#SYS_HAS_MAG) to `0` if the system does not have a magnetometer.
- Set [SYS_HAS_BARO](../advanced_config/parameter_reference.md#SYS_HAS_BARO) to `0` if the system does not have a barometer.
- Configure the Q estimator: set [ATT_ACC_COMP](../advanced_config/parameter_reference.md#ATT_ACC_COMP) to `0`, [ATT_W_ACC](../advanced_config/parameter_reference.md#ATT_W_ACC) to 0.4 and [ATT_W_GYRO_BIAS](../advanced_config/parameter_reference.md#ATT_W_GYRO_BIAS) to 0.
  You can tune these later if you wish.

### Failsafe

Configure [RC loss and low battery failsafe](../config/safety.md).
If you do not use a GPS, set the failsafe to **Lockdown**, which turns off the motors.
Test RC loss on the bench without props attached by turning off the remote when the vehicle is armed.

Make sure to assign a [kill switch](../config/safety.md#kill-switch) or an [arming switch](../config/safety.md#arm-disarm-switch).
Test it and train to use it!

### PID Tuning

::: info
Make sure to calibrate the ESCs before doing any tuning.
:::

At this point you should be ready for a first test flight.

Assuming the vehicle is able to fly using the default settings, we then do a first pass of [Basic MC PID tuning](../config_mc/pid_tuning_guide_multicopter_basic.md).
The vehicle needs to be **undertuned** (the **P** and **D** gains should be set too low), such that there are no oscillations from the controller that could be interpreted as noise (the default gains might be good enough).
This is important for the [filter tuning](#filter-tuning) (there will be a second PID tuning round later).

### Control Latency

The _control latency_ is the delay from a physical disturbance of the vehicle until the motors react to the change.

:::tip
It is _crucial_ to reduce the control latency as much as possible!
A lower latency allows you to increase the rate **P** gains, which means better flight performance.
Even one millisecond added to the latency makes a difference.
:::

These are the factors that affect the latency:

- A soft airframe or soft vibration mounting increases latency (they act as a filter).
- [Low-pass filters](../config_mc/filter_tuning.md) in software and on the sensor chip trade off increased latency for improved noise filtering.
- PX4 software internals: the sensor signals need to be read in the driver and then pass through the controller to the output driver.
- The IO chip (MAIN pins) adds about 5.4 ms latency compared to using the AUX pins (this does not apply to a _Pixracer_ or _Omnibus F4_, but does apply to a Pixhawk).
  To avoid the IO delay attach the motors to the AUX pins instead.
- PWM output signal: enable [Dshot](../peripherals/dshot.md) by preference to reduce latency (or One-Shot if DShot is not supported).
  The protocol is selected for a group of outputs during [Actuator Configuration](../config/actuators.md).

### Filter Tuning

Filters trade off control latency and noise filtering, both of which impact performance.
For information see: [Filter/Control Latency Tuning](../config_mc/filter_tuning.md)

### PID Tuning (Second Round)

Now do a second round of PID tuning, this time as tight as possible, and also tuning the thrust curve.

:::tip
You can use the approach described in [Basic MC PID tuning](../config_mc/pid_tuning_guide_multicopter_basic.md) to tune the frame, but you will need to use the [Advanced Multicopter PID Tuning Guide (Advanced/Detailed)](../config_mc/pid_tuning_guide_multicopter.md#thrust-curve) to understand how to tune the thrust curve.

### Airmode

After you have verified that the vehicle flies well at low and high throttle, you can enable [airmode](../config_mc/pid_tuning_guide_multicopter.md#airmode) with the [MC_AIRMODE](../advanced_config/parameter_reference.md#MC_AIRMODE) parameter.
This feature makes sure that the vehicle is still controllable and tracks the rate at low throttle.
