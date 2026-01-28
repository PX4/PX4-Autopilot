# Multicopter PID Tuning Guide (Manual/Basic)

This tutorial explains how to _manually_ tune the PID loops on PX4 for all [multicopter setups](../airframes/airframe_reference.md#copter) (Quads, Hexa, Octo etc).

:::tip
[Autotune](../config/autotune_mc.md) is recommended for most users, as it is far faster, easier and provides good tuning for most frames.
Manual tuning is recommended for frames where autotuning does not work, or where fine-tuning is essential.
:::

Generally if you're using an appropriate [supported frame configuration](../airframes/airframe_reference.md#copter), the default tuning should allow you to fly the vehicle safely.
Tuning is recommended for all new vehicle setups to get the _very best_ performance, because relatively small hardware and assembly changes can affect the gains required tuning gains for optimal flight.
For example, different ESCs or motors change the optimal tuning gains.

## Introduction

PX4 uses **P**roportional, **I**ntegral, **D**erivative (PID) controllers (these are the most widespread control technique).

The _QGroundControl_ **PID Tuning** setup provides real-time plots of the vehicle setpoint and response curves.
The goal of tuning is to set the P/I/D values such that the _Response_ curve matches the _Setpoint_ curve as closely as possible (i.e. a fast response without overshoots).

![QGC Rate Controller Tuning UI](../../assets/mc_pid_tuning/qgc_mc_pid_tuning_rate_controller.png)

The controllers are layered, which means a higher-level controller passes its results to a lower-level controller.
The lowest-level controller is the **rate controller**, followed by the **attitude controller**, and finally the **velocity & position controller**.
The PID tuning needs to be done in this same order, starting with the rate controller, as it will affect all other controllers.

The testing procedure for each controller (rate, attitude, velocity/position) and axis (yaw, roll, pitch) is always the same: create a fast setpoint change by moving the sticks very rapidly and observe the response.
Then adjust the sliders (as discussed below) to improve the tracking of the response to the setpoint.

:::tip

- Rate controller tuning is the most important, and if tuned well, the other controllers often need no or only minor adjustments
- Usually the same tuning gains can be used for roll and pitch.
- use Acro/Stabilized/Altitude mode to tune the rate controller
- Use [Position mode](../flight_modes_mc/position.md) to tune the _Velocity Controller_ and the _Position Controller_.
  Make sure to switch to the _Simple position control_ mode so you can generate step inputs.
  ![QGC PID tuning: Simple control selector](../../assets/mc_pid_tuning/qgc_mc_pid_tuning_simple_control.png)
  :::

## Preconditions

- You have selected the closest matching [default frame configuration](../config/airframe.md) for your vehicle.
  This should give you a vehicle that already flies.
- You should have done an [ESC calibration](../advanced_config/esc_calibration.md).
- If using PWM outputs their minimum values should be set correctly in the [Actuator Configuration](../config/actuators.md).
  These need to be set low, but such that the **motors never stop** when the vehicle is armed.

  This can be tested in [Acro mode](../flight_modes_mc/acro.md) or in [Stabilized mode](../flight_modes_mc/manual_stabilized.md):

  - Remove propellers
  - Arm the vehicle and lower the throttle to the minimum
  - Tilt the vehicle to all directions, about 60 degrees
  - Check that no motors turn off

- Use a high-rate telemetry link such as WiFi if at all possible (a typical low-range telemetry radio is not fast enough for real-time feedback and plots).
  This is particularly important for the rate controller.
- Disable [MC_AIRMODE](../advanced_config/parameter_reference.md#MC_AIRMODE) before tuning a vehicle (there is an options for this in the PID tuning screen).

:::warning
Poorly tuned vehicles are likely to be unstable, and easy to crash.
Make sure to have assigned a [Kill switch](../config/safety.md#emergency-switches).
:::

## Tuning Procedure

The tuning procedure is:

1. Arm the vehicle, takeoff, and hover (typically in [Position mode](../flight_modes_mc/position.md)).
1. Open _QGroundControl_ **Vehicle Setup > PID Tuning**
   ![QGC Rate Controller Tuning UI](../../assets/mc_pid_tuning/qgc_mc_pid_tuning_rate_controller.png)
1. Select the **Rate Controller** tab.
1. Confirm that the airmode selector is set to **Disabled**
1. Set the _Thrust curve_ value to: 0.3 (PWM, power-based controllers) or 1 (RPM-based ESCs)

   ::: info
   For PWM, power-based and (some) UAVCAN speed controllers, the control signal to thrust relationship may not be linear.
   As a result, the optimal tuning at hover thrust may not be ideal when the vehicle is operating at higher thrust.

   The thrust curve value can be used to compensate for this non-linearity:

   - For PWM controllers, 0.3 is a good default (which may benefit from [further tuning](../config_mc/pid_tuning_guide_multicopter.md#thrust-curve)).
   - For RPM-based controllers, use 1 (no further tuning is required as these have a quadratic thrust curve).

   For more information see the [detailed PID tuning guide](../config_mc/pid_tuning_guide_multicopter.md#thrust-curve).
   :::

1. Set the _Select Tuning_ radio button to: **Roll**.
1. (Optionally) Select the **Automatic Flight Mode Switching** checkbox.
   This will _automatically_ switch from [Position mode](../flight_modes_mc/position.md) to [Stabilised mode](../flight_modes_mc/manual_stabilized.md) when you press the **Start** button
1. For rate controller tuning switch to _Acro mode_, _Stabilized mode_ or _Altitude mode_ (unless automatic switching is enabled).
1. Select the **Start** button in order to start tracking the setpoint and response curves.
1. Rapidly move the _roll stick_ full range and observe the step response on the plots.
   :::tip
   Stop tracking to enable easier inspection of the plots.
   This happens automatically when you zoom/pan.
   Use the **Start** button to restart the plots, and **Clear** to reset them.
   :::
1. Modify the three PID values using the sliders (for roll rate-tuning these affect `MC_ROLLRATE_K`, `MC_ROLLRATE_I`, `MC_ROLLRATE_D`) and observe the step response again.
   The values are saved to the vehicle as soon as the sliders are moved.
   ::: info
   The goal is for the _Response_ curve to match the _Setpoint_ curve as closely as possible (i.e. a fast response without overshoots).
   :::
   The PID values can be adjusted as follows:
   - P (proportional) or K gain:
     - increase this for more responsiveness
     - reduce if the response is overshooting and/or oscillating (up to a certain point increasing the D gain also helps).
   - D (derivative) gain:
     - this can be increased to dampen overshoots and oscillations
     - increase this only as much as needed, as it amplifies noise (and can lead to hot motors)
   - I (integral) gain:
     - used to reduce steady-state error
     - if too low, the response might never reach the setpoint (e.g. in wind)
     - if too high, slow oscillations can occur
1. Repeat the tuning process above for the pitch and yaw:
   - Use _Select Tuning_ radio button to select the axis to tune
   - Move the appropriate sticks (i.e. pitch stick for pitch, yaw stick for yaw).
   - For pitch tuning, start with the same values as for roll.
     :::tip
     Use the **Save to Clipboard** and **Reset from Clipboard** buttons to copy the roll settings for initial pitch settings.
     :::
1. Repeat the tuning process for the attitude controller on all the axes.
1. Repeat the tuning process for the velocity and positions controllers (on all the axes).

   - Use Position mode when tuning these controllers
   - Select the **Simple position control** option in the _Position control mode ..._ selector (this allows direct control for the generation of step inputs)

     ![QGC PID tuning: Simple control selector](../../assets/mc_pid_tuning/qgc_mc_pid_tuning_simple_control.png)

All done!
Remember to re-enable airmode before leaving the setup.
