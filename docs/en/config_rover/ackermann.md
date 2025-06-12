# Configuration/Tuning (Ackermann Rover)

This topic provides a step-by-step guide for setting up your [Ackermann rover](../frames_rover/ackermann.md).

Successive steps enable [drive modes](../flight_modes_rover/ackermann.md) with more autopilot support and features.

::: warning
Each step is dependent on the previous steps having been completed.
Modes will only work properly if the preceding modes have been configured.
:::

## Basic Setup

To configure the Ackermann rover frame and outputs:

1. Enable Rover support by flashing the [PX4 rover build](../frames_rover/index.md#flashing-the-rover-build) onto your flight controller.
   Note that this is a special build that contains rover-specific modules.

2. In the [Airframe](../config/airframe.md) configuration select the _Generic Rover Ackermann_:

   ![QGC screenshot showing selection of the airframe 'Generic ackermann rover'](../../assets/config/airframe/airframe_generic_rover_ackermann.png)

   Select the **Apply and Restart** button.

   ::: info
   If this airframe is not displayed and you have checked that you are using rover firmware (not the default), you can alternatively enable this frame by setting the [SYS_AUTOSTART](../advanced_config/parameter_reference.md#SYS_AUTOSTART) parameter to `51000`.
   :::

3. Open the [Actuators Configuration & Testing](../config/actuators.md) to map the steering and throttle functions to flight controller outputs.

## Manual Mode

::: warning
For this mode to work properly the [Basic Setup](#basic-setup) must've already been completed!
:::

The basic setup already covers the minimum setup required to use the rover in [Manual mode](../flight_modes_rover/ackermann.md#manual-mode).

This mode is also affected by (optional) acceleration/deceleration limits.
As configuration of these limits becomes mandatory for subsequent modes, we do this setup here.

Navigate to [Parameters](../advanced_config/parameters.md) in QGroundControl and set the following parameters:

1. [RA_WHEEL_BASE](#RA_WHEEL_BASE) [m]: Measure the distance from the back to the front wheels.
2. [RA_MAX_STR_ANG](#RA_MAX_STR_ANG) [deg]: Measure the maximum steering angle.

   ![Geometric parameters](../../assets/airframes/rover/rover_ackermann/geometric_parameters.png)

3. [RO_MAX_THR_SPEED](#RO_MAX_THR_SPEED) [m/s]: Drive the rover at full throttle and set this parameter to the observed value of the ground speed.

   :::info
   This parameter is also used for the feed-forward term of the speed control.
   It will be further tuned in the configuration of [Position mode](#position-mode).
   :::

4. (Optional) [RO_ACCEL_LIM](#RO_ACCEL_LIM) [m/s^2]: Maximum acceleration you want to allow for your rover.

   <a id="RO_ACCEL_LIM_CONSIDERATIONS"></a>

   :::tip
   Your rover has a maximum possible acceleration which is determined by the maximum torque the motor can supply.
   This may or may not be appropriate for your vehicle and use case.

   One approach to determine an appropriate value is:

   1. From a standstill, give the rover full throttle until it reaches the maximum speed.
   1. Disarm the rover and plot the `measured_speed_body_x` from [RoverVelocityStatus](../msg_docs/RoverVelocityStatus.md).
   1. Divide the maximum speed by the time it took to reach it and set this as the value for [RO_ACCEL_LIM](#RO_ACCEL_LIM).

   Some RC rovers have enough torque to lift up if the maximum acceleration is not limited.
   If that is the case:

   1. Set [RO_ACCEL_LIM](#RO_ACCEL_LIM) to a low value, give the rover full throttle from a standstill and observe its behaviour.
   1. Increase [RO_ACCEL_LIM](#RO_ACCEL_LIM) until the rover starts to lift up during the acceleration.
   1. Set [RO_ACCEL_LIM](#RO_ACCEL_LIM) to the highest value that does not cause the rover to lift up.
      :::

5. (Optional) [RO_DECEL_LIM](#RO_DECEL_LIM) [m/s^2]: Maximum deceleration you want to allow for your rover.

   :::tip
   The same [considerations](#RO_ACCEL_LIM_CONSIDERATIONS) as in the configuration of [RO_ACCEL_LIM](#RO_ACCEL_LIM) apply.
   :::

   :::info
   This parameter is also used for the calculation of the speed setpoint during [Auto modes](#auto-modes).
   :::

6. (Optional) [RA_STR_RATE_LIM](#RA_STR_RATE_LIM) [deg/s]: Maximum steering rate you want to allow for your rover.

   :::tip
   This value depends on your rover and use case.
   For bigger rovers there might be a mechanical limit that is easy to identify by steering the rover at a standstill and increasing
   [RA_STR_RATE_LIM](#RA_STR_RATE_LIM) until you observe the steering rate to no longer be limited by the parameter.
   For smaller rovers you might observe the steering to be too aggressive. Set [RA_STR_RATE_LIM](#RA_STR_RATE_LIM) to a low value and steer the rover at a standstill.
   Increase the parameter until you reach the maximum steering rate you are comfortable with.
   :::

   :::warning
   A low maximum steering rate makes the rover worse at tracking steering setpoints, which can lead to a poor performance in the subsequent modes.
   :::

## Acro Mode

::: warning
For this mode to work properly [Manual mode](#acro-mode) must've already been configured!
:::

To set up [Acro mode](../flight_modes_rover/ackermann.md#acro-mode) configure the following [parameters](../advanced_config/parameters.md) in QGroundControl:

1. [RO_YAW_RATE_LIM](#RO_YAW_RATE_LIM): Maximum yaw rate you want to allow for your rover.

   :::tip
   Limiting the yaw rate is necessary if the rover is prone rolling over, loosing traction at high speeds or if passenger comfort is important.
   Small rovers especially can be prone to rolling over when steering aggressively at high speeds.

   If this is the case:

   1. In [Acro mode](../flight_modes_rover/ackermann.md#acro-mode), set [RO_YAW_RATE_LIM](#RO_YAW_RATE_LIM) to a small value and drive the rover at full throttle and with the right stick all the way to the left or right.
   1. Increase [RO_YAW_RATE_LIM](#RO_YAW_RATE_LIM) until the wheels of the rover start to lift up.
   1. Set [RO_YAW_RATE_LIM](#RO_YAW_RATE_LIM) to the highest value that does not cause the rover to lift up.

   If you see no need to limit the yaw rate, set this parameter to the maximum yaw rate the rover can achieve:

   1. In [Manual mode](#manual-mode) drive the rover at full throttle and with the maximum steering angle.
   2. Plot the `measured_yaw_rate` from [RoverRateStatus](../msg_docs/RoverRateStatus.md) and enter the highest observed value for [RO_YAW_RATE_LIM](#RO_YAW_RATE_LIM).

   :::

2. [RO_YAW_RATE_P](#RO_YAW_RATE_P) [-]: Proportional gain of the closed loop yaw rate controller.
   The closed loop acceleration control will compare the yaw rate setpoint with the measured yaw rate and adapt the motor commands based on the error between them.
   The proportional gain is multiplied with this error and that value is added to the motor command.
   This compensates for disturbances such as uneven ground and external forces.

   :::tip
   To tune this parameter:

   1. Put the rover in [Acro mode](../flight_modes_rover/ackermann.md#acro-mode) and hold the throttle stick and the right stick at a few different levels for a couple of seconds each.
   2. Disarm the rover and from the flight log plot the `adjusted_yaw_rate_setpoint` from [RoverRateStatus](../msg_docs/RoverRateStatus.md) and the `measured_yaw_rate` from [RoverRateStatus](../msg_docs/RoverRateStatus.md) over each other.
   3. Increase [RO_YAW_RATE_P](#RO_YAW_RATE_P) if the measured value does not track the setpoint fast enough or decrease it if the measurement overshoots the setpoint by too much.
   4. Repeat until you are satisfied with the behaviour.
      :::

3. [RO_YAW_RATE_I](#RO_YAW_RATE_I) [-]: Integral gain of the closed loop yaw rate controller.
   The integral gain accumulates the error between the desired and actual yaw rate scaled by the integral gain over time and that value is added to the motor command.

   ::: tip
   An integrator might not be necessary at this stage, but it will become important for subsequent modes.
   :::

The rover is now ready to drive in [Acro mode](../flight_modes_rover/ackermann.md#acro-mode).

## Stabilized Mode

::: warning
For this mode to work properly [Acro mode](#acro-mode) must've already been configured!
:::

For [Stabilized mode](../flight_modes_rover/differential.md#stabilized-mode) the controller utilizes a closed loop yaw controller, which creates a yaw rate setpoint to control the yaw when it is active:

![Cascaded PID for yaw control](../../assets/airframes/rover/rover_differential/cascaded_pid_for_yaw.png)

Unlike the closed loop yaw rate, this controller has no feed-forward term.
Therefore you only need to tune the closed loop gains:

1. [RO_YAW_P](#RO_YAW_P) [-]: Proportional gain for the closed loop yaw controller.

   ::: tip
   In stabilized mode the closed loop yaw control is only active when driving a straight line (no yaw rate input).
   To tune it start with a value of 1 for [RO_YAW_P](#RO_YAW_P).
   Put the rover into stabilized mode and move the left stick of your controller up to drive forwards.
   Disarm the rover and from the flight log plot the `measured_yaw` and the `adjusted_yaw_setpoint` from the [RoverAttitudeStatus](../msg_docs/RoverAttitudeStatus.md) message over each other.
   Increase/Decrease the parameter until you are satisfied with the setpoint tracking.
   :::

::: info
For the closed loop yaw control an integrator gain is useful because this setpoint is often constant for a while and an integrator eliminates steady state errors that can cause the rover to never reach the setpoint.
Since the yaw and yaw rate controllers are cascaded, there only needs to be one integrator which is in the yaw rate controller. If you observe a steady state error in the yaw setpoint increase the [RO_YAW_RATE_I](#RO_YAW_RATE_I) parameter.
:::

The rover is now ready to drive in [Stabilized mode](../flight_modes_rover/ackermann.md#stabilized-mode).

## Position Mode

:::warning
For this mode to work properly [Stabilized mode](#stabilized-mode) must already be configured!
:::

[Position mode](../flight_modes_rover/ackermann.md#position-mode) is the most advanced manual mode, utilizing closed loop yaw rate and speed control and leveraging position estimates.

To configure set the following parameters:

1. [RO_SPEED_LIM](#RO_SPEED_LIM) [m/s]: This is the maximum speed you want to allow for your rover.
   This will define the stick-to-speed mapping for position mode and set an upper limit for the speed setpoint for all [auto modes](#auto-modes).
2. [RO_MAX_THR_SPEED](#RO_MAX_THR_SPEED) [m/s]: This parameter is used to calculate the feed-forward term of the closed loop speed control which linearly maps desired speeds to normalized motor commands.
   As mentioned in the [Manual mode](../flight_modes_rover/ackermann.md#manual-mode) configuration , a good starting point is the observed ground speed when the rover drives at maximum throttle in [Manual mode](../flight_modes_rover/ackermann.md#manual-mode).

   <a id="RA_SPEED_TUNING"></a>

   ::: tip
   To further tune this parameter:

   1. Set [RO_SPEED_P](#RO_SPEED_P) and [RO_SPEED_I](#RO_SPEED_I) to zero.
      This way the speed is only controlled by the feed-forward term, which makes it easier to tune.
   1. Put the rover in [Position mode](../flight_modes_rover/ackermann.md#position-mode) and then move the left stick of your controller up and/or down and hold it at a few different levels for a couple of seconds each.
   1. Disarm the rover and from the flight log plot the `adjusted_speed_body_x_setpoint` and the `measured_speed_body_x` from the [RoverVelocityStatus](../msg_docs/RoverVelocityStatus.md) message over each other.
   1. If the actual speed of the rover is higher than the speed setpoint, increase [RO_MAX_THR_SPEED](#RO_MAX_THR_SPEED).
      If it is the other way around decrease the parameter and repeat until you are satisfied with the setpoint tracking.

   :::

   ::: info
   If your rover oscillates when driving a straight line in [Position mode](../flight_modes_rover/ackermann.md#position-mode), set this parameter to the observed ground speed at maximum throttle in [Manual mode](../flight_modes_rover/ackermann.md#manual-mode) and complete steps 5-7 first before continuing the tuning of the closed loop speed control (Steps 2-4).
   :::

3. [RO_SPEED_P](#RO_SPEED_P) [-]: Proportional gain of the closed loop speed controller.

   ::: tip
   This parameter can be tuned the same way as [RO_MAX_THR_SPEED](#RA_SPEED_TUNING).
   If you tuned [RO_MAX_THR_SPEED](#RO_MAX_THR_SPEED) well, you might only need a very small value.
   :::

4. [RO_SPEED_I](#RO_SPEED_I) [-]: Integral gain for the closed loop speed controller.

   ::: tip
   For the closed loop speed control an integrator gain is useful because this setpoint is often constant for a while and an integrator eliminates steady state errors that can cause the rover to never reach the setpoint.
   :::

5. [PP_LOOKAHD_GAIN](#PP_LOOKAHD_GAIN): When driving in a straight line (right stick centered) position mode leverages the same path following algorithm used in [auto modes](#auto-modes) called [pure pursuit](#pure-pursuit-guidance-logic) to achieve the best possible straight line driving behaviour ([Illustration of control architecture](#pure_pursuit_controller)).
   This parameter determines how aggressive the controller will steer towards the path.

   ::: tip
   Decreasing the parameter makes it more aggressive but can lead to oscillations.

   To tune this:

   1. Start with a value of 1 for [PP_LOOKAHD_GAIN](#PP_LOOKAHD_GAIN)
   2. Put the rover in [Position mode](../flight_modes_rover/ackermann.md#position-mode) and while driving a straight line at approximately half the maximum speed observe its behaviour.
   3. If the rover does not drive in a straight line, reduce the value of the parameter, if it oscillates around the path increase the value.
   4. Repeat until you are satisfied with the behaviour.

   :::

6. [PP_LOOKAHD_MIN](#PP_LOOKAHD_MIN): Minimum threshold for the lookahead distance used by the [pure pursuit algorithm](#pure-pursuit-guidance-logic).

   ::: tip
   Put the rover in [Position mode](../flight_modes_rover/ackermann.md#position-mode) and drive at very low speeds, if the rover starts to oscillate even though the tuning of [PP_LOOKAHD_GAIN](#PP_LOOKAHD_GAIN) was good for medium speeds, then increase the value of [PP_LOOKAHD_MIN](#PP_LOOKAHD_MIN).
   :::

7. [PP_LOOKAHD_MAX](#PP_LOOKAHD_MAX): Maximum threshold for the lookahead distance used by [pure pursuit](#pure-pursuit-guidance-logic).

   ::: tip
   Put the rover in [Position mode](../flight_modes_rover/ackermann.md#position-mode) and drive at very high speeds, if the rover does not drive in a straight line even though the tuning of [PP_LOOKAHD_GAIN](#PP_LOOKAHD_GAIN) was good for medium speeds, then decrease the value of [PP_LOOKAHD_MAX](#PP_LOOKAHD_MAX).
   :::

The rover is now ready to drive in [Position mode](../flight_modes_rover/ackermann.md#position-mode).

## Auto Modes

::: warning
For auto modes to work properly [Manual Mode](#manual-mode), [Acro mode](#acro-mode)and [Position mode](#position-mode) must already be configured!
:::

<a id="pure_pursuit_controller"></a>

In [auto modes](../flight_modes_rover/ackermann.md#auto-modes) the autopilot takes over navigation tasks using the following control architecture:

![Pure Pursuit Controller](../../assets/airframes/rover/rover_ackermann/ackermann_rover_guidance_structure.png)

The required parameter configuration is discussed in the following sections.

### Speed

1. [RO_DECEL_LIM](#RO_DECEL_LIM) [m/s^2] and [RO_JERK_LIM](#RO_JERK_LIM) [m/s^3] are used to calculate a speed trajectory such that the rover reaches the next waypoint with the correct [cornering speed](#cornering-speed).

   ::: tip
   Plan a mission for the rover to drive a square and observe how it slows down when approaching a waypoint:

   - If the rover decelerates too quickly decrease the [RO_DECEL_LIM](#RO_DECEL_LIM) parameter, if it starts slowing down too early increase the parameter.
   - If you observe a jerking motion as the rover slows down, decrease the [RO_JERK_LIM](#RO_JERK_LIM) parameter otherwise increase it as much as possible as it can interfere with the tuning of [RO_DECEL_LIM](#RO_DECEL_LIM).

   These two parameters have to be tuned as a pair, repeat until you are satisfied with the behaviour.
   :::

2. Plot the `adjusted_speed_body_x_setpoint` and `measured_speed_body_x` from the [RoverVelocityStatus](../msg_docs/RoverVelocityStatus.md) message over each other.
   If the tracking of these setpoints is not satisfactory adjust the values for [RO_SPEED_P](#RO_SPEED_P) and [RO_SPEED_I](#RO_SPEED_I).

### Corner Cutting

The module employs a special cornering logic causing the rover to "cut corners" to achieve a smooth trajectory.
This is done by scaling the acceptance radius based on the corner the rover has to drive (for geometric explanation see [Cornering logic](#mission-cornering-logic-info-only)).

![Cornering Logic](../../assets/airframes/rover/rover_ackermann/cornering_comparison.png)

The degree to which corner cutting is allowed can be tuned, or disabled, with the following parameters:

::: info
The corner cutting effect is a tradeoff between how close you get to the waypoint and the smoothness of the trajectory.
:::

1. [NAV_ACC_RAD](#NAV_ACC_RAD) [m]: Default acceptance radius
   This is also used as a lower bound for the acceptance radius scaling.
2. [RA_ACC_RAD_MAX](#RA_ACC_RAD_MAX) [m]: The maximum the acceptance radius can be scaled to.
   Set equal to [NAV_ACC_RAD](#NAV_ACC_RAD) to disable the corner cutting effect.
3. [RA_ACC_RAD_GAIN](#RA_ACC_RAD_GAIN) [-]: This tuning parameter is a multiplicand on the [calculated ideal acceptance radius](#corner-cutting-logic) to account for dynamic effects.

   :::tip
   Initially set this parameter to `1`.
   If you observe the rover overshooting the corner, increase this parameter until you are satisfied with the behaviour.
   Note that the scaling of the acceptance radius is limited by [RA_ACC_RAD_MAX](#RA_ACC_RAD_MAX).
   :::

### Path Following

The [pure pursuit](#pure-pursuit-guidance-logic) algorithm is used to calculate a yaw rate setpoint for the vehicle that is then close loop controlled.
The close loop yaw rate was tuned in the configuration of the [Acro mode](#acro-mode), and the pure pursuit was tuned when setting up the [Position mode](#position-mode).
During any auto navigation task observe the behaviour of the rover.

If you are unsatisfied with the path following, there are 2 steps to take:

1. Plot the `adjusted_yaw_rate_setpoint` and the `measured_yaw_rate` from the [RoverRateStatus](../msg_docs/RoverRateStatus.md) over each other.
   If the tracking of these setpoints is not satisfactory adjust the values for [RO_YAW_RATE_P](#RO_YAW_RATE_P) and [RO_YAW_RATE_I](#RO_YAW_RATE_I).
2. Plot the `adjusted_yaw_setpoint` and the `measured_yaw` from the [RoverAttitudeStatus](../msg_docs/RoverAttitudeStatus.md) over each other.
   If the tracking of these setpoints is not satisfactory adjust the value for [RO_YAW_P](#RO_YAW_RATE_P) and potentially further tune [RO_YAW_RATE_I](#RO_YAW_RATE_I).
3. Steps 1 and 2 ensures accurate setpoint tracking, if the path following is still unsatisfactory you need to further tune the [pure pursuit](#pure-pursuit-guidance-logic) parameters.

## Pure Pursuit Guidance Logic

The desired yaw setpoints are generated using a pure pursuit algorithm.

The controller takes the intersection point between a circle around the vehicle and a line segment.
In mission mode this line is usually constructed by connecting the previous and current waypoint.

![Pure Pursuit Algorithm](../../assets/airframes/rover/flight_modes/pure_pursuit_algorithm.png)

The radius of the circle around the vehicle is used to tune the controller and is often referred to as look-ahead distance.

The look-ahead distance sets how aggressive the controller behaves and is defined as $l_d = v \cdot k$.
It depends on the velocity $v$ of the rover and a tuning parameter $k$ that can be set with the parameter [PP_LOOKAHD_GAIN](#PP_LOOKAHD_GAIN).

::: info
A lower value of [PP_LOOKAHD_GAIN](#PP_LOOKAHD_GAIN) makes the controller more aggressive but can lead to oscillations!
:::

The lookahead is constrained between [PP_LOOKAHD_MAX](#PP_LOOKAHD_MAX) and [PP_LOOKAHD_MIN](#PP_LOOKAHD_MIN).

If the distance from the path to the rover is bigger than the lookahead distance, the rover will target the point on the path that is closest to the rover.

To summarize, the following parameters can be used to tune the controller:

| Parameter                                                                                                | Description                             | Unit |
| -------------------------------------------------------------------------------------------------------- | --------------------------------------- | ---- |
| <a id="PP_LOOKAHD_GAIN"></a>[PP_LOOKAHD_GAIN](../advanced_config/parameter_reference.md#PP_LOOKAHD_GAIN) | Main tuning parameter                   | -    |
| <a id="PP_LOOKAHD_MAX"></a>[PP_LOOKAHD_MAX](../advanced_config/parameter_reference.md#PP_LOOKAHD_MAX)    | Maximum value for the look ahead radius | m    |
| <a id="PP_LOOKAHD_MIN"></a>[PP_LOOKAHD_MIN](../advanced_config/parameter_reference.md#PP_LOOKAHD_MIN)    | Minimum value for the look ahead radius | m    |

## Mission Cornering Logic (Info only)

### Corner Cutting Logic

To enable a smooth trajectory, the acceptance radius of waypoints is scaled based on the angle between a line segment from the current-to-previous and current-to-next waypoints.
The ideal trajectory would be to arrive at the next line segment with the heading pointing towards the next waypoint.
For this purpose the minimum turning circle of the rover is inscribed tangentially to both line segments.

![Cornering Logic](../../assets/airframes/rover/rover_ackermann/cornering_logic.png)

The acceptance radius of the waypoint is set to the distance from the waypoint to the tangential points between the circle and the line segments:

$$
\begin{align*}
r_{min} &= \frac{L}{\sin\left( \delta_{max}\right) } \\
\theta  &= \frac{1}{2}\arccos\left( \frac{\vec{a}*\vec{b}}{|\vec{a}||\vec{b}|}\right) \\
r_{acc} &= \frac{r_{min}}{\tan\left( \theta\right) }
\end{align*}
$$

| Symbol         | Description                        | Unit |
| -------------- | ---------------------------------- | ---- |
| $\vec{a}$      | Vector from current to previous WP | m    |
| $\vec{b}$      | Vector from current to next WP     | m    |
| $r_{min}$      | Minimum turn radius                | m    |
| $\delta_{max}$ | Maximum steer angle                | m    |
| $r_{acc}$      | Acceptance radius                  | m    |

### Cornering Speed

To smoothen the trajectory further and reduce the risk of the rover rolling over, the rover speed is regulated as follows:

1. During cornering the rover drives at the following speed:

   <!-- prettier-ignore -->
   $$ v_{cor, max} = \dot{\psi}_{max} \cdot r $$

   with $r:$ Turning radius for the upcoming corner and $\dot{\psi}_{max}:$ Maximum yaw rate ([RO_YAW_RATE_LIM](#RO_YAW_RATE_LIM)).

2. In between waypoints (straight line) the rover speed is regulated such that it will arrive at the acceptance radius of the waypoint with the desired cornering speed.

The rover is constrained between the maximum speed [RO_SPEED_LIM](#RO_SPEED_LIM) and the speed where the maximum steering angle does not cause the rover to exceed the yaw rate limit:

<!-- prettier-ignore -->
$$ v_{min} = \frac{w_b \cdot \dot{\psi}_{max}}{tan(\delta_{max})} $$

with $w_b:$ Wheel base ([RA_WHEEL_BASE](#RA_WHEEL_BASE)), $\dot{\psi}_{max}:$ Maximum yaw rate ([RO_YAW_RATE_LIM](#RO_YAW_RATE_LIM)) and $\delta_{max}:$ Maximum steering angle ([RA_MAX_STR_ANG](#RA_MAX_STR_ANG)).

## Parameter Overview

List of all parameters of the ackermann rover module:

| Parameter                                                                                                   | Description                                                           | Unit    |
| ----------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------- | ------- |
| <a id="RA_WHEEL_BASE"></a>[RA_WHEEL_BASE](../advanced_config/parameter_reference.md#RA_WHEEL_BASE)          | Wheel base                                                            | m       |
| <a id="RA_MAX_STR_ANG"></a>[RA_MAX_STR_ANG](../advanced_config/parameter_reference.md#RA_MAX_STR_ANG)       | Maximum steering angle                                                | deg     |
| <a id="RO_MAX_THR_SPEED"></a>[RO_MAX_THR_SPEED](../advanced_config/parameter_reference.md#RO_MAX_THR_SPEED) | Speed the rover drives at maximum throttle                            | m/s     |
| <a id="RO_YAW_RATE_LIM"></a>[RO_YAW_RATE_LIM](../advanced_config/parameter_reference.md#RO_YAW_RATE_LIM)    | Maximum allowed yaw rate                                              | m/s^2   |
| <a id="RO_YAW_RATE_P"></a>[RO_YAW_RATE_P](../advanced_config/parameter_reference.md#RO_YAW_RATE_P)          | Proportional gain for yaw rate controller                             | -       |
| <a id="RO_YAW_RATE_I"></a>[RO_YAW_RATE_I](../advanced_config/parameter_reference.md#RO_YAW_RATE_I)          | Integral gain for yaw rate controller                                 | -       |
| <a id="RO_YAW_P"></a>[RO_YAW_P](../advanced_config/parameter_reference.md#RO_YAW_P)                         | Proportional gain for yaw controller                                  | -       |
| <a id="RO_SPEED_LIM"></a>[RO_SPEED_LIM](../advanced_config/parameter_reference.md#RO_SPEED_LIM)             | Maximum allowed speed                                                 | m/s     |
| <a id="RO_SPEED_P"></a>[RO_SPEED_P](../advanced_config/parameter_reference.md#RO_SPEED_P)                   | Proportional gain for speed controller                                | -       |
| <a id="RO_SPEED_I"></a>[RO_SPEED_I](../advanced_config/parameter_reference.md#RO_SPEED_I)                   | Integral gain for speed controller                                    | -       |
| <a id="PP_LOOKAHD_GAIN"></a>[PP_LOOKAHD_GAIN](../advanced_config/parameter_reference.md#PP_LOOKAHD_GAIN)    | Main tuning parameter for pure pursuit                                | -       |
| <a id="PP_LOOKAHD_MAX"></a>[PP_LOOKAHD_MAX](../advanced_config/parameter_reference.md#PP_LOOKAHD_MAX)       | Maximum value for the look ahead radius of the pure pursuit algorithm | m       |
| <a id="PP_LOOKAHD_MIN"></a>[PP_LOOKAHD_MIN](../advanced_config/parameter_reference.md#PP_LOOKAHD_MIN)       | Minimum value for the look ahead radius of the pure pursuit algorithm | m       |
| <a id="NAV_ACC_RAD"></a>[NAV_ACC_RAD](../advanced_config/parameter_reference.md#NAV_ACC_RAD)                | Default acceptance radius                                             | m       |
| <a id="RA_STR_RATE_LIM"></a>[RA_STR_RATE_LIM](../advanced_config/parameter_reference.md#RA_STR_RATE_LIM)    | (Optional) Maximum allowed steering rate                              | deg/s   |
| <a id="RO_ACCEL_LIM"></a>[RO_ACCEL_LIM](../advanced_config/parameter_reference.md#RO_ACCEL_LIM)             | (Optional) Maximum allowed acceleration                               | m/s^2   |
| <a id="RO_DECEL_LIM"></a>[RO_DECEL_LIM](../advanced_config/parameter_reference.md#RO_DECEL_LIM)             | (Optional) Maximum allowed deceleration                               | m/s^2   |
| <a id="RO_JERK_LIM"></a>[RO_JERK_LIM](../advanced_config/parameter_reference.md#RO_JERK_LIM)                | (Optional) Maximum allowed jerk                                       | $m/s^3$ |
| <a id="RA_ACC_RAD_MAX"></a>[RA_ACC_RAD_MAX](../advanced_config/parameter_reference.md#RA_ACC_RAD_MAX)       | (Optional) Maximum radius the acceptance radius can be scaled to      | m       |
| <a id="RA_ACC_RAD_GAIN"></a>[RA_ACC_RAD_GAIN](../advanced_config/parameter_reference.md#RA_ACC_RAD_GAIN)    | (Optional) Tuning parameter for the acceptance radius scaling         | -       |

## See Also

- [Drive Modes (Ackermann Rover)](../flight_modes_rover/ackermann.md).
