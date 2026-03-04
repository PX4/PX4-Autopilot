# Static Pressure Buildup

Air flowing over an enclosed vehicle can cause the _static pressure_ to change within the canopy/hull.
Depending on the location of holes/leaks in the hull, you can end up with under or overpressure (similar to a wing).

The change in pressure can affect barometer measurements, leading to an inaccurate altitude estimate.
This might manifest as the vehicle losing altitude when it stops moving in [Altitude](../flight_modes_mc/altitude.md), [Position](../flight_modes_mc/position.md) or [Mission](../flight_modes_mc/mission.md) modes (when the vehicle stops moving the static pressure drops, the sensor reports a higher altitude, and the vehicle compensates by descending).
The problem is particularly visible on multicopters because fixed wing vehicles move with a more constant airspeed (and it is the airspeed deltas that are noticeable).

One solution is to use foam-filled venting holes to reduce the buildup (as much as possible) and then attempt dynamic calibration to remove any remaining effects.

:::tip
Before "fixing" the problem you should first check that the Z setpoint tracks the estimated altitude (to verify that there are no controller issues).
:::

::: info
While it is possible to remove the barometer from the altitude estimate (i.e. only use altitude from the GPS), this is not recommended.
GPS is inaccurate in many environments, and particularly in urban environments where you have signal reflections off buildings.
:::

## Airflow Analysis

You can modify the hull by drilling holes or filling them with foam.

One way to analyse the effects of these changes is to mount the drone on a car and drive around (on a relatively level surface) with the hull exposed to air/wind.
By looking at the ground station you can review the effects of movement-induced static pressure changes on the measured altitude (using the road as "ground truth).

This process allows rapid iteration without draining batteries: modify drone, drive/review, repeat!

:::tip
Aim for a barometer altitude drop of less than 2 metres at maximum horizontal speed before attempting software-based calibration below.
:::

## Dynamic Calibration

After modifying the hardware, you can then use the [EKF2_PCOEF\_\*](../advanced_config/parameter_reference.md#EKF2_PCOEF_XN) parameters to tune for expected barometer variation based on relative air velocity.
For more information see [Using PX4's Navigation Filter (EKF2) > Correction for Static Pressure Position Error](../advanced_config/tuning_the_ecl_ekf.md#correction-for-static-pressure-position-error).

::: info
The approach works well if the relationship between the error due to static pressure and the velocity varies linearly.
If the vehicle has a more complex aerodynamic model it will be less effective.
:::
