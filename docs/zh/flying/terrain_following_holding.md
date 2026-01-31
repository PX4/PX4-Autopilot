# 地形跟随/保持

PX4 supports [Terrain Following](#terrain_following) and [Terrain Hold](#terrain_hold) in [Position](../flight_modes_mc/position.md) and [Altitude modes](../flight_modes_mc/altitude.md), on _multicopters_ and _VTOL vehicles in MC mode_ that have a [distance sensor](../sensor/rangefinders.md).

:::info
PX4 does not "natively" support terrain following in missions.
_QGroundControl_ can be used to define missions that _approximately_ follow terrain (this just sets waypoint altitudes based on height above terrain, where terrain height at waypoints is obtained from a map database).
:::

<a id="terrain_following"></a>

## Terrain Following

_Terrain following_ enables a vehicle to automatically maintain a relatively constant height above ground level when traveling at low altitudes.
This is useful for avoiding obstacles and for maintaining constant height when flying over varied terrain (e.g. for aerial photography).

:::tip
This feature can be enabled in [Position](../flight_modes_mc/position.md) and [Altitude modes](../flight_modes_mc/altitude.md), on _multicopters_ and _VTOL vehicles in MC mode_ that have a [distance sensor](../sensor/rangefinders.md).
:::

When _terrain following_ is enabled, PX4 uses the output of the EKF estimator to provide the altitude estimate, and the estimated terrain altitude (calculated from distance sensor measurements using another estimator) to provide the altitude setpoint.
As the distance to ground changes, the altitude setpoint adjusts to keep the height above ground constant.

At higher altitudes (when the estimator reports that the distance sensor data is invalid) the vehicle switches to _altitude following_, and will typically fly at a near-constant height above mean sea level (AMSL) using an absolute height sensor for altitude data.

:::info
More precisely, the vehicle will use the available selected sources of altitude data as defined in [Using PX4's Navigation Filter (EKF2) > Height](../advanced_config/tuning_the_ecl_ekf.md#height).
:::

Terrain following is enabled by setting [MPC_ALT_MODE](../advanced_config/parameter_reference.md#MPC_ALT_MODE) to `1`.

<a id="terrain_hold"></a>

## Terrain Hold

_Terrain hold_ uses a distance sensor to help a vehicle to better maintain a constant height above ground in altitude control modes, when horizontally stationary at low altitude.
This allows a vehicle to avoid altitude changes due to barometer drift or excessive barometer interference from rotor wash.

:::info
This feature can be enabled in [Position](../flight_modes_mc/position.md) and [Altitude modes](../flight_modes_mc/altitude.md), on _multicopters_ and _VTOL vehicles in MC mode_ that have a [distance sensor](../sensor/rangefinders.md).
:::

When moving horizontally (`speed >` [MPC_HOLD_MAX_XY](../advanced_config/parameter_reference.md#MPC_HOLD_MAX_XY)), or above the altitude where the distance sensor is providing valid data, the vehicle will switch into _altitude following_.

Terrain holding is enabled by setting [MPC_ALT_MODE](../advanced_config/parameter_reference.md#MPC_ALT_MODE) to `2`.

:::info
_Terrain hold_ is implemented similarly to [terrain following](#terrain_following).
It uses the output of the EKF estimator to provide the altitude estimate, and the estimated terrain altitude (calculated from distance sensor measurements using a separate, single state terrain estimator) to provide the altitude setpoint.
If the distance to ground changes due to external forces, the altitude setpoint adjusts to keep the height above ground constant.
:::
