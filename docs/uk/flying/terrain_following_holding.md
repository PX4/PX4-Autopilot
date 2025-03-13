# Слідування місцевості/утримання

PX4 supports [Terrain Following](#terrain_following) and [Terrain Hold](#terrain_hold) in [Position](../flight_modes_mc/position.md) and [Altitude modes](../flight_modes_mc/altitude.md), on _multicopters_ and _VTOL vehicles in MC mode_ that have a [distance sensor](../sensor/rangefinders.md).

:::info
PX4 does not "natively" support terrain following in missions.
_QGroundControl_ can be used to define missions that _approximately_ follow terrain (this just sets waypoint altitudes based on height above terrain, where terrain height at waypoints is obtained from a map database).
:::

<a id="terrain_following"></a>

## Слідування місцевості

_Terrain following_ enables a vehicle to automatically maintain a relatively constant height above ground level when traveling at low altitudes.
Це корисно для уникнення перешкод і для підтримки постійної висоти під час польоту над різноманітним рельєфом (наприклад, для аерофотозйомки).

:::tip
This feature can be enabled in [Position](../flight_modes_mc/position.md) and [Altitude modes](../flight_modes_mc/altitude.md), on _multicopters_ and _VTOL vehicles in MC mode_ that have a [distance sensor](../sensor/rangefinders.md).
:::

When _terrain following_ is enabled, PX4 uses the output of the EKF estimator to provide the altitude estimate, and the estimated terrain altitude (calculated from distance sensor measurements using another estimator) to provide the altitude setpoint.
При зміні відстані до землі установлена точка висоти коригується так, щоб зберегти висоту над землею постійною.

At higher altitudes (when the estimator reports that the distance sensor data is invalid) the vehicle switches to _altitude following_, and will typically fly at a near-constant height above mean sea level (AMSL) using an absolute height sensor for altitude data.

:::info
More precisely, the vehicle will use the available selected sources of altitude data as defined in [Using PX4's Navigation Filter (EKF2) > Height](../advanced_config/tuning_the_ecl_ekf.md#height).
:::

Terrain following is enabled by setting [MPC_ALT_MODE](../advanced_config/parameter_reference.md#MPC_ALT_MODE) to `1`.

<a id="terrain_hold"></a>

## Утримання місцевості

_Terrain hold_ uses a distance sensor to help a vehicle to better maintain a constant height above ground in altitude control modes, when horizontally stationary at low altitude.
Це дозволяє транспортному засобу уникнути зміни висоти через дрейф барометра або надмірні перешкоди барометру від омивання ротора.

:::info
This feature can be enabled in [Position](../flight_modes_mc/position.md) and [Altitude modes](../flight_modes_mc/altitude.md), on _multicopters_ and _VTOL vehicles in MC mode_ that have a [distance sensor](../sensor/rangefinders.md).
:::

When moving horizontally (`speed >` [MPC_HOLD_MAX_XY](../advanced_config/parameter_reference.md#MPC_HOLD_MAX_XY)), or above the altitude where the distance sensor is providing valid data, the vehicle will switch into _altitude following_.

Terrain holding is enabled by setting [MPC_ALT_MODE](../advanced_config/parameter_reference.md#MPC_ALT_MODE) to `2`.

:::info
_Terrain hold_ is implemented similarly to [terrain following](#terrain_following).
Вона використовує вихід оцінювача EKF для надання оцінки висоти та оцінку висоти місцевості (розраховану на основі вимірів датчика відстані за допомогою окремого однорівневого оцінювача місцевості), щоб надати точку установки висоти.
Якщо відстань до землі змінюється через зовнішні сили, точка установки висоти коригується, щоб забезпечити постійну висоту над землею.
:::
