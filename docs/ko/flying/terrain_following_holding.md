# 지형 추적 및 유지

PX4 supports [Terrain Following](#terrain_following) and [Terrain Hold](#terrain_hold) in [Position](../flight_modes_mc/position.md) and [Altitude modes](../flight_modes_mc/altitude.md), on _multicopters_ and _VTOL vehicles in MC mode_ that have a [distance sensor](../sensor/rangefinders.md).

:::info
PX4 does not "natively" support terrain following in missions.
_QGroundControl_ can be used to define missions that _approximately_ follow terrain (this just sets waypoint altitudes based on height above terrain, where terrain height at waypoints is obtained from a map database).
:::

<a id="terrain_following"></a>

## 지형 추적

_Terrain following_ enables a vehicle to automatically maintain a relatively constant height above ground level when traveling at low altitudes.
이것은 장애물을 피하고 다양한 지형을 비행시 일정 고도 유지에 유용합니다 (예 : 항공 사진).

:::tip
This feature can be enabled in [Position](../flight_modes_mc/position.md) and [Altitude modes](../flight_modes_mc/altitude.md), on _multicopters_ and _VTOL vehicles in MC mode_ that have a [distance sensor](../sensor/rangefinders.md).
:::

When _terrain following_ is enabled, PX4 uses the output of the EKF estimator to provide the altitude estimate, and the estimated terrain altitude (calculated from distance sensor measurements using another estimator) to provide the altitude setpoint.
지면까지의 거리가 변하면, 고도 설정 값이 조정되어지면 위의 높이를 일정하게 유지합니다.

At higher altitudes (when the estimator reports that the distance sensor data is invalid) the vehicle switches to _altitude following_, and will typically fly at a near-constant height above mean sea level (AMSL) using an absolute height sensor for altitude data.

:::info
More precisely, the vehicle will use the available selected sources of altitude data as defined in [Using PX4's Navigation Filter (EKF2) > Height](../advanced_config/tuning_the_ecl_ekf.md#height).
:::

Terrain following is enabled by setting [MPC_ALT_MODE](../advanced_config/parameter_reference.md#MPC_ALT_MODE) to `1`.

<a id="terrain_hold"></a>

## 지형 유지

_Terrain hold_ uses a distance sensor to help a vehicle to better maintain a constant height above ground in altitude control modes, when horizontally stationary at low altitude.
이를 통해 기체는 기압계 드리프트 또는 로터 세척으로 인한 과도한 기압계 간섭으로 인한 고도 변화를 피할 수 있습니다.

:::info
This feature can be enabled in [Position](../flight_modes_mc/position.md) and [Altitude modes](../flight_modes_mc/altitude.md), on _multicopters_ and _VTOL vehicles in MC mode_ that have a [distance sensor](../sensor/rangefinders.md).
:::

When moving horizontally (`speed >` [MPC_HOLD_MAX_XY](../advanced_config/parameter_reference.md#MPC_HOLD_MAX_XY)), or above the altitude where the distance sensor is providing valid data, the vehicle will switch into _altitude following_.

Terrain holding is enabled by setting [MPC_ALT_MODE](../advanced_config/parameter_reference.md#MPC_ALT_MODE) to `2`.

:::info
_Terrain hold_ is implemented similarly to [terrain following](#terrain_following).
EKF 추정기의 출력을 사용하여 고도 추정치를 제공하고 추정 지형 고도 (별도의 단일 상태 지형 추정기를 사용하여 거리 센서 측정에서 계산 됨)를 사용하여 고도 설정치를 제공합니다.
외부 힘으로 인해 지면까지의 거리가 변경되면, 지면 위의 높이를 일정하게 유지하기 위해 고도 설정 값이 조정됩니다.
:::
