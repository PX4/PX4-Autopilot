# 착륙 감지기 설정

착륙 감지기는 접지와 착륙 상태에서 기체의 핵심 상태를 나타내는 동적 기체 모델입니다.
이 섹션에서는 기체의 착륙 활동을 개선하는 용도로 미세 조정 가능한 주요 매개변수를 설명합니다.

## 자동 시동 끄기

기체가 착륙하면 착륙 감지기에서 자동으로 시동을 끕니다.

You can set [COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND) to specify the number of seconds after landing that the system should disarm (or turn off auto-disarming by setting the parameter to -1).

## 멀티콥터 설정

The complete set of relevant landing detector parameters are listed in the parameter reference with the prefix [LNDMC](../advanced_config/parameter_reference.md#land-detector) (these can be edited in QGroundControl via the [parameter editor](../advanced_config/parameters.md)).

:::tip
Information about how the parameters affect landing can be found below in [Land Detector States](#mc-land-detector-states).
:::

각 기체에서 착륙 동작 개선용 미세 조정 핵심 매개변수는 다음과 같습니다:

- [MPC_THR_HOVER](../advanced_config/parameter_reference.md#MPC_THR_HOVER) - the hover throttle of the system (default is 50%).
  고도를 정확하게 제어하고 올바른 착륙 감지를 보장하도록 이 매개변수를 정확하게 설정하여야 합니다.
  적재 장치가 없는 레이서 또는 대형 카메라 드론은 좀 더 낮은 값을 설정하여야 합니다(예: 35%).

  ::: info
  Incorrectly setting `MPC_THR_HOVER` may result in ground-contact or maybe-landed detection while still in air (in particular, while descending in [Position mode](../flight_modes_mc/position.md) or [Altitude mode](../flight_modes_mc/altitude.md)).
  이 현상은 기체의 "요동"(모터를 껐다가 즉시 모터가 켜지는) 현상을 유발합니다.

:::

- [MPC_THR_MIN](../advanced_config/parameter_reference.md#MPC_THR_MIN) - the overall minimum throttle of the system.
  제어 하강을 가능하게하기 위하여 설정되어야 합니다.

- [MPC_LAND_CRWL](../advanced_config/parameter_reference.md#MPC_LAND_CRWL) - the vertical speed applied in the last stage of autonomous landing if the system has a distance sensor and it is present and working. Has to be set larger than LNDMC_Z_VEL_MAX.

### MC Land Detector States

멀티콥터는 착륙 감지에 3개의 서로 다른 상태를 거치게 됩니다. 각각의 상태는 이전 상태의 조건에 더해 엄격한 제약조건을 가지게 됩니다.
센서 손실로 인해 조건이 만족되지 않는다면, 기본값으로 그 조건은 참이 됩니다.
For instance, in [Acro mode](../flight_modes_mc/acro.md) and no sensor is active except for the gyro sensor, then the detection solely relies on thrust output and time.

In order to proceed to the next state, each condition has to be true for a third of the configured total land detector trigger time [LNDMC_TRIG_TIME](../advanced_config/parameter_reference.md#LNDMC_TRIG_TIME).
If the vehicle is equipped with a distance sensor, but the distance to ground is currently not measurable (usually because it is too large), the trigger time is increased by a factor of 3.

만약에 조건중 하나라도 만족하지 않으면, 착륙 감지기는 즉시 현재 상태를 벗어납니다.

#### 접지

Conditions for this state:

- no vertical movement ([LNDMC_Z_VEL_MAX](../advanced_config/parameter_reference.md#LNDMC_Z_VEL_MAX))
- no horizontal movement ([LNDMC_XY_VEL_MAX](../advanced_config/parameter_reference.md#LNDMC_XY_VEL_MAX))
- lower thrust than [MPC_THR_MIN](../advanced_config/parameter_reference.md#MPC_THR_MIN) + (hover throttle - [MPC_THR_MIN](../advanced_config/parameter_reference.md#MPC_THR_MIN)) \* (0.3, unless a hover thrust estimate is available, then 0.6),
- additional check if vehicle is currently in a height-rate controlled flight mode: the vehicle has to have the intent to descend (vertical velocity setpoint above LNDMC_Z_VEL_MAX).
- additional check for vehicles with a distance sensor: current distance to ground is below 1m.

If the vehicle is in position- or velocity-control and ground contact was detected,
the position controller will set the thrust vector along the body x-y-axis to zero.

#### 착륙 예측

Conditions for this state:

- all conditions of the [ground contact](#ground-contact) state are true
- is not rotating ([LNDMC_ROT_MAX](../advanced_config/parameter_reference.md#LNDMC_ROT_MAX))
- has low thrust `MPC_THR_MIN + (MPC_THR_HOVER - MPC_THR_MIN) * 0.1`
- no freefall detected

If the vehicle only has knowledge of thrust and angular rate,
in order to proceed to the next state the vehicle has to have low thrust and no rotation for 8.0 seconds.

If the vehicle is in position or velocity control and maybe landed was detected,
the position controller will set the thrust vector to zero.

#### 착륙

Conditions for this state:

- all conditions of the [maybe landed](#maybe-landed) state are true

## Fixed-wing Configuration

Tuning parameters for fixed-wing land detection:

- [LNDFW_AIRSPD_MAX](../advanced_config/parameter_reference.md#LNDFW_AIRSPD_MAX) - the maximum airspeed allowed for the system still to be considered landed.
  Has to be a tradeoff between airspeed sensing accuracy and triggering fast enough.
  좋은 대기속도 센서는 이 파라미터 값을 낮출 수 있게 합니다.
- [LNDFW_VEL_XY_MAX ](../advanced_config/parameter_reference.md#LNDFW_VEL_XY_MAX) - the maximum horizontal velocity for the system to be still be considered landed.
- [LNDFW_VEL_Z_MAX](../advanced_config/parameter_reference.md#LNDFW_VEL_XY_MAX) - the maximum vertical velocity for the system to be still be considered landed.
- [LNDFW_XYACC_MAX](../advanced_config/parameter_reference.md#LNDFW_XYACC_MAX) - the maximal horizontal acceleration for the system to still be considered landed.
- [LNDFW_TRIG_TIME](../advanced_config/parameter_reference.md#LNDFW_TRIG_TIME) - Trigger time during which the conditions above have to be fulfilled to declare a landing.

:::info
When FW launch detection is enabled ([FW_LAUN_DETCN_ON](../advanced_config/parameter_reference.md#FW_LAUN_DETCN_ON)), the vehicle will stay in "landed" state until takeoff is detected (which is purely based on acceleration and not velocity).
:::

## VTOL Land Detector

The VTOL land detector is 1:1 the same as the MC land detector if the system is in hover mode. In FW mode, land detection is disabled.
