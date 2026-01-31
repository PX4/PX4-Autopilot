# 着陆探测器配置

着陆探测器是一个动态飞行器模型，代表了从接触地面到着陆的关键飞行器状态。
本节主题说明您可能希望调整的主要参数，以改善飞行器着陆行为。

## 自动上锁

当飞行器着陆时着陆探测器会为飞行器自动加锁。

You can set [COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND) to specify the number of seconds after landing that the system should disarm (or turn off auto-disarming by setting the parameter to -1).

## 多旋翼配置

The complete set of relevant landing detector parameters are listed in the parameter reference with the prefix [LNDMC](../advanced_config/parameter_reference.md#land-detector) (these can be edited in QGroundControl via the [parameter editor](../advanced_config/parameters.md)).

:::tip
Information about how the parameters affect landing can be found below in [Land Detector States](#mc-land-detector-states).
:::

为了改善特定机架上的着陆，您可能需要调整的其他关键参数包括:

- [MPC_THR_HOVER](../advanced_config/parameter_reference.md#MPC_THR_HOVER) - the hover throttle of the system (default is 50%).
  正确设置这一点是很重要的, 因为它使高度控制更加准确, 并确保正确的地面检测。
  没有安装有效载荷的穿越机或大型相机的无人机可能需要更低的悬停油门 (例如 35%)。

  ::: info
  Incorrectly setting `MPC_THR_HOVER` may result in ground-contact or maybe-landed detection while still in air (in particular, while descending in [Position mode](../flight_modes_mc/position.md) or [Altitude mode](../flight_modes_mc/altitude.md)).
  这将导致载具 "抽搐" （关闭电机，然后立即将其重新打开）。

:::

- [MPC_THR_MIN](../advanced_config/parameter_reference.md#MPC_THR_MIN) - the overall minimum throttle of the system.
  应该将其设置为可控的下降

- [MPC_LAND_CRWL](../advanced_config/parameter_reference.md#MPC_LAND_CRWL) - the vertical speed applied in the last stage of autonomous landing if the system has a distance sensor and it is present and working. Has to be set larger than LNDMC_Z_VEL_MAX.

### MC Land Detector States

为了探测着陆，多旋翼首先必须经历三个不同的状态，其中每个状态都包含来自先前状态的条件以及更严格的约束。
如果由于缺少传感器而无法达到条件，则默认情况下认为该条件为真。
For instance, in [Acro mode](../flight_modes_mc/acro.md) and no sensor is active except for the gyro sensor, then the detection solely relies on thrust output and time.

In order to proceed to the next state, each condition has to be true for a third of the configured total land detector trigger time [LNDMC_TRIG_TIME](../advanced_config/parameter_reference.md#LNDMC_TRIG_TIME).
If the vehicle is equipped with a distance sensor, but the distance to ground is currently not measurable (usually because it is too large), the trigger time is increased by a factor of 3.

如果一个条件失败，则陆地探测器立即退出当前状态。

#### 地面接触

Conditions for this state:

- no vertical movement ([LNDMC_Z_VEL_MAX](../advanced_config/parameter_reference.md#LNDMC_Z_VEL_MAX))
- no horizontal movement ([LNDMC_XY_VEL_MAX](../advanced_config/parameter_reference.md#LNDMC_XY_VEL_MAX))
- lower thrust than [MPC_THR_MIN](../advanced_config/parameter_reference.md#MPC_THR_MIN) + (hover throttle - [MPC_THR_MIN](../advanced_config/parameter_reference.md#MPC_THR_MIN)) \* (0.3, unless a hover thrust estimate is available, then 0.6),
- additional check if vehicle is currently in a height-rate controlled flight mode: the vehicle has to have the intent to descend (vertical velocity setpoint above LNDMC_Z_VEL_MAX).
- additional check for vehicles with a distance sensor: current distance to ground is below 1m.

If the vehicle is in position- or velocity-control and ground contact was detected,
the position controller will set the thrust vector along the body x-y-axis to zero.

#### 可能着陆

Conditions for this state:

- all conditions of the [ground contact](#ground-contact) state are true
- is not rotating ([LNDMC_ROT_MAX](../advanced_config/parameter_reference.md#LNDMC_ROT_MAX))
- has low thrust `MPC_THR_MIN + (MPC_THR_HOVER - MPC_THR_MIN) * 0.1`
- no freefall detected

If the vehicle only has knowledge of thrust and angular rate,
in order to proceed to the next state the vehicle has to have low thrust and no rotation for 8.0 seconds.

If the vehicle is in position or velocity control and maybe landed was detected,
the position controller will set the thrust vector to zero.

#### 降落完成

Conditions for this state:

- all conditions of the [maybe landed](#maybe-landed) state are true

## Fixed-wing Configuration

Tuning parameters for fixed-wing land detection:

- [LNDFW_AIRSPD_MAX](../advanced_config/parameter_reference.md#LNDFW_AIRSPD_MAX) - the maximum airspeed allowed for the system still to be considered landed.
  Has to be a tradeoff between airspeed sensing accuracy and triggering fast enough.
  越好的空速传感器允许此参数的值越低。
- [LNDFW_VEL_XY_MAX ](../advanced_config/parameter_reference.md#LNDFW_VEL_XY_MAX) - the maximum horizontal velocity for the system to be still be considered landed.
- [LNDFW_VEL_Z_MAX](../advanced_config/parameter_reference.md#LNDFW_VEL_XY_MAX) - the maximum vertical velocity for the system to be still be considered landed.
- [LNDFW_XYACC_MAX](../advanced_config/parameter_reference.md#LNDFW_XYACC_MAX) - the maximal horizontal acceleration for the system to still be considered landed.
- [LNDFW_TRIG_TIME](../advanced_config/parameter_reference.md#LNDFW_TRIG_TIME) - Trigger time during which the conditions above have to be fulfilled to declare a landing.

:::info
When FW launch detection is enabled ([FW_LAUN_DETCN_ON](../advanced_config/parameter_reference.md#FW_LAUN_DETCN_ON)), the vehicle will stay in "landed" state until takeoff is detected (which is purely based on acceleration and not velocity).
:::

## VTOL Land Detector

The VTOL land detector is 1:1 the same as the MC land detector if the system is in hover mode. In FW mode, land detection is disabled.
