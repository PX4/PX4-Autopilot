# 切换状态估计器

此页显示了可用的状态估计器以及如何在它们之间切换。

:::tip
EKF2 is the default and should be used unless you have a reason not to (in particular on vehicles with a GNSS/GPS).
The Q-Estimator can be used if you don't have GPS, and is commonly used in [multicopter racers](../config_mc/racer_setup.md).
:::

## 可用的估计器

可用的估计器如下：

- **EKF2 attitude, position and wind states estimator** (_recommended_) - An extended Kalman filter estimating attitude, 3D position / velocity and wind states.

- **LPE position estimator** (_deprecated_) - An extended Kalman filter for 3D position and velocity states.

  :::warning
  LPE is deprecated.
  （在开发 PX4 v1.14时）它是工作的，但是不再支持或维护。

:::

- **Q attitude estimator** - A very simple, quaternion based complementary filter for attitude.
  它不需要 GPS、磁力计或气压计。
  <!-- Q estimator is supported (at time of writing in PX4 v1.14). Test added in PX4-Autopilot/pull/21922 -->

## 如何启用不同的估计器

<!-- Changed in https://github.com/PX4/PX4-Autopilot/pull/22567 after v1.14 -->

要启用特定的估算器，请启用其参数并禁用其他参数：

- [EKF2_EN](../advanced_config/parameter_reference.md#EKF2_EN) - EKF2 (default/recommended)
- [ATT_EN](../advanced_config/parameter_reference.md#ATT_EN) - Q Estimator (quaternion based attitude estimator)
- [LPE_EN](../advanced_config/parameter_reference.md#LPE_EN) - LPE (not supported for Fixed-wing)

:::warning
It is important to enable one, and only one, estimator.
If more than one is enabled, the first to publish the UOrb topics [vehicle_attitude](../msg_docs/VehicleAttitude.md) or [vehicle_local_position](../msg_docs/VehicleLocalPosition.md) is used.
如果没有启用，主题将不发布。
:::

:::info
For FMU-v2 (only) you will also need to build PX4 to specifically include required estimator (e.g. EKF2: `make px4_fmu-v2`, LPE: `make px4_fmu-v2_lpe`).
这是因为 FMU-v2 不具有足够的资源同时包含这两个估计器。
其他的 Pixhawk FMU 版本同时拥有两个估计器。
:::
