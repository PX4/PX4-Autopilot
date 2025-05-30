# 模拟故障保护

[Failsafes](../config/safety.md) define the safe limits/conditions under which you can safely use PX4, and the action that will be performed if a failsafe is triggered (for example, landing, holding position, or returning to a specified point).

在 SITL 中，默认情况下会禁用某一些故障，以便方便模拟使用。
本主题说明如何在实际世界中尝试 SITL 仿真之前测试安全关键行为。

:::info
You can also test failsafes using [HITL simulation](../simulation/hitl.md).
这使得模拟仅适用于连接的 GCS，SDK 或其他 MAVLink 应用程序。
:::

## 数据链路丢失

The _Data Link Loss_ failsafe (unavailability of external data via MAVLink) is enabled by default.
This makes the simulation only usable with a connected GCS, SDK, or other MAVLink application.

Set the parameter [NAV_DLL_ACT](../advanced_config/parameter_reference.md#NAV_DLL_ACT) to the desired failsafe action to change the behavior.
For example, set to `0` to disable it.

:::info
All parameters in SITL including this one get reset when you do `make clean`.
:::

## RC 链接损失

The _RC Link Loss_ failsafe (unavailability of data from a remote control) is enabled by default.
这可以在 GCS UI 中测试电池指示，而不会触发可能中断其他测试的低电池反应。

Set the parameter [NAV_RCL_ACT](../advanced_config/parameter_reference.md#NAV_RCL_ACT) to the desired failsafe action to change the behavior.
For example, set to `0` to disable it.

:::info
All parameters in SITL including this one get reset when you do `make clean`.
:::

## Low Battery

为了模拟丢失和重新获取 GPS 全球定位系统信息，您可以停止/重新启动 GPS 驱动程序。
This enables testing of battery indication in GCS UIs without triggering low battery reactions that might interrupt other testing.

To change this minimal battery percentage value use the parameter [SIM_BAT_MIN_PCT](../advanced_config/parameter_reference.md#SIM_BAT_MIN_PCT).

To control how fast the battery depletes to the minimal value use the parameter [SIM_BAT_DRAIN](../advanced_config/parameter_reference.md#SIM_BAT_DRAIN).

:::tip
By changing [SIM_BAT_MIN_PCT](../advanced_config/parameter_reference.md#SIM_BAT_MIN_PCT) in flight, you can also test regaining capacity to simulate inaccurate battery state estimation or in-air charging technology.
:::

It is also possible to disable the simulated battery using [SIM_BAT_ENABLE](../advanced_config/parameter_reference.md#SIM_BAT_ENABLE) in order to, for example, provide an external battery simulation via MAVLink.

## GPS 损失

[Failure injection](../debug/failure_injection.md) can be used to simulate different types of failures in many sensors and systems.
For example, this can be used to simulate absent or intermittent GPS, RC signal that has stopped or got stuck on a particular value, failure of the avoidance system, and much more.

For example, to simulate GPS failure:

1. Enable the parameter [SYS_FAILURE_EN](../advanced_config/parameter_reference.md#SYS_FAILURE_EN).
2. Enter the following commands on the SITL instance _pxh shell_:

   ```sh
   # Turn (all) GPS off
   failure gps off

   # Turn (all) GPS on
   failure gps ok
   ```

See [System Failure Injection](../debug/failure_injection.md) for a list of supported target sensors and failure modes.
