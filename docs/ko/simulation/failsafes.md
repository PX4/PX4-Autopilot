# 안전장치 시뮬레이션

[Failsafes](../config/safety.md) define the safe limits/conditions under which you can safely use PX4, and the action that will be performed if a failsafe is triggered (for example, landing, holding position, or returning to a specified point).

SITL에서는 시뮬레이션 편리성을 위하여, 일부 안전 장치가 기본적으로 비활성화되어 있습니다.
실환경 테스트 이전에 SITL 시뮬레이션에서 안전에 중요한 기능을 테스트하는 방법을 설명합니다.

:::info
You can also test failsafes using [HITL simulation](../simulation/hitl.md).
HITL은 비행 컨트롤러의 일반 설정 매개변수를 사용합니다.
:::

## 데이터 링크 손실

The _Data Link Loss_ failsafe (unavailability of external data via MAVLink) is enabled by default.
따라서 연결된 GCS, SDK 또는 기타 MAVLink 애플리케이션에서만 시뮬레이션 가능합니다.

Set the parameter [NAV_DLL_ACT](../advanced_config/parameter_reference.md#NAV_DLL_ACT) to the desired failsafe action to change the behavior.
For example, set to `0` to disable it.

:::info
All parameters in SITL including this one get reset when you do `make clean`.
:::

## RC 링크 손실

The _RC Link Loss_ failsafe (unavailability of data from a remote control) is enabled by default.
다른 시험을 가로막는 배터리 용량 부족 상태를 유발하지 않고 지상 통제 장치의 배터리 표시를 시험해볼 수 있습니다.

Set the parameter [NAV_RCL_ACT](../advanced_config/parameter_reference.md#NAV_RCL_ACT) to the desired failsafe action to change the behavior.
For example, set to `0` to disable it.

:::info
All parameters in SITL including this one get reset when you do `make clean`.
:::

## 배터리 부족

시뮬레이션된 배터리는 에너지가 고갈되지 않도록 구현되며, 기본적으로 용량의 50%까지만 소모되므로 보고된 전압이 표시됩니다. <em x-id="3">pxh shell</em>의 SITL 인스턴스에서 <code>param set SIM_GPS_BLOCK 1</code> 명령과 <code>param set SIM_GPS_BLOCK 0</code> 명령을 실행하여 GPS 메시지를 차단하고 해제하는 방식으로 시험해볼 수 있습니다.

To change this minimal battery percentage value use the parameter [SIM_BAT_MIN_PCT](../advanced_config/parameter_reference.md#SIM_BAT_MIN_PCT).

To control how fast the battery depletes to the minimal value use the parameter [SIM_BAT_DRAIN](../advanced_config/parameter_reference.md#SIM_BAT_DRAIN).

:::tip
By changing [SIM_BAT_MIN_PCT](../advanced_config/parameter_reference.md#SIM_BAT_MIN_PCT) in flight, you can also test regaining capacity to simulate inaccurate battery state estimation or in-air charging technology.
:::

It is also possible to disable the simulated battery using [SIM_BAT_ENABLE](../advanced_config/parameter_reference.md#SIM_BAT_ENABLE) in order to, for example, provide an external battery simulation via MAVLink.

## 센서/시스템 장애

[Failure injection](../debug/failure_injection.md) can be used to simulate different types of failures in many sensors and systems.
GPS가 없거나 간헐적으로 발생하는 경우, 특정 값에서 멈추거나 멈추는 RC 신호, 회피 시스템의 오류 등을 시뮬레이션 할 수 있습니다.

GPS 오류를 시뮬레이션하려면 다음을 수행합니다.

1. Enable the parameter [SYS_FAILURE_EN](../advanced_config/parameter_reference.md#SYS_FAILURE_EN).
2. Enter the following commands on the SITL instance _pxh shell_:

   ```sh
   # Turn (all) GPS off
   failure gps off

   # Turn (all) GPS on
   failure gps ok
   ```

See [System Failure Injection](../debug/failure_injection.md) for a list of supported target sensors and failure modes.
