# Fixed-wing Altitude/Position Controller Tuning

이 가이드는 비행 임무와 고도 및 위치 제어 모드에서 고정익 컨트롤러를 정밀 조정에 많은 도움이 됩니다.
PX4 uses TECS for altitude and airspeed control, and NPFG for horizontal heading/position control.

:::info
An incorrectly set gain during tuning can make altitude or heading control unstable.
따라서 TECS 게인 조정시에는 안정된 제어 모드에서 비행기를 비행하고 착륙시킬 수 있어야합니다.
:::

:::tip
All parameters are documented in the [Parameter Reference](../advanced_config/parameter_reference.md#fw-tecs).
이 가이드에서는 중요한 매개변수들을 설명합니다.
:::

## TECS 튜닝 (고도 및 대기 속도)

TECS (Total Energy Control System)는 항공기의 고도 및 대기 속도를 제어하기 위해 스로틀과 피치 각도 설정점을 조정하는 고정익 가이드 알고리즘입니다.
For a detailed description of the TECS algorithm and the control diagram, see [Controller Diagrams](../flight_stack/controller_diagrams.md).

A well-tuned attitude controller is required before tuning TECS: [PID Tuning Guide](../config_fw/pid_tuning_guide_fixedwing.md).

TECS 튜닝은 주로 기체 제한을 올바르게 설정하는 것입니다.
이러한 제한은 아래에 설명된 일련의 비행 기동으로부터 결정될 수있는 매개변수로 설정할 수 있습니다.
Most of the maneuvers required the plane to be flown by a pilot in [Stabilized flight mode](../flight_modes_fw/stabilized.md).

:::tip
It is highly beneficial to have a person available who can read and take note of telemetry data while the pilot is flying the maneuvers.
정확성을 높이기 위해 비행 중에 얻은 데이터를 비행 로그에 기록된 데이터로 확인하는 것이 좋습니다.
:::

#### 1 차 : 트림 조건

Fly in [stabilized mode](../flight_modes_fw/stabilized.md) and find trim values for both throttle and pitch angle for level flight at trim airspeed.
스로틀로 속도와 피치를 조정하여 수평 비행을 유지하십시오.

다음 매개 변수를 설정하십시오.

- [FW_AIRSPD_TRIM](../advanced_config/parameter_reference.md#FW_AIRSPD_TRIM) - set to the desired trim airspeed flown during the maneuver.
- [FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM) - set to the throttle required to fly at trim airspeed.
- [FW_PSP_OFF](../advanced_config/parameter_reference.md#FW_PSP_OFF) - set to the pitch angle required to maintain level flight.

#### 2nd: Airspeed & Throttle Limits

Fly in [stabilized mode](../flight_modes_fw/stabilized.md) and increase throttle while maintaining level flight using pitch control - until the vehicle reaches
the maximum allowed airspeed.

다음 매개 변수를 설정하십시오.

- [FW_THR_MAX](../advanced_config/parameter_reference.md#FW_THR_MAX) - set to the throttle you applied to reach maximum airspeed during level flight.
- [FW_THR_MIN](../advanced_config/parameter_reference.md#FW_THR_MIN) - set to the minimum throttle the plane should fly at.
- [FW_AIRSPD_MAX](../advanced_config/parameter_reference.md#FW_AIRSPD_MAX) - set to the maximum airspeed you achieved during level flight at `FW_THR_MAX`.

#### 3rd: Pitch & Climb Rate Limits

:::warning
Do not use [FW_T_CLMB_MAX](../advanced_config/parameter_reference.md#FW_T_CLMB_MAX), [FW_T_SINK_MAX](../advanced_config/parameter_reference.md#FW_T_SINK_MAX) or [FW_T_SINK_MIN](../advanced_config/parameter_reference.md#FW_T_SINK_MIN) to specify the desired climb or sink performance you would like to get from the vehicle!
The parameters define the operating limitations and they should be set during the tuning phase, as described below.
:::

Fly in stabilized mode, apply full throttle (`FW_THR_MAX`) and slowly increase the pitch angle of the vehicle until the airspeed reaches `FW_AIRSPD_TRIM`.

- [FW_P_LIM_MAX](../advanced_config/parameter_reference.md#FW_P_LIM_MAX) - set to the pitch angle required to climb at trim airspeed when applying `FW_THR_MAX`.
- [FW_T_CLMB_MAX](../advanced_config/parameter_reference.md#FW_T_CLMB_MAX) - set to the climb rate achieved during the climb at `FW_AIRSPD_TRIM`.

Fly in stabilized mode, reduce the throttle to `FW_THR_MIN` and slowly decrease the pitch angle until the vehicle reaches `FW_AIRSPD_MAX`.

- [FW_P_LIM_MIN](../advanced_config/parameter_reference.md#FW_P_LIM_MIN) - set to the pitch angle required to reach `FW_AIRSPD_MAX` at `FW_THR_MIN`.
- [FW_T_SINK_MAX](../advanced_config/parameter_reference.md#FW_T_SINK_MAX) - set to the sink rate achieved during the descent.

Fly in stabilized mode, reduce throttle to `FW_THR_MIN` and adjust the pitch angle such that the plane maintains `FW_AIRSPD_TRIM`.

- [FW_T_SINK_MIN](../advanced_config/parameter_reference.md#FW_T_SINK_MIN) - set to the sink rate achieved while maintaining `FW_AIRSPD_TRIM`.

Specify the target climb and sink rate for autonomous missions by adjusting [FW_T_CLMB_R_SP](../advanced_config/parameter_reference.md#FW_T_CLMB_R_SP) and [FW_T_SINK_R_SP](../advanced_config/parameter_reference.md#FW_T_SINK_R_SP).
These specify the height rates at which the vehicle will climb or descend in order to change altitude.
Furthermore, these two values define the height rate limits commanded by the user in [Altitude mode](../flight_modes_fw/altitude.md) and [Position mode](../flight_modes_fw/position.md).

### FW Path Control Tuning (Position)

All path control parameters are described [here](../advanced_config/parameter_reference.md#fw-path-control).

- [NPFG_PERIOD](../advanced_config/parameter_reference.md#NPFG_PERIOD) - This is the previously called L1 distance and defines the tracking point ahead of the aircraft it's following.
  A value of 10-20 meters works for most aircraft.
  진동없이 반응이 날카로울 때까지 튜닝하는 동안 천천히 줄입니다.
  Vehicles with a slow roll dynamic should have this value increased.
