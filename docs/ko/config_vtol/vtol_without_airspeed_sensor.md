# 대기속도 센서 미장착 VTOL

<Badge type="warning" text="Experimental" />

:::warning
Support for VTOLs without an airspeed sensor is considered experimental and should only be attempted by experienced pilots.
The use of an airspeed sensor is recommended.
:::

Fixed-wing vehicles use [airspeed sensors](../sensor/airspeed.md) to determine the speed at which the airplane is moving through the air.
Depending on wind this could vary from groundspeed.
Every airplane has a minimum airspeed below which the airplane will stall.
In mild weather conditions and with settings significantly above stall speed a VTOL can operate without the use of an airspeed sensor.

This guide will outline the parameter settings required to bypass the airspeed sensor for VTOL planes.

:::info
Most settings described here should also be applicable to fixed-wing vehicles that are not VTOL, but this is currently untested.
Transition turning and quad-chute are VTOL-specific.
:::

## 준비

Before attempting to eliminate an airspeed sensor you should first determine a safe throttle level.
Also the duration for a front transition needs to be known.
To do this you can either perform a reference flight with an airspeed sensor or fly the vehicle manually.
In both cases the reference flight should be performed in very low wind conditions.

The flight should be performed at a speed that would be sufficient to fly in high wind conditions and should consist of:

- 성공적인 순방향 전환
- 직선 및 수평 비행
- 적극적인 회전
- 높은 고도로 고속 상승

## 로그 검사

After the reference flight download the log and use [FlightPlot](../log/flight_log_analysis.md#flightplot) (or another analysis tool) to examine the log.
Plot the altitude (`GPOS.Alt`), thrust (`ATC1.Thrust`), groundspeed (Expression: `sqrt(GPS.VelN\^2 + GPS.VelE\^2)`), pitch (`ATT.Pitch`) and roll (`AT.Roll`).

또한 전면 전환이 완료되는 데 걸린 시간도 기록합니다.
이 값은 최소 전환 시간을 설정하는 데 사용됩니다.

마지막으로 크루즈 비행 중 지상 속도를 기록하십시오.
This will be used to set the minimum transition time.
For safety reasons you should add +- 30% to this time.

Finally take note of the groundspeed during cruise flight.
This can be used to tune your throttle setting after the first flight without an airspeed sensor.

## 매개변수 설정

To bypass the airspeed preflight check you need to set [SYS_HAS_NUM_ASPD](../advanced_config/parameter_reference.md#SYS_HAS_NUM_ASPD) to 0.

To prevent an installed airspeed sensor being used for feedback control set [FW_USE_AIRSPD](../advanced_config/parameter_reference.md#FW_USE_AIRSPD) to `False`.
This allows you to test the system's behavior in the airspeed-less setting while still having the actual airspeed reading available to check the safety margin to stall speed etc.

Set the trim throttle ([FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM)) to the percentage as determined from the log of the reference flight.
Note that QGC scales this from `1..100` and the thrust value from the log is scaled from `0..1`.
그러므로, 0.65의 추력을 65로 입력해야합니다.
For safety reasons it is recommended to add +- 10% throttle to the determined value for testing a first flight.

Set the minimum front transition time ([VT_TRANS_MIN_TM](../advanced_config/parameter_reference.md#VT_TRANS_MIN_TM)) to the number of seconds determined from the reference flight and add +- 30% for safety.

### 권장 매개 변수(선택 사항)

Because the risk of stalling is real, it is recommended to set the 'fixed-wing minimum altitude' (a.k.a. 'quad-chute') threshold ([VT_FW_MIN_ALT](../advanced_config/parameter_reference.md#VT_FW_MIN_ALT)).

This will cause the VTOL to transition back to multicopter mode and initiate the [Return mode](../flight_modes_vtol/return.md) below a certain altitude.
이 값을 15 미터 또는 20 미터로 설정하여 멀티콥터가 실속에서 회복할 시간을 제공할 수 있습니다.

The position estimator tested for this mode is EKF2, which is enabled by default (for more information see [Switching State Estimators](../advanced/switching_state_estimators.md#how-to-enable-different-estimators) and [EKF2_EN ](../advanced_config/parameter_reference.md#EKF2_EN)).

## 대기속도 센서가 없는 첫 비행

The values apply to a position controlled flight (like [Hold mode](../flight_modes_fw/hold.md) or [Mission mode](../flight_modes_vtol/mission.md) or Mission mode).
It is therefore recommended that a mission is configured at a safe altitude, approximately 10m above the quad-chute threshold.

기준 비행과 같이, 이 비행은 풍속이 매우 작은 조건에서 수행되어야 합니다.
For the first flight the following is recommended:

- 같은 고도 유지
- 웨이포인트를 충분히 넓고 급회전이 필요하지 않도록 설정하십시오.
- 수동 조작이 필요한 경우 시야에 들어오도록 임무를 작게 유지하십시오.
- If the airspeed is very high, consider performing a manual back transition by switching to Altitude mode.

임무가 성공적으로 완료되면, 로그에서 다음 사항을 확인하여야 합니다.

- The groundspeed should be considerably above the groundspeed from the reference flight.
- 고도는 기준 비행보다 크게 낮아서는 안됩니다.
- 피치 각도는 기준 비행과 일관되게 다르지 않아야합니다.

이러한 모든 조건이 충족되면, 지면 속도가 기준 비행 속도와 일치할 때까지 작은 단계로 크루즈 스로틀을 조정할 수 있습니다.

## Parameter Overview

The relevant parameters are:

- [FW_USE_AIRSPD](../advanced_config/parameter_reference.md#FW_USE_AIRSPD)
- [SYS_HAS_NUM_ASPD](../advanced_config/parameter_reference.md#SYS_HAS_NUM_ASPD)
- [EKF2_EN](../advanced_config/parameter_reference.md#EKF2_EN) (1), [ATT_EN](../advanced_config/parameter_reference.md#ATT_EN) (0), [LPE_EN](../advanced_config/parameter_reference.md#LPE_EN) (0)
- [FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM): determined (e.g. 70%)
- [VT_TRANS_MIN_TM](../advanced_config/parameter_reference.md#VT_TRANS_MIN_TM): determined (e.g. 10 seconds)
- [VT_FW_MIN_ALT](../advanced_config/parameter_reference.md#VT_FW_MIN_ALT): 15
