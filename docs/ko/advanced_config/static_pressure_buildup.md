# 정압 축적

Air flowing over an enclosed vehicle can cause the _static pressure_ to change within the canopy/hull.
선체의 구멍/누출 위치에 따라 저압 또는 과압 (날개와 유사)이 발생할 수 있습니다.

압력의 변화는 기압계 측정에 영향을 끼치므로, 고도 추정이 정확하지 않을 수 있습니다.
This might manifest as the vehicle losing altitude when it stops moving in [Altitude](../flight_modes_mc/altitude.md), [Position](../flight_modes_mc/position.md) or [Mission](../flight_modes_mc/mission.md) modes (when the vehicle stops moving the static pressure drops, the sensor reports a higher altitude, and the vehicle compensates by descending).
The problem is particularly visible on multicopters because fixed wing vehicles move with a more constant airspeed (and it is the airspeed deltas that are noticeable).

한 가지 해결책은 거품으로 채워진 환기 구멍을 사용하여 축적을 최소화한 다음 (가능한 한 많이) 동적 보정을 시도하여 나머지 효과를 제거하는 것입니다.

:::tip
Before "fixing" the problem you should first check that the Z setpoint tracks the estimated altitude (to verify that there are no controller issues).
:::

:::info
While it is possible to remove the barometer from the altitude estimate (i.e. only use altitude from the GPS), this is not recommended.
GPS는 많은 환경, 특히 건물에서 신호가 반사되는 도시 환경에서는 정확하지 않습니다.
:::

## 기류 분석

구멍을 뚫거나 폼으로 채워 선체를 수정할 수 있습니다.

이러한 변화를 분석하는 방법은 드론을 자동차에 장착하여 선체가 공기/바람에 노출된 상태에서 (상대적으로 평평한 표면에서) 주행하는 것입니다.
지상국을 살펴봄으로써 측정 고도에 대한 움직임으로 인한 정압 변화의 영향을 검토기능합니다(도로를 "지상 사실"로 사용).

이 프로세스는 배터리 소모없이 빠르게 반복(드론 수정, 운전/검토, 반복)할 수 있습니다.

:::tip
Aim for a barometer altitude drop of less than 2 metres at maximum horizontal speed before attempting software-based calibration below.
:::

## 동적 보정

After modifying the hardware, you can then use the [EKF2_PCOEF\_\*](../advanced_config/parameter_reference.md#EKF2_PCOEF_XN) parameters to tune for expected barometer variation based on relative air velocity.
For more information see [Using PX4's Navigation Filter (EKF2) > Correction for Static Pressure Position Error](../advanced_config/tuning_the_ecl_ekf.md#correction-for-static-pressure-position-error).

:::info
The approach works well if the relationship between the error due to static pressure and the velocity varies linearly.
기체에 복잡한 공기 역학 모델이 있으면 효율성이 떨어집니다.
:::
