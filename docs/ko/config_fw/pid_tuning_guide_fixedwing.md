# Fixed-wing Rate/Attitude Controller Tuning Guide

This guide explains how to manually tune the fixed-wing PID loop.
It is intended for advanced users / experts, as incorrect PID tuning may crash your aircraft.

:::info
[Autotune](../config/autotune_fw.md) is recommended for most users, as it is far faster, easier and provides good tuning for most frames.
예를 들어, 새로운 ESC 또는 모터에는 다른 튜닝 게인이 필요합니다.
:::

## 전제 조건

- Trims must be configured first (before PID turning).
  The [Fixed-Wing Trimming Guide](../config_fw/trimming_guide_fixedwing.md) explains how.
- Incorrectly set gains during tuning can make attitude control unstable.
  A pilot tuning gains should therefore be able to fly and land the plane in [manual](../flight_modes_fw/manual.md) (override) control.
- Excessive gains (and rapid servo motion) can violate the maximum forces of your airframe - increase gains carefully.
- Roll and pitch tuning follow the same sequence.
  The only difference is that pitch is more sensitive to trim offsets, so [trimming](../config_fw/trimming_guide_fixedwing.md) has to be done carefully and integrator gains need more attention to compensate this.

## Establishing the Airframe Baseline

If a pilot capable of manual flight is available, its best to establish a few core system properties on a manual trial.
To do this, fly these maneuvers.
Even if you can't note all the quantities immediately on paper, the log file will be very useful for later tuning.

:::info
All these quantities will be automatically logged.
You only need to take notes if you want to directly move on to tuning without looking at the log files.

- 편리한 속도로 수평 비행하십시오.
  스로틀 스틱 위치와 대기 속도를 기록하십시오 (예 : 70 % → 0.7 스로틀, 15m/s 대기 속도).
- 최대 스로틀과 10-30 초 동안 충분한 대기 속도로 상승하십시오 (예 : 12m/s 대기 속도, 30초에 100m 상승).
- 스로틀이 0이고 적절한 대기 속도로 10-30초 동안 하강합니다 (예 : 18m/s 대기 속도, 30초에 80m 하강).
- 60도 롤이 될 때까지 전체 롤 스틱을 사용하여 오른쪽으로 강하게 쌓은 다음 반대쪽에서 60도까지 전체 롤 스틱으로 왼쪽으로 강하게 저장합니다.
- 45도를 높이고 45도를 낮춥니다.

:::

This guide will use these quantities to set some of the controller gains later on.

## Tune Roll

Tune first the roll axis, then pitch.
The roll axis is safer as an incorrect tuning leads only to motion, but not a loss of altitude.

### 피드포워드 게인 조정

To tune this gain, first set the other gains to their minimum values (nominally 0.005, but check the parameter documentation).

#### Gains to set to minimum values

- [FW_RR_I](../advanced_config/parameter_reference.md#FW_RR_I)
- [FW_RR_P](../advanced_config/parameter_reference.md#FW_RR_P)

#### 튜닝 대상 게인

- [FW_RR_FF](../advanced_config/parameter_reference.md#FW_RR_FF) - start with a value of 0.4.
  Increase this value (doubling each time) until the plane rolls satisfactorily and reaches the setpoint.
  프로세스가 끝나면 게인을 20% 낮춥니 다.

### 속도 게인 조정

- [FW_RR_P](../advanced_config/parameter_reference.md#FW_RR_P) - start with a value of 0.06.
  시스템이 흔들리거나 트위치를 시작할 때까지 이 값을 늘립니다 (매번 두 배로 증가).
  그런 다음 게인을 50% 줄입니다.

### 적분기 게인으로 트림 오프셋 조정

- [FW_RR_I](../advanced_config/parameter_reference.md#FW_RR_I) - start with a value of 0.01.
  명령된 롤 값과 실제 롤 값 사이에 오프셋이 없을 때까지이 값을 늘립니다 (매번 두 배로 증가) (로그 파일을 확인해야 할 가능성이 높음).

## Tune Pitch

The pitch axis might need more integrator gain and a correctly set pitch offset.

### 피드포워드 게인 조정

To tune this gain, set the other gains to their minimum values.

#### Gains to set to minimum values

- [FW_PR_I](../advanced_config/parameter_reference.md#FW_PR_I)
- [FW_PR_P](../advanced_config/parameter_reference.md#FW_PR_I)

#### 튜닝 대상 게인

- [FW_PR_FF](../advanced_config/parameter_reference.md#FW_PR_FF) - start with a value of 0.4.
  Increase this value (doubling each time) until the plane pitches satisfactory and reaches the setpoint.
  프로세스가 끝나면 게인을 20% 낮춥니 다.

### 속도 게인 조정

- [FW_PR_P](../advanced_config/parameter_reference.md#FW_PR_P) - start with a value of 0.04.
  시스템이 흔들리거나 트위치를 시작할 때까지 이 값을 늘립니다 (매번 두 배로 증가).
  그런 다음 게인을 50% 줄입니다.

### 적분기 게인으로 트림 오프셋 조정

- [FW_PR_I](../advanced_config/parameter_reference.md#FW_PR_I) - start with a value of 0.01.
  명령된 피치 값과 실제 피치 값 사이에 오프셋이 없을 때까지이 값을 늘립니다 (매번 두 배로 증가) (로그 파일을 확인해야 할 가능성이 높음).

## Adjusting the Time Constant of the Outer Loop

The overall softness / hardness of the control loop can be adjusted by the time constant.
The default of 0.5 seconds should be fine for normal fixed-wing setups and usually does not require adjustment.

- [FW_P_TC](../advanced_config/parameter_reference.md#FW_P_TC) - set to a default of 0.5 seconds, increase to make the Pitch response softer, decrease to make the response harder.
- [FW_R_TC](../advanced_config/parameter_reference.md#FW_R_TC) - set to a default of 0.5 seconds, increase to make the Roll response softer, decrease to make the response harder.

## Other Tuning Parameters

이 가이드에서는 중요한 매개변수들을 설명합니다.
Additional tuning parameters are documented in the [Parameter Reference](../advanced_config/parameter_reference.md).
