# Multicopter Racer Setup

This page describes how to setup and configure a multicopter racer for optimal performance (in particular for [Acro mode](../flight_modes_mc/acro.md)).

레이서들은 빠른 기동을 위해 설계된 기체입니다.
가능하면, 유경험자에게 도움을 받는 것이 좋습니다.

:::tip
Many things described here can also be applied to improve the flight performance of other types of multicopters.
:::

:::info
A racer usually omits some sensors (e.g. GPS).
결과적으로 사용 가능한 안전 장치 옵션은 적어지는 것입니다.
:::

## 빌드 옵션

일반적으로 레이서는 일부 센서들을 사용하지 않습니다.

자이로와 가속도계만을 사용하는 최소 구성을 할 수 있습니다.

:::info
If the board has an internal magnetometer, it should not be used (small racers are particularly prone to strong electromagnetic interference).
:::

레이서에는 일반적으로 중량 중가 문제와 충돌시 파손 문제로 인하여 GPS가 없습니다 (GPS + 외부 자력계는 자기 간섭을 피하기 위해 고전류에서 멀리 떨어진 GPS 마스트에 배치해야합니다. 이는 쉽게 파손될 수 있음을 의미합니다.)

그러나 GPS를 추가시에는 초보자들에게 이로운 점이 몇 가지 있습니다.

- 위치 유지모드에서 기체가 한 곳에 머물 수 있습니다.
  방향을 잃거나 브레이크가 필요한 경우에 매우 편리합니다.
  또한 안전하게 착륙할 수 있습니다.
- [Return mode](../flight_modes_mc/return.md) can be used, either on a switch or as RC loss/low battery failsafe.
- 충돌시의 마지막 위치를 파악할 수 있습니다.
- 로그에는 비행 트랙이 포함되어 있으므로 비행을 검토할 수 있습니다 (3D).
  이것은 곡예 비행 기술을 향상에 많은 도움이 됩니다.

:::info
During aggressive acrobatic maneuvers the GPS can lose its position fix for a short time.
If you switch into [position mode](../flight_modes_mc/position.md) during that time, [altitude mode](../flight_modes_mc/altitude.md) will be used instead until the position becomes valid again.
:::

## 하드웨어 설정

다음 단락에서는 기체 제작시 몇 가지 중요한 사항에 대하여 설명합니다.
If you need complete build instructions, you can follow the [QAV-R 5" KISS ESC Racer](../frames_multicopter/qav_r_5_kiss_esc_racer.md) build log.

### 진동 설정

진동을 줄이기 위한 여러가지 조립 방법이 있습니다.
For example, the flight controller can be mounted with vibration dampening foam, or using [O-rings](../frames_multicopter/qav_r_5_kiss_esc_racer.md#mounting).

While there is no single best method, you will typically have fewer problems with vibrations if you use high-quality components (frame, motors, props) as for example used in the [QAV-R 5" KISS ESC Racer](../frames_multicopter/qav_r_5_kiss_esc_racer.md).

Make sure to use **balanced props**.

### 무게 중심

무게 중심이 추력 중심에서 최대한 가까워야 합니다.
좌우 균형은 일반적으로 문제가 되지 않지만, 전후 균형은 문제가 될 수 있습니다.
배터리의 적당한 위치를 표시하여 항상 같은 위치에 장착하는 것이 좋습니다.

:::info
The integral term can account for an imbalanced setup, and a custom mixer can do that even better.
그러나 기체 설정으로 불균형을 수정하는 방법이 제일 좋습니다.
:::

## 소프트웨어 설정

레이서를 조립 후에는 소프트웨어를 설정하여야 합니다.

Go through the [Basic Configuration Guide](../config/index.md).
In particular, set the [Airframe](../config/airframe.md) that most closely matches your frame (typically you will choose the [Generic 250 Racer](../airframes/airframe_reference.md#copter_quadrotor_x_generic_250_racer) airframe, which sets some racer-specific parameters by default).

중요한 매개 변수는 다음과 같습니다.

- Enable One-Shot or DShot by selecting the protocol for a group of outputs during [Actuator Configuration](../config/actuators.md).
- Set the maximum roll-, pitch- and yaw rates for Stabilized mode as desired: [MC_ROLLRATE_MAX](../advanced_config/parameter_reference.md#MC_ROLLRATE_MAX), [MC_PITCHRATE_MAX](../advanced_config/parameter_reference.md#MC_PITCHRATE_MAX) and [MC_YAWRATE_MAX](../advanced_config/parameter_reference.md#MC_YAWRATE_MAX).
  The maximum tilt angle is configured with [MPC_MAN_TILT_MAX](../advanced_config/parameter_reference.md#MPC_MAN_TILT_MAX).
- The minimum thrust [MPC_MANTHR_MIN](../advanced_config/parameter_reference.md#MPC_MANTHR_MIN) should be set to 0.

### 추정기

GPS를 사용하는 경우에는 이 섹션을 건너 뛰고 기본 추정기를 사용할 수 있습니다.
그렇지 않으면 자력계나 기압계를 사용하지 않는 Q 자세 추정기로 전환하여야 합니다.

To enable it set [ATT_EN = 1](../advanced_config/parameter_reference.md#ATT_EN), [EKF2_EN =0 ](../advanced_config/parameter_reference.md#EKF2_EN) and [LPE_EN = 0](../advanced_config/parameter_reference.md#LPE_EN) (for more information see [Switching State Estimators](../advanced/switching_state_estimators.md#how-to-enable-different-estimators)).

Then change the following parameters:

- Set [SYS_HAS_MAG](../advanced_config/parameter_reference.md#SYS_HAS_MAG) to `0` if the system does not have a magnetometer.
- Set [SYS_HAS_BARO](../advanced_config/parameter_reference.md#SYS_HAS_BARO) to `0` if the system does not have a barometer.
- Configure the Q estimator: set [ATT_ACC_COMP](../advanced_config/parameter_reference.md#ATT_ACC_COMP) to `0`, [ATT_W_ACC](../advanced_config/parameter_reference.md#ATT_W_ACC) to 0.4 and [ATT_W_GYRO_BIAS](../advanced_config/parameter_reference.md#ATT_W_GYRO_BIAS) to 0.
  필요한 경우에 차후에 튜닝할 수 있습니다.

### 안전장치

Configure [RC loss and low battery failsafe](../config/safety.md).
If you do not use a GPS, set the failsafe to **Lockdown**, which turns off the motors.
차량이 시동이 켜지면 리모컨을 꺼서 프로펠러를 제거한 다음에, 벤치에서 RC 손실을 테스트합니다.

Make sure to assign a [kill switch](../config/safety.md#kill-switch) or an [arming switch](../config/safety.md#arm-disarm-switch).
테스트하고 연습을 충분히 하여야 합니다.

### PID 튜닝

:::info
Make sure to calibrate the ESCs before doing any tuning.
:::

이 시점에서 첫 번째 테스트 비행을 준비하여야 합니다.

Assuming the vehicle is able to fly using the default settings, we then do a first pass of [Basic MC PID tuning](../config_mc/pid_tuning_guide_multicopter_basic.md).
The vehicle needs to be **undertuned** (the **P** and **D** gains should be set too low), such that there are no oscillations from the controller that could be interpreted as noise (the default gains might be good enough).
This is important for the [filter tuning](#filter-tuning) (there will be a second PID tuning round later).

### 제어 지연

The _control latency_ is the delay from a physical disturbance of the vehicle until the motors react to the change.

:::tip
It is _crucial_ to reduce the control latency as much as possible!
A lower latency allows you to increase the rate **P** gains, which means better flight performance.
지연 시간은 1/1000 초가 추가 되어도 현격한 차이를 나타냅니다.
:::

지연 시간에 영향을 미치는 요인은 다음과 같습니다.

- 부드러운 기체 또는 부드러운 진동 장착은 대기 시간을 증가시킵니다 (필터 역할을 함).
- [Low-pass filters](../config_mc/filter_tuning.md) in software and on the sensor chip trade off increased latency for improved noise filtering.
- PX4 소프트웨어 내부 : 센서 신호를 드라이버에서 읽은 다음 컨트롤러를 통해 출력 드라이버로 전달하여야 합니다.
- The IO chip (MAIN pins) adds about 5.4 ms latency compared to using the AUX pins (this does not apply to a _Pixracer_ or _Omnibus F4_, but does apply to a Pixhawk).
  To avoid the IO delay attach the motors to the AUX pins instead.
- PWM output signal: enable [Dshot](../peripherals/dshot.md) by preference to reduce latency (or One-Shot if DShot is not supported).
  The protocol is selected for a group of outputs during [Actuator Configuration](../config/actuators.md).

### 필터 튜닝

필터는 성능에 영향을 미치는 제어 대기 시간과 노이즈 필터링을 절충합니다.
For information see: [Filter/Control Latency Tuning](../config_mc/filter_tuning.md)

### PID 튜닝 (두 번째 단계)

이제 두 번째 PID 튜닝을 수행합니다. 이번에는 가능한 한 빡빡하게하고 추력 곡선도 튜닝합니다.

:::tip
You can use the approach described in [Basic MC PID tuning](../config_mc/pid_tuning_guide_multicopter_basic.md) to tune the frame, but you will need to use the [Advanced Multicopter PID Tuning Guide (Advanced/Detailed)](../config_mc/pid_tuning_guide_multicopter.md#thrust-curve) to understand how to tune the thrust curve.

### 에어모드

After you have verified that the vehicle flies well at low and high throttle, you can enable [airmode](../config_mc/pid_tuning_guide_multicopter.md#airmode) with the [MC_AIRMODE](../advanced_config/parameter_reference.md#MC_AIRMODE) parameter.
이 기능은 기체가 제어 가능하고 낮은 스로틀에서 속도를 추적하도록 합니다.
