# QGroundControl Flight-Readiness Status

PX4 performs a number of preflight sensor quality and estimator checks to determine if, for example, there is a good enough position estimate to fly the vehicle in the current mode, and will block arming if the vehicle is not ready.

QGroundControl can be used to determine whether the vehicle is ready to fly, and more importantly what checks are failing.

:::tip
You can also get readiness notifications from the [vehicle status LEDs](../getting_started/led_meanings.md) and [warning tunes](../getting_started/tunes.md).
However QGC is the only way to determine the precise reasons why PX4 will not arm.
:::

## Flight Readiness Status

The overall "readiness to fly" is displayed in QGroundControl in the top left corner near the **Q** menu icon, as shown below:

![QGC flight readiness indicators from top left corner](../../assets/flying/qgc_flight_readiness.png)

The three states are:

- "Ready to Fly" (Green background): The vehicle is ready to fly in all modes, and can be armed.
- "Ready to Fly" (Amber background): The vehicle is ready to fly in the current mode and can be armed, but some check is failing that means it will not be able to switch to some other mode.
- "Not Ready" (Amber background): The vehicle is not ready to fly in the current mode, and cannot be armed.

## QGC Arming Check Report

<Badge type="tip" text="PX4 v1.14" /> <Badge type="tip" text="QGC v4.2.0" />

You can find out what prearming checks are failing using the QGroundControl [Arming Check Report](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html#arm) in _Fly View_.
To access this UI select the [Flight Readiness Status](#flight-readiness-status) indicator in the top left corner of QGroundControl's Fly View.

![QGC Arming Check Report](../../assets/flying/qgc_arming_checks_ui.png)

The Arming Check Report will then pop up and list all current warnings, with a toggle on the right of each warning that expands each entry with additional information and possible solutions.

Once each issue is resolved it will disappear from the UI. When all issues blocking arming have been removed you can use the arm button to display the arming confirmation slider, and arm the vehicle (or you can just take off).

:::tip
The QGC Arming Checks UI is available in the QGC Daily Build (QGC v4.2.0 and later), and works with PX4 v1.14 and later.
:::

## Flight Logs

Preflight errors are also reported in _QGroundControl_ as `PREFLIGHT FAIL` messages.
The `estimator_status.gps_check_fail_flags` message [in the logs](../getting_started/flight_reporting.md) shows which GPS quality checks are failing.

Note that the [Arming Check Report](#qgc-arming-check-report) is a much easier way to determine reasons for failure, but the logs may be useful in versions prior to PX4 v1.14.

## EKF 비행 사전 검사와 오류 메시지

This sections lists errors, with associated checks and parameters, that are reported by [EKF2](../advanced_config/tuning_the_ecl_ekf.md) (and propagate to _QGroundControl_).
These are provided for information only (the QGC Arming Checks UI is the best way to get error and solution information).

#### 사전 확인 실패 : EKF 높은 IMU 액셀 바이어스  :

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/Arming/PreFlightCheck/checks/ekf2Check.cpp#L267 -->

<!-- Useful primer on biases: https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-imuspecs -->

<!-- Mathieu Bresciani is expert -->

EKF IMU 가속 바이어스는 IMU 센서에서 보고한 측정된 가속도와 EKF2 추정기에서 보고한 예상 가속도 간의 차이입니다(IMU, GNSS, 유량 센서 등을 포함한 여러 소스의 위치 및/또는 속도 데이터를 융합합니다).
This bias may change when the sensor is turned on ("turn-on bias") and over time due to noise and temperature differences ("in-run bias").
숫자는 일반적으로 매우 작아(0에 가까움) 다른 소스의 측정이 모두 가속도에 동의함을 나타냅니다.

경고는 바이어스가 임의의 임계값보다 높다는 것을 나타냅니다(기체 이륙 불가).
가속도계 또는 열 보정이 필요하다는 신호일 가능성이 높습니다.

- If you _sometimes_ get the warning: [re-calibrate the accelerometer](../config/accelerometer.md).
- If you get _regularly_ get the warning: Perform a [thermal calibration](../advanced_config/sensor_thermal_calibration.md).
- 열 보정 후에도 경고가 계속 표시되는 경우(또는 열 보정을 수행할 수 없는 경우):
  - 센서 또는 자동 조종 장치 하드웨어에서 문제가 발생하지 않는 지 확인합니다.
    - 이를 수행하는 가장 쉬운 방법은 다른 자동 조종 장치로 동일한 프레임/센서를 테스트하는 것입니다.
    - Alternatively, [log and compare](../dev_log/logging.md#configuration) all accelerometers across a number of bench test runs with `6: Sensor comparison` enabled in [SDLOG_PROFILE](../advanced_config/parameter_reference.md#SDLOG_PROFILE).
  - 가속도계 바이어스 학습 조정 매개변수 변경을 시도합니다.

매개변수를 늘리면 자동 조종 장치가 이상을 감지할 가능성이 줄어들고 추정기의 안정성을 수정할 수 있습니다.
그러나 센서에 다른 방법으로 해결할 수 없는 문제가 있는 경우 필요할 수 있습니다(예: 더 나은 성능을 위해 EKF를 조정할 수 있지만 가속도계를 "더 잘" 보정할 수 있는 방법은 없음).

:::warning
Tuning these parameters is a last resort.
추정기 성능이 향상될 수 있는 데이터가 있는 경우에만 시도합니다.
:::

| 매개변수                                                                                                                                                                       | 설명                                                                                                                                                                                                                                                                                                                                                                                                              |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="EKF2_ABL_LIM"></a>[EKF2_ABL_LIM](../advanced_config/parameter_reference.md#EKF2_ABL_LIM)                                  | EKF가 추정할 수 있는 최대 바이어스 값(이 값을 초과하면 바이어스가 잘리고 EKF는 자체 재설정을 시도하며 다중 EKF 시스템에서 작동하는 IMU가 있는 더 건강한 EKF로 전환할 수도 있음). The autopilot will report a "high accel bias" if the estimated bias exceeds 75% of this parameter during a preflight check and prevent takeoff. 0.4m/s2의 현재 값은 이미 상당히 높으며, 이를 높이면 자동조종장치가 문제를 감지할 가능성이 줄어듭니다. |
| <a id="EKF2_ABIAS_INIT"></a>[EKF2_ABIAS_INIT](../advanced_config/parameter_reference.md#EKF2_ABIAS_INIT)                         | Initial bias uncertainty (if perfectly calibrated, this is related to the "turn-on bias" of the sensor). 일부 사용자는 센서가 잘 보정되어 있고 켜기 바이어스가 작다는 것을 알고 있으면 이 값을 줄이고 싶어할 수 있습니다.                                                                                                                                                                                   |
| <a id="EKF2_ACC_B_NOISE"></a>[EKF2_ACC_B_NOISE](../advanced_config/parameter_reference.md#EKF2_ACC_B_NOISE) | The expected "in-run bias" of the accelerometer or "how fast do we expect the bias to change per second". 기본적으로 이 값은 온도 변화로 인한 드리프트를 포함할 만큼 충분히 큽니다. IMU가 온도 보정된 경우 사용자는 이 매개변수를 줄이기를 원할 수 있습니다.                                                                                                                                                                |
| <a id="EKF2_ABL_ACCLIM"></a>[EKF2_ABL_ACCLIM](../advanced_config/parameter_reference.md#EKF2_ABL_ACCLIM)                         | 추정자가 가속도 편향을 학습하려고 시도하는 최대 가속도입니다. 이는 추정자가 비선형성 및 스케일 팩터 오류로 인한 편향을 학습하는 것을 방지하기 위한 것입니다. (거의 어떤 사용자도 자신이 하는 일을 정말로 알고 있는 경우를 제외하고는 해당 매개변수를 변경할 필요가 없습니다.)                                                                                                                                                                                  |

#### PREFLIGHT FAIL: EKF HIGH IMU GYRO BIAS

- 이 오류는 EKF에 의해 추정 된 IMU 자이로 바이어스가 과도할 때 발생합니다.
- 이 경우 과도하다는 것은 바이어스 추정치가 10deg/s(구성된 제한의 절반, 20deg/s로 하드코딩됨)를 초과함을 의미합니다.

#### PREFLIGHT FAIL: ACCEL SENSORS INCONSISTENT - CHECK CALIBRATION

- 이 오류 메시지는 다른 IMU의 가속 측정 불일치시에 발생합니다.
- 이 검사는 IMU가 두 개 이상인 보드에만 적용됩니다.
- The check is controlled by the [COM_ARM_IMU_ACC](../advanced_config/parameter_reference.md#COM_ARM_IMU_ACC) parameter.

#### PREFLIGHT FAIL: GYRO SENSORS INCONSISTENT - CHECK CALIBRATION

- 이 오류 메시지는 다른 IMU의 각속도 측정 불일치시에 발생합니다.
- 이 검사는 IMU가 두 개 이상인 보드에만 적용됩니다.
- The check is controlled by the [COM_ARM_IMU_GYR](../advanced_config/parameter_reference.md#COM_ARM_IMU_GYR) parameter.

#### PREFLIGHT FAIL: COMPASS SENSORS INCONSISTENT - CHECK CALIBRATION

- 이 오류 메시지는 다른 나침반 센서의 측정 차이가 과도한 경우에 생성됩니다.
- 잘못된 교정, 방향 또는 자기 간섭을 나타냅니다.
- 이 검사는 두 개 이상의 나침반이 연결된 경우에만 해당됩니다.
- The check is controlled by the [COM_ARM_MAG_ANG](../advanced_config/parameter_reference.md#COM_ARM_MAG_ANG) parameter.

#### PREFLIGHT FAIL: EKF INTERNAL CHECKS

- 이 오류 메시지는 수평 GPS 속도, 자기 편 요각, 수직 GPS 속도 또는 수직 위치 센서 (기본적으로 Baro이지만 비표준 매개 변수가 사용되는 경우 거리 측정기 또는 GPS 일 수 있음)의 혁신 크기가 과도한 경우 발생합니다. 혁신은 관성 항법 계산에 의한 예측치와 센서 측정치의 차이입니다.
- 사용자는 로그 파일에서 혁신 수준을 확인하여 원인을 파악하여야합니다. These can be found under the `ekf2_innovations` message.
  일반적으로 많이 일어나는 문제들은 아래와 같습니다.
  - 워밍업시 IMU 드리프트. 자동 조종 장치를 다시 시작하면 문제를 해결할 수 있습니다. IMU 가속도와 및 자이로 보정이 필요할 수 있습니다.
  - 차량 움직임과 관련된 인접 자기 간섭. 이동중인 차량을 해결하고 대기 중이거나 전원을 다시 켜십시오.
  - 차량 움직임과 관련된 잘못된 자력계 보정. 재보정으로 문제를 해결하십시오.
  - 시작시 초기 충격 또는 빠른 움직임으로 인해 관성 탐색 솔루션이 잘못되었습니다. 차량을 다시 시작하고, 처음 5 초 동안 움직임을 최소화하여 문제를 해결하십시오.

## 기타 매개 변수:

비행사전검사와 관련된 매개변수들은 다음과 같습니다.

#### COM_ARM_WO_GPS

The [COM_ARM_WO_GPS](../advanced_config/parameter_reference.md#COM_ARM_WO_GPS) parameter controls whether or not arming is allowed without a global position estimate.

- `1` (default): Arming _is_ allowed without a position estimate for flight modes that do not require position information (only).
- `0`: Arming is allowed only if EKF is providing a global position estimate and EFK GPS quality checks are passing
