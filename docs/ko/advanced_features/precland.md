# 정밀 착륙

PX4 supports precision landing for _multicopters_ on either stationary or moving targets.
표적은 온보드 IR 센서와 착륙 표지 또는 오프보드 포지셔닝 시스템에 의해 제공될 수 있습니다.

Precision landing can be [started/initiated](#initiating-a-precision-landing) as part of a [mission](#mission), in a [Return mode](#return-mode-precision-landing) landing, or by entering the [_Precision Land_ flight mode](#precision-landing-flight-mode).

:::info
Precision landing is only possible with a valid global position (due to a limitation in the current implementation of the position controller).
:::

## 개요

### 착륙 모드

정밀 착륙은 "필수" 또는 "가능성 탐색"으로 설정 가능합니다.
선택한 모드에 따라 정밀착륙 매커니즘은 달라집니다.

#### 필수 모드

In _Required Mode_ the vehicle will search for a target if none is visible when landing is initiated.
목표물을 찾은 경우에는 기체는 정밀 착륙을 실행합니다.

The search procedure consists of climbing to the search altitude ([PLD_SRCH_ALT](../advanced_config/parameter_reference.md#PLD_SRCH_ALT)).
If the target is still not visible at the search altitude and after a search timeout ([PLD_SRCH_TOUT](../advanced_config/parameter_reference.md#PLD_SRCH_TOUT)), a normal landing is initiated at the current position.

:::info
If using an offboard positioning system PX4 assumes that the target is visible when it is receiving MAVLink [LANDING_TARGET](https://mavlink.io/en/messages/common.html#LANDING_TARGET) messages.
:::

#### 가능성 탐색 모드

In _Opportunistic Mode_ the vehicle will use precision landing _if_ (and only if) the target is visible when landing is initiated.
If it is not visible the vehicle immediately performs a _normal_ landing at the current position.

### 착륙 과정

정밀 착륙에는 세 단계가 있습니다.

1. **Horizontal approach:** The vehicle approaches the target horizontally while keeping its current altitude.
  Once the position of the target relative to the vehicle is below a threshold ([PLD_HACC_RAD](../advanced_config/parameter_reference.md#PLD_HACC_RAD)), the next phase is entered.
  If the target is lost during this phase (not visible for longer than [PLD_BTOUT](../advanced_config/parameter_reference.md#PLD_BTOUT)), a search procedure is initiated (during a required precision landing) or the vehicle does a normal landing (during an opportunistic precision landing).

2. **Descent over target:** The vehicle descends, while remaining centered over the target.
  If the target is lost during this phase (not visible for longer than `PLD_BTOUT`), a search procedure is initiated (during a required precision landing) or the vehicle does a normal landing (during an opportunistic precision landing).

3. **Final approach:** When the vehicle is close to the ground (closer than [PLD_FAPPR_ALT](../advanced_config/parameter_reference.md#PLD_FAPPR_ALT)), it descends while remaining centered over the target.
  만약 목표물이 이 단계에서 잡히지 않는다면, 기체는 정밀 착륙의 모드와 무관하게 계속 하강합니다.

Search procedures are initiated in the first and second steps, and will run at most [PLD_MAX_SRCH](../advanced_config/parameter_reference.md#PLD_MAX_SRCH) times.
착륙 단계 흐름도

A flow diagram showing the phases can be found in [landing phases flow Diagram](#landing-phases-flow-diagram) below.

## 정밀 착륙 수행

Precision landing can be used in missions, during the landing phase in _Return mode_, or by entering the _Precision Land_ mode.

<a id="mission"></a>

### 미션 모드 정밀 착륙

Precision landing can be initiated as part of a [mission](../flying/missions.md) using [MAV_CMD_NAV_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND) with `param2` set appropriately:

- `0`: Normal landing without using the target.
- `1`: [Opportunistic](#opportunistic-mode) precision landing.
- `2`: [Required](#required-mode) precision landing.

### 리턴 모드 정밀 착륙

Precision landing can be used in the [Return mode](../flight_modes_mc/return.md) landing phase.

This is enabled using the parameter [RTL_PLD_MD](../advanced_config/parameter_reference.md#RTL_PLD_MD), which takes the following values:

- `0`: Precision landing disabled (land as normal).
- `1`: [Opportunistic](#opportunistic-mode) precision landing.
- `2`: [Required](#required-mode) precision landing.

### 정밀 착륙 비행 모드

Precision landing can be enabled by switching to the _Precision Landing_ flight mode.

You can verify this using the [_QGroundControl_ MAVLink Console](../debug/mavlink_shell.md#qgroundcontrol-mavlink-console) to enter the following command:

```sh
commander mode auto:precland
```

:::info
When switching to the mode in this way, the precision landing is always "required"; there is no way to specify the type of landing.
:::

:::info
At time of writing is no _convenient_ way to directly invoke precision landing (other than commanding return mode):

- _QGroundControl_ does not provide it as a UI option.
- [MAV_CMD_NAV_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND) only works in missions.
- [MAV_CMD_DO_SET_MODE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE) should work, but you will need to determine the appropriate base and custom modes used by PX4 to represent the precision landing mode.

:::

## 하드웨어 설정

### IR 센서/비콘 설정

The IR sensor/landing beacon solution requires an [IR-LOCK Sensor](https://irlock.com/products/ir-lock-sensor-precision-landing-kit) and downward facing [distance sensor](../sensor/rangefinders.md) connected to the flight controller, and an IR beacon as a target (e.g. [IR-LOCK MarkOne](https://irlock.com/collections/markone)).
정밀 착륙은 약 10 cm 이내의 오차로 착륙할 수 있게 합니다. GPS 착륙은 수 미터의 오차가 발생할 수 있습니다.

Install the IR-LOCK sensor by following the [official guide](https://irlock.readme.io/v2.0/docs).
센서의 x축이 기체의 y축과 정렬되어 있는지, 센서의 y축이 기체의 -x 방향과 정렬되어 있는지 확인하십시오 (카메라에서 전방으로 90도 기울인 경우).

Install a [range/distance sensor](../sensor/rangefinders.md) (the _LidarLite v3_ has been found to work well).

:::info
Many infrared based range sensors do not perform well in the presence of the IR-LOCK beacon.
호환 가능한 다른 센서는 IR-LOCK 설명서를 참조하십시오.
:::

## 오프보드 포지셔닝

The offboard solution requires a positioning system that implements the MAVLink [Landing Target Protocol](https://mavlink.io/en/services/landing_target.html).
이것은 착륙 목표를 결정하기 위해 모든 위치 지정 메커니즘을 사용할 수 있습니다(예: 컴퓨터 비전 및 시각적 표시).

The system must publish the coordinates of the target in the [LANDING_TARGET](https://mavlink.io/en/messages/common.html#LANDING_TARGET) message.
Note that PX4 _requires_ `LANDING_TARGET.frame` to be [MAV_FRAME_LOCAL_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_NED) and only populates the fields `x`, `y`, and `z`.
The origin of the local NED frame [0,0] is the home position (you can map this home position to global coordinates using [GPS_GLOBAL_ORIGIN](https://mavlink.io/en/messages/common.html#GPS_GLOBAL_ORIGIN)).

PX4 does not explicitly require a [distance sensor](../sensor/rangefinders.md) or other sensors, but will perform better if it can more precisely determine its own position.

## 펌웨어 설정

Precision landing requires the modules `irlock` and `landing_target_estimator`.
이들은 대부분의 비행 컨트롤러에 PX4 펌웨어에 기본적으로 포함되어 있습니다.

FMUv2 기반 컨트롤러에는 기본적으로 포함되지 않습니다.
On these, and other boards where they are not included, you can add them by setting the following keys to 'y' in the relevant configuration file for your flight controller (e.g. as done here for FMUv5: [PX4-Autopilot/boards/px4/fmu-v5/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/default.px4board)):

```
CONFIG_DRIVERS_IRLOCK=y
CONFIG_MODULES_LANDING_TARGET_ESTIMATOR=y
```

## PX4 매개변수 설정

IR-Lock 센서는 기본적으로 비활성화되어 있습니다.
Enable it by setting [SENS_EN_IRLOCK](../advanced_config/parameter_reference.md#SENS_EN_IRLOCK) to `1` (true).

[LTEST_MODE](../advanced_config/parameter_reference.md#LTEST_MODE) determines if the target is assumed to be stationary or moving.
If `LTEST_MODE` is set to moving (e.g. it is installed on a vehicle on which the multicopter is to land), target measurements are only used to generate position setpoints in the precision landing controller.
If `LTEST_MODE` is set to stationary, the target measurements are also used by the vehicle position estimator (EKF2 or LPE).

Other relevant parameters are listed in the parameter reference under [Landing_target estimator](../advanced_config/parameter_reference.md#landing-target-estimator) and [Precision land](../advanced_config/parameter_reference.md#precision-land) parameters.
가장 유용한 몇 가지가 아래에 나열되어 있습니다.

| 매개변수                                                                                                                                            | 설명                                                                                                                                                                                       |
| ----------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="SENS_EN_IRLOCK"></a>[SENS_EN_IRLOCK](../advanced_config/parameter_reference.md#SENS_EN_IRLOCK) | IR-LOCK 센서(외부 I2C). Disable: `0` (default): Enable: `1`).          |
| <a id="LTEST_MODE"></a>[LTEST_MODE](../advanced_config/parameter_reference.md#LTEST_MODE)                                  | Landing target is moving (`0`) or stationary (`1`). 기본값은 이동입니다.                                                    |
| <a id="PLD_HACC_RAD"></a>[PLD_HACC_RAD](../advanced_config/parameter_reference.md#PLD_HACC_RAD)       | 차량이 하강을 시작할 수평 허용 반경입니다. 기본값은 0.2m입니다.                                                                                                   |
| <a id="PLD_BTOUT"></a>[PLD_BTOUT](../advanced_config/parameter_reference.md#PLD_BTOUT)                                     | 착륙 목표 시간 초과 후 목표물이 손실된 것으로 간주됩니다. 기본값은 5초 입니다.                                                                                                           |
| <a id="PLD_FAPPR_ALT"></a>[PLD_FAPPR_ALT](../advanced_config/parameter_reference.md#PLD_FAPPR_ALT)    | 최종 접근 고도. 기본값은 0.1 미터 입니다.                                                                                                               |
| <a id="PLD_MAX_SRCH"></a>[PLD_MAX_SRCH](../advanced_config/parameter_reference.md#PLD_MAX_SRCH)       | 착륙시 최대 검색 시도 횟수입니다.                                                                                                                                                      |
| <a id="RTL_PLD_MD"></a>[RTL_PLD_MD](../advanced_config/parameter_reference.md#RTL_PLD_MD)             | RTL 정밀 지상 모드. `0`: disabled, `1`: [Opportunistic](#opportunistic-mode), `2`: [Required](#required-mode). |

### IR 비콘 스케일링

IR-LOCK 센서의 렌즈 왜곡으로 인해 측정 스케일링이 필수적입니다.

[LTEST_SCALE_X](../advanced_config/parameter_reference.md#LTEST_SCALE_X) and [LTEST_SCALE_Y](../advanced_config/parameter_reference.md#LTEST_SCALE_Y) can be used to scale beacon measurements before they are used to estimate the beacon's position and velocity relative to the vehicle.
Note that `LTEST_SCALE_X` and `LTEST_SCALE_Y` are considered in the sensor frame, not the vehicle frame.

To calibrate these scale parameters, set `LTEST_MODE` to moving, fly your multicopter above the beacon and perform forward-backward and left-right motions with the vehicle, while [logging](../dev_log/logging.md#configuration) `landing_target_pose` and `vehicle_local_position`.
Then, compare `landing_target_pose.vx_rel` and `landing_target_pose.vy_rel` to `vehicle_local_position.vx` and `vehicle_local_position.vy`, respectively (both measurements are in NED frame).
추정된 비컨 속도가 기체 속도보다 일관되게 작거나 크면 스케일 파라미터를 조정하여 보정합니다.

If you observe slow sideways oscillations of the vehicle while doing a precision landing with `LTEST_MODE` set to stationary, the beacon measurements are likely scaled too high and you should reduce the scale parameter in the relevant direction.

## 시뮬레이션

Precision landing with the IR-LOCK sensor and beacon can be simulated in [Gazebo Classic](../sim_gazebo_classic/index.md).

IR-LOCK 비컨과 범위 센서와 IR-LOCK 카메라가 장착된 기체를 사용하여 시뮬레이션을 시작하려면 다음을 실행하십시오.

```sh
make px4_sitl gazebo-classic_iris_irlock
```

You can change the location of the beacon either by moving it in the Gazebo Classic GUI or by changing its location in the [Gazebo world](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/worlds/iris_irlock.world#L42).

## 작동 원리

### 착륙 목표 추정기

The `landing_target_estimator` takes measurements from the `irlock` driver as well as the estimated terrain height to estimate the beacon's position relative to the vehicle.

The measurements in `irlock_report` contain the tangent of the angles from the image center to the beacon.
다른 말로, 측정은 z 성분의 크기가 1이고, 측정은 비컨을 나타내는 벡터의 x와 y성분입니다.
비컨에서 카메라부터의 거리의 측정을 스케일링하는것은 비컨에서 카메라까지의 벡터를 반환합니다.
이것으로 상대 위치는 북쪽으로 정렬되게 회전하고, 기체 자세 추정치를 사용해 기체 프레임을 수평으로 만듭니다.
상대 위치 측정의 x, y 성분은 별도의 칼만 필터로 필터링됩니다. 이 필터는 속도 추정치를 생성하고 일시적으로 생긴 이상값을 거부하는 단순 저대역 필터입니다.

The `landing_target_estimator` publishes the estimated relative position and velocity whenever a new `irlock_report` is fused into the estimate.
비컨이 보이지 않거나, 신호 측정이 거부되면 아무 것도 보고하지 않습니다.
The landing target estimate is published in the `landing_target_pose` uORB message.

### 고급 기체 위치 추정

If the target is specified to be stationary using the parameter `LTEST_MODE`, the vehicle's position/velocity estimate can be improved with the help of the target measurements.
기체의 음의 속도를 측정을 목표물의 속도와 결합하여 추정합니다.

### 착륙 단계 흐름도

This image shows the [landing phases](#landing-phases) as a flow diagram.

![Precision Landing Flow Diagram](../../assets/precision_land/precland-flow-diagram.png)
