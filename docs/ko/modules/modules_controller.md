# 모듈 참조: 콘트롤러

## airship_att_control

Source: [modules/airship_att_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/airship_att_control)

### 설명

이것은 비행선 자세 및 속도 컨트롤러를 구현합니다. Ideally it would
take attitude setpoints (`vehicle_attitude_setpoint`) or rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

Currently it is feeding the `manual_control_setpoint` topic directly to the actuators.

### 구현

제어 대기 시간을 줄이기 위하여, 모듈은 IMU 드라이버에서 게시한 자이로 주제를 직접 폴링합니다.

<a id="airship_att_control_usage"></a>

### 사용법

```
airship_att_control <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## control_allocator

Source: [modules/control_allocator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/control_allocator)

### 설명

이것은 제어 할당을 구현합니다. 토크 및 추력 설정값을 입력으로 사용하고, 액추에이터 설정값 메시지를 출력합니다.

<a id="control_allocator_usage"></a>

### 사용법

```
control_allocator <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## flight_mode_manager

Source: [modules/flight_mode_manager](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/flight_mode_manager)

### 설명

이것은 모든 모드에 대한 설정값 생성을 구현합니다. 차량의 현재 모드 상태를 컨트롤러에 대한 입력 및 출력 설정값으로 사용합니다.

<a id="flight_mode_manager_usage"></a>

### 사용법

```
flight_mode_manager <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## fw_att_control

Source: [modules/fw_att_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/fw_att_control)

### 설명

fw_att_control은 고정익 자세 컨트롤러입니다.

<a id="fw_att_control_usage"></a>

### 사용법

```
fw_att_control <command> [arguments...]
 Commands:
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```

## fw_pos_control

Source: [modules/fw_pos_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/fw_pos_control)

### 설명

fw_pos_control is the fixed-wing position controller.

<a id="fw_pos_control_usage"></a>

### 사용법

```
fw_pos_control <command> [arguments...]
 Commands:
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```

## fw_rate_control

Source: [modules/fw_rate_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/fw_rate_control)

### 설명

fw_rate_control is the fixed-wing rate controller.

<a id="fw_rate_control_usage"></a>

### 사용법

```
fw_rate_control <command> [arguments...]
 Commands:
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```

## mc_att_control

Source: [modules/mc_att_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_att_control)

### 설명

이것은 멀티콥터 자세 컨트롤러를 구현합니다. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

컨트롤러에는 각도 오류에 대한 P 루프가 있습니다.

간행물: 구현된 쿼터니언 태도 제어를 문서화,
제목: 비선형 쿼드로콥터 자세 제어(2013),
저자: Dario Brescianini, Markus Hehn and Raffaello D'Andrea
동적 시스템 및 제어 연구소(IDSC), ETH 취리히

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

<a id="mc_att_control_usage"></a>

### 사용법

```
mc_att_control <command> [arguments...]
 Commands:
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```

## mc_pos_control

Source: [modules/mc_pos_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_pos_control)

### 설명

컨트롤러에는 위치 오류용 P 루프와 속도 오류용 PID 루프의 두 가지 루프가 있습니다.
속도 컨트롤러의 출력은 추력 방향(즉, 멀티콥터 방향에 대한 회전 행렬)과 추력 스칼라(즉, 멀티콥터 추력 자체)로 분할되는 추력 벡터입니다.

컨트롤러는 작업에 오일러 각도를 사용하지 않으며, 보다 인간 친화적인 제어 및 로깅을 위해서만 생성됩니다.

<a id="mc_pos_control_usage"></a>

### 사용법

```
mc_pos_control <command> [arguments...]
 Commands:
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```

## mc_rate_control

Source: [modules/mc_rate_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_rate_control)

### 설명

이것은 멀티콥터 속도 컨트롤러를 구현합니다. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

컨트롤러에는 각속도 오류에 대한 PID 루프가 있습니다.

<a id="mc_rate_control_usage"></a>

### 사용법

```
mc_rate_control <command> [arguments...]
 Commands:
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```

## navigator

Source: [modules/navigator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/navigator)

### 설명

자율 비행 모드를 담당하는 모듈입니다. 여기에는 임무(데이터맨에서 읽기), 이륙 및 RTL이 포함됩니다.
또한, 지오펜스 위반 검사를 담당합니다.

### 구현

The different internal modes are implemented as separate classes that inherit from a common base class `NavigatorMode`.
The member `_navigation_mode` contains the current active mode.

Navigator publishes position setpoint triplets (`position_setpoint_triplet_s`), which are then used by the position
controller.

<a id="navigator_usage"></a>

### 사용법

```
navigator <command> [arguments...]
 Commands:
   start

   fencefile     load a geofence file from SD card, stored at etc/geofence.txt

   fake_traffic  publishes 24 fake transponder_report_s uORB messages

   stop

   status        print status info
```

## rover_ackermann

Source: [modules/rover_ackermann](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/rover_ackermann)

### 설명

Rover ackermann module.

<a id="rover_ackermann_usage"></a>

### 사용법

```
rover_ackermann <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## rover_differential

Source: [modules/rover_differential](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/rover_differential)

### 설명

Rover differential module.

<a id="rover_differential_usage"></a>

### 사용법

```
rover_differential <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## rover_mecanum

Source: [modules/rover_mecanum](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/rover_mecanum)

### 설명

Rover mecanum module.

<a id="rover_mecanum_usage"></a>

### 사용법

```
rover_mecanum <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## rover_pos_control

Source: [modules/rover_pos_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/rover_pos_control)

### 설명

L1 컨트롤러를 사용하여 그라운드 로버의 위치를 제어합니다.

Publishes `vehicle_thrust_setpoint (only in x) and vehicle_torque_setpoint (only yaw)` messages at IMU_GYRO_RATEMAX.

### 구현

현재 이 구현은 일부 모드만 지원합니다.

- 완전 수동: 스로틀 및 편요각 제어가 액츄에이터에 직접 전달됩니다.
- 자동 미션: 로버가 미션을 실행합니다.
- 배회: 로버가 배회 반경 내로 이동한 다음 모터를 중지합니다.

### 예

CLI 사용 예:

```
rover_pos_control start
rover_pos_control status
rover_pos_control stop
```

<a id="rover_pos_control_usage"></a>

### 사용법

```
rover_pos_control <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## spacecraft

Source: [modules/spacecraft](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/spacecraft)

```
### Description
This implements control allocation for spacecraft vehicles.
It takes torque and thrust setpoints as inputs and outputs
actuator setpoint messages.
```

<a id="spacecraft_usage"></a>

### 사용법

```
spacecraft <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## uuv_att_control

Source: [modules/uuv_att_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/uuv_att_control)

### 설명

무인수중선(UUV)의 자세를 제어합니다.

Publishes `vehicle_thrust_setpont` and `vehicle_torque_setpoint` messages at a constant 250Hz.

### 구현

현재 이 구현은 일부 모드만 지원합니다.

- 완전 수동: 롤, 피치, 요 및 스로틀 컨트롤이 액추에이터에 직접 전달됩니다.
- 자동 임무: 무인수중선이 임무를 실행합니다.

### 예

CLI 사용 예:

```
uuv_att_control start
uuv_att_control status
uuv_att_control stop
```

<a id="uuv_att_control_usage"></a>

### 사용법

```
uuv_att_control <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## uuv_pos_control

Source: [modules/uuv_pos_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/uuv_pos_control)

### 설명

무인수중선(UUV)의 자세를 제어합니다.
Publishes `attitude_setpoint` messages.

### 구현

현재 이 구현은 일부 모드만 지원합니다.

- 완전 수동: 롤, 피치, 요 및 스로틀 컨트롤이 액추에이터에 직접 전달됩니다.
- 자동 임무: 무인수중선이 임무를 실행합니다.

### 예

CLI 사용 예:

```
uuv_pos_control start
uuv_pos_control status
uuv_pos_control stop
```

<a id="uuv_pos_control_usage"></a>

### 사용법

```
uuv_pos_control <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## vtol_att_control

Source: [modules/vtol_att_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/vtol_att_control)

### 설명

fw_att_control은 고정익 자세 컨트롤러입니다.

<a id="vtol_att_control_usage"></a>

### 사용법

```
vtol_att_control <command> [arguments...]
 Commands:

   stop

   status        print status info
```
