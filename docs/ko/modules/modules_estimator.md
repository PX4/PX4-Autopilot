# 모듈 참조: 추정기

## AttitudeEstimatorQ

Source: [modules/attitude_estimator_q](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/attitude_estimator_q)

### 설명

자세 추정자 Q입니다.

<a id="AttitudeEstimatorQ_usage"></a>

### 사용법

```
AttitudeEstimatorQ <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## airspeed_estimator

Source: [modules/airspeed_selector](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/airspeed_selector)

### 설명

이 모듈은 표시(IAS), 보정(CAS), 실제 속도(TAS) 및 추정이 현재 유효하지 않은 경우와 기반 센서 판독값 또는 지상 속도에서 풍속을 뺀 경우 정보를 포함하는 단일 airspeed_validated 주제를 제공합니다.
다중 "원시" 속도 입력을 지원하는 이 모듈은 오류 감지시 자동으로 유효한 센서로 전환합니다. 고장 감지와 IAS에서 CAS까지의 축척 계수 추정을 위하여 여러 바람 추정기를 실행하고 이를 게시합니다.

<a id="airspeed_estimator_usage"></a>

### 사용법

```
airspeed_estimator <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## ekf2

Source: [modules/ekf2](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/ekf2)

### 설명

확장 칼만 필터를 사용한 태도 및 위치 추정기입니다. 멀티콥터와 고정익에 사용됩니다.

The documentation can be found on the [ECL/EKF Overview & Tuning](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html) page.

ekf2 can be started in replay mode (`-r`): in this mode, it does not access the system time, but only uses the
timestamps from the sensor topics.

<a id="ekf2_usage"></a>

### 사용법

```
ekf2 <command> [arguments...]
 Commands:
   start
     [-r]        Enable replay mode

   stop

   status        print status info
     [-v]        verbose (print all states and full covariance matrix)

   select_instance Request switch to new estimator instance
     <instance>  Specify desired estimator instance
```

## local_position_estimator

Source: [modules/local_position_estimator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/local_position_estimator)

### 설명

확장 칼만 필터를 사용한 태도 및 위치 추정기입니다.

<a id="local_position_estimator_usage"></a>

### 사용법

```
local_position_estimator <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## mc_hover_thrust_estimator

Source: [modules/mc_hover_thrust_estimator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_hover_thrust_estimator)

### 설명

<a id="mc_hover_thrust_estimator_usage"></a>

### 사용법

```
mc_hover_thrust_estimator <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
