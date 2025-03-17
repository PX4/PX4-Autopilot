# 모듈 참조: 시뮬레이션

## simulator_sih

Source: [modules/simulation/simulator_sih](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/simulator_sih)

### 설명

This module provides a simulator for quadrotors and fixed-wings running fully
inside the hardware autopilot.

This simulator subscribes to "actuator_outputs" which are the actuator pwm
signals given by the control allocation module.

이 시뮬레이터는 루프에 상태 추정기를 통합하기 위하여 실제 노이즈로 손상된 센서 신호를 게시합니다.

### 구현

시뮬레이터는 선형대수를 사용하여 운동 방정식을 구현합니다.
쿼터니언 표현은 태도에 사용됩니다.
적분에는 순방향 오일러가 사용됩니다.
대부분의 변수는 스택 오버플로를 피하기 위하여 .hpp 파일에서 전역으로 선언됩니다.

<a id="simulator_sih_usage"></a>

### 사용법

```
simulator_sih <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
