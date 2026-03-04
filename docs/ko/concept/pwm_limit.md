# PWM 제한 상태 머신

The`PWM_limit State Machine` controls PWM outputs as a function of pre-armed and armed inputs.
"armed"의 어설션과 준비된 신호의 어설션에 대한 스로틀 증가 사이의 지연을 제공합니다.

## 요약

**Inputs**

- armed: 회전하는 프로펠러와 같은 위험한 행동을 가능하게 한다고 주장
- pre-armed: asserted to enable benign behaviors such as moving control surfaces
- this input overrides the current state
- assertion of pre-armed immediately forces behavior of state ON, regardless of current state
- deassertion of pre-armed reverts behavior to current state

**States**

- INIT과 OFF
  - pwm 출력은 무장 해제된 값으로 설정됩니다.
- RAMP
  - pwm outputs ramp from disarmed values to min values.
- ON
  - pwm 출력은 제어 값에 따라 설정됩니다.

## 상태 전환 다이어그램

![PWM Limit state machine diagram](../../assets/diagrams/pwm_limit_state_diagram.svg)
