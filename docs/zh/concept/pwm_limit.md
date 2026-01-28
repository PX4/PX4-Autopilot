# PWM_limit 状态机

The`PWM_limit State Machine` controls PWM outputs as a function of pre-armed and armed inputs.
并且会在解锁指令发出后、飞机油门增加之前引入一个延时。

## 总览

**Inputs**

- 解锁（armed）模式：宣告允许执行危险的动作指令，如转动螺旋桨。
- pre-armed: asserted to enable benign behaviors such as moving control surfaces
- this input overrides the current state
- assertion of pre-armed immediately forces behavior of state ON, regardless of current state
- deassertion of pre-armed reverts behavior to current state

**States**

- INIT 和 OFF
  - pwm 输出设置为锁定状态的值。
- RAMP
  - pwm outputs ramp from disarmed values to min values.
- 打开
  - 根据实际控制量设定 pwm 的输出值。

## 状态转移图

![PWM Limit state machine diagram](../../assets/diagrams/pwm_limit_state_diagram.svg)
