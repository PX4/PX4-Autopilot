# VTOL Ice Shedding feature

## 综述

Ice shedding is a feature that periodically spins unused motors in fixed-wing
flight, to break off any ice that is starting to build up in the motors while it
is still feasible to do so.

It is configured by the paramter `CA_ICE_PERIOD`. When it is 0, the feature is
disabled, when it is above 0, it sets the duration of the ice shedding cycle in
seconds. In each cycle, the rotors are spun for two seconds at a motor output of
0.01.

:::warning
When enabling the feature on a new airframe, there is the risk of producing
torques that disturb the fixed-wing rate controller. To mitigate this risk:

- Set your `PWM_MIN` values correctly, so that the motor output 0.01 actually
  produces 1% thrust
- Be prepared to take control and switch back to multicopter

:::
