# VTOL Ice Shedding feature

Ice shedding is a feature that periodically spins unused motors in fixed-wing
flight, to break off any ice that is starting to build up in the motors while it
is still feasible to do so.

It is configured by two parameters:
 - `CA_ICE_PERIOD`: When it is 0, the feature is disabled, when it is above 0,
 it sets the duration of the ice shedding cycle in seconds. In each cycle, the
 rotors are spun for `CA_ICE_ON_TIME` seconds at a motor output of 0.01.
 - `CA_ICE_ON_TIME`: The duration for which the motors are spun during each
 cycle, in seconds.

:::warning
When enabling the feature on a new airframe, there is the risk of producing
torques that disturb the fixed-wing rate controller. To mitigate this risk:
 - Test it with very low `CA_ICE_ON_TIME` initially and slowly work your way up
 - Be prepared to take control and switch back to multicopter
:::
