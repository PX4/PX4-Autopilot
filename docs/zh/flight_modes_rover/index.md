# Drive Modes

Flight modes (or more accurately "Drive modes" for ground vehicles) provide autopilot support to make it easier to manually drive the vehicle or to execute autonomous missions.

This section outlines all supported drive modes for [Rovers](../frames_rover/index.md).

For information on mapping RC control switches to specific modes see: [Basic Configuration > Flight Modes](../config/flight_mode.md).

:::warning
Selecting any other mode than those listed below will either stop the rover or can lead to undefined behaviour.
:::

## Manual Modes

| 模式                                              | 描述                                                                                                                                                                                                                                                                                                  |
| ----------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Manual](manual.md#manual-mode)                 | No autopilot support. User is responsible for keeping the rover on the desired course and maintaining speed and rate of turn.                                                                                                                                       |
| [Acro](manual.md#acro-mode)                     | + Maintains the yaw rate (feels more like driving a car than manual mode). <br>+ Allows maximum yaw rate to be limited (protects against roll over).                                                                          |
| [Stabilized](manual.md#stabilized-mode)         | + Maintains the yaw (significantly better at holding a straight line).                                                                                                                                                                                           |
| [Position](manual.md#position-mode)             | + Maintains the course (best mode for driving a straight line).<br>+ Maintains speed against disturbances, e.g. when driving up a hill.<br>+ Allows maximum speed to be limited.                 |
| [Manual Parking](manual.md#manual-parking-mode) | Reduced throttle for precise maneuvering in tight spaces. Differential and mecanum rovers are driven tank-style, using one stick per side. Ackermann vehicles are controlled in the same way as they are in manual mode, but with reduced throttle. |

## Auto Modes

| 模式                              | 描述                                                                                      |
| ------------------------------- | --------------------------------------------------------------------------------------- |
| [Mission](auto.md#mission-mode) | Automatic mode that causes the vehicle to execute a predefined mission. |
| [Return](auto.md#return-mode)   | Automatic mode that returns the vehicle to the launch position.         |

## Failsafe Fallback Sequence

If a rover loses navigation while in an autonomous or semi-autonomous mode that relies on it, such as losing GNSS while in **Position** mode, PX4 will fall back to the mode that maintains the most control given the (remaining) available sensor information.

The failure cascade sequence with a manual controller available is: **Position -> Stabilized -> Acro -> Manual**

If manual control input is not available, the fallback cascade continues to: **Hold -> RTL -> Disarm**

Note that rovers will not use unsupported multicopter modes, such as Altitude or Descend.
If the failsafe system attempts to force the rover into an unsupported `Descend` state, the rover will safely bypass termination and automatically fall back to the action defined by the `COM_POS_FS_ACT` parameter.
