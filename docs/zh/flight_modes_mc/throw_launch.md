# Throw Launch (Multicopter)

<Badge type="tip" text="PX4 v1.15" /> <Badge type="warning" text="Experimental" />

:::warning
Experimental
This feature was introduced in PX4 v1.15.

- It has not yet been broadly tested on different vehicle configurations or scenarios.
- The majority of testing has been done in position mode.
  Other modes should also work.

:::

This feature allows a multicopter to be started by arming it from a fixed position and then throwing it into the air.
The vehicle turns on the motors only after the launch is detected, and then operates according to its current mode.

When throw launch is enabled, the vehicle is initially armed in a "lockdown" state, in which the propellers do not spin.
The propellors will not activate until the vehicle is thrown or is disarmed, and the arming tone will continue playing during this time.
The vehicle will not automatically disarm after arming, and must be manually disarmed if you choose not to throw it.

The vehicle detects that it has been thrown based on reaching a certain speed (5m/s), and then starts the motors at the apex of the throw (once it determines that it has started to fall).
You need to throw the vehicle high enough so that it can stabilize its height well before falling anywhere near people or obstacles.

备注：

- The mode is disabled by default, and must be enabled using a [parameter](#parameters) before arming.
- When enabled you cannot take off from the ground using the normal modes.
- The vehicle should not be transported after being armed and before the throw.
  In particular, the throw should not be executed from a moving platform.
  The reason for this is that the condition to start the motors depends on absolute speed of the multicopter and does not account for any additional movement.
  Trying to throw the drone from a moving platform might result in the motors being started prematurely.

## 安全

:::warning
Throw launch is dangerous as it requires the operator to hold an armed multicopter and be in proximity when it is flying.
:::

Before testing, make sure that the aircraft can take off with the normal position or takeoff modes.
Also ensure that the propellers do not spin on arming after enabling the feature.

In addition:

1. Wear safety equipment.
  Eye protection and work gloves are recommended.
2. Have an easily accessible and tested [kill switch](../config/safety.md#kill-switch).
  Remind the operator to be attentive and use the kill switch if needed.
  Pilots tend to forget that vehicles are replaceable, but they are not!
3. Test as much as possible without propellers.
  Keep the tools for removing propellers nearby/readily accessible.
4. Test this feature with at least two people — one handling the aircraft, the other one the remote control.
5. Keep in mind that after the throw, the exact behavior of the aircraft might be hard to predict as it depends heavily on the way it is thrown.
  Sometimes it will stay perfectly in place, but sometimes (e.g., due to extensive roll), it might drift to one side while stabilizing.
  Keep a safe distance!

On first flight of a new vehicle we recommend performing a [Throw Launch test without propellers](#throw-launch-pretest) (see below).

## Throw Launch Pretest

A throw launch without propellers can be used to confirm that arming does not occur prematurely, and for the operator to understand what to expect during the flight.

The steps for this test are:

1. Dismount the propellers.
2. Set [COM_THROW_EN](../advanced_config/parameter_reference.md#COM_THROW_EN) to `Enabled`.
3. Arm the aircraft.
  The engines should not spin, but the vehicle should be armed and keep playing the arming tune.
4. Throw the aircraft about 2m into the air.
  If the aircraft is not thrown high enough, the motors will not turn on.
5. The engines should start just after crossing the apex.
6. Engage the kill switch (ideally a second person operating the RC should do this).
7. Catch the drone.
  Remember to use safety gloves!

## Throw Launch

The steps for a throw launch are:

1. Set [COM_THROW_EN](../advanced_config/parameter_reference.md#COM_THROW_EN) to `Enabled`.
2. Arm the aircraft.
  The propellers should not spin, but the vehicle should be armed and keep playing the arming tune.
3. Throw the aircraft away from you, forward and up (about 2m away and 2m up is recommended).
  - The vehicle must reach the speed of [COM_THROW_SPEED](../advanced_config/parameter_reference.md#COM_THROW_SPEED) to detect launch, which by default is set to 5 m/s.
    If this speed is not achieved, the motors will not start and the aircraft will fall to the ground.
  - Try to avoid excessive rotation during the throw, as this might cause the drone to fail or behave unpredictably.
    The exact meaning of "excessive rotation" depends on the platform: for instance, [PX4Vision](../complete_vehicles_mc/px4_vision_kit.md) used for the testing, still managed to recover after 2-3 full rotations.
4. After a downward velocity is detected (the vehicle reaches its apex and starts falling down), the motors should turn on and the vehicle will start flying in the current mode.

## 参数

The following parameters can be used to enable and configure throw launch:

- [COM_THROW_EN](../advanced_config/parameter_reference.md#COM_THROW_EN) enables the feature.
- [COM_THROW_SPEED](../advanced_config/parameter_reference.md#COM_THROW_SPEED) determines the minimum speed the aircraft should reach to detect the throw.
  If it is not reached, the engines will not turn on.

## See Also

- [Takeoff Mode (Fixed-Wing) > Catapult/Hand Launch](../flight_modes_fw/takeoff.md#catapult-hand-launch).

<!--
Notes:
https://github.com/PX4/PX4-Autopilot/pull/23822
https://github.com/PX4/PX4-Autopilot/blob/371a99c3221dd09dce0b218c45df405188d96cfd/src/modules/commander/Commander.cpp#L1894-L1896 - lockdown setting
-->
