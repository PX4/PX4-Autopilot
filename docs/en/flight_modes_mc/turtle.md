# Turtle Mode (Multicopter)

Turtle mode lets the pilot recover an overturned Quad-X using selected motors in reverse.
Roll and pitch select the motors and control their thrust.
Centred sticks stop the motors.
Turtle can be armed in any orientation and does not stop automatically when the aircraft becomes upright.

## Requirements

- A fixed Quad-X using four vertical motors, with its normal motor geometry and assignments configured.
- All four motors on the same DShot driver, using non-3D ESC firmware that supports temporary direction commands 20 and 21.
- RC input, valid attitude estimation and the normal Commander arming checks.
- Firmware built with `CONFIG_MODE_TURTLE`.
- Throw launch disabled (`COM_THROW_EN=0`).

Enable `TRTL_EN` and reboot.
Assign Turtle to an RC flight-mode slot using `COM_FLTMODE1` through `COM_FLTMODE6` (value 18).
Set `TRTL_MAX` to limit recovery thrust and `TRTL_TIME` to limit each continuous attempt.
`TRTL_MAX` is a normalised thrust fraction, not a DShot command value.

## Pilot Sequence

1. Disarm, lower throttle fully, centre roll/pitch/yaw and select Turtle.
   Turtle does not require an inverted or tilted orientation.
2. Arm using the RC arm switch or button.
   Keep the sticks centred while the ESC direction changes.
3. Move roll or pitch towards the side to use for recovery.
   Cardinal stick directions select two motors; diagonals select one corner.
   Greater stick deflection increases thrust up to `TRTL_MAX`.
4. Centre roll and pitch to stop thrust.
   Move the sticks again for another attempt.
   If an attempt reaches `TRTL_TIME`, centre the sticks before retrying.
5. Once upright, centre the sticks to stop the motors.
   Holding a recovery stick continues motor output until it is centred, the attempt times out or recovery is interrupted.
   Disarm, select a normal flight mode and arm again after direction restoration completes.

All four ESCs receive the temporary reverse setting, but only the selected motors receive thrust.
The ESCs remain reversed between attempts.
Disarming restores their configured direction without saving ESC settings.

## Interruptions

Raised throttle, lost or invalid RC input, invalid attitude, a recovery-controller failure or a flight-mode failsafe stops recovery and requests disarming.
Leaving Turtle while armed also disarms before the normal flight controller takes over.
Actuator tests and normal rearming are blocked while direction restoration is in progress.

After an FC reset during recovery, disconnect and reconnect power to both the FC and ESCs before arming.

## Diagnostics

`listener turtle_control` reports mode readiness, recovery direction and thrust limit.
`listener actuator_motors` reports the resulting motor outputs.
`listener motor_direction_status` reports driver capability and direction-command progress.
A completed direction request means the driver has finished processing it.

Remove propellers for motor-mapping and direction checks.
