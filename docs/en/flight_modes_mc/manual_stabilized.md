# Stabilized Mode (Multicopter)

<img src="../../assets/site/difficulty_medium.png" title="Medium difficulty to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />&nbsp;

The _Stabilized_ manual mode stabilizes and levels the multicopter when the RC control sticks are centred.
To move/fly the vehicle you move the sticks outside of the centre.

::: info
This mode is also enabled if you set the flight mode to _Manual_.
:::

When sticks are outside the centre, the roll and pitch sticks control the _angle_ of the vehicle (attitude) around the respective axes, the yaw stick controls the rate of rotation above the horizontal plane, and the throttle controls altitude/speed.

As soon as you release the control sticks they will return to the center deadzone.
The multicopter will level out and stop once the roll and pitch sticks are centered.
The vehicle will then hover in place/maintain altitude - provided it is properly balanced, throttle is set appropriately (see [below](#params)), and no external forces are applied (e.g. wind).
The craft will drift in the direction of any wind and you have to control the throttle to hold altitude.

![MC Manual Flight](../../assets/flight_modes/stabilized_mc.png)

## Technical Description

RC mode where centered sticks level vehicle.

::: info
[Altitude mode](../flight_modes_mc/altitude.md) additionally stabilizes the vehicle altitude when sticks are centred, and [Position mode](../flight_modes_mc/position.md) stabilizes both altitude and position over ground.
:::

The pilot's inputs are passed as roll and pitch angle commands and a yaw rate command.
Throttle is rescaled (see [below](#params)) and passed directly to control allocation.
The autopilot controls the attitude, meaning it regulates the roll and pitch angles to zero when the RC sticks are centered inside the controller deadzone (consequently leveling-out the attitude).
The autopilot does not compensate for drift due to wind (or other sources).

- Centered sticks (inside deadband):
  - Roll/Pitch sticks level vehicle.
- Outside center:
  - Roll/Pitch sticks control tilt angle in those orientations, resulting in corresponding left-right and forward-back movement.
  - Throttle stick controls up/down speed (and movement speed in other axes).
  - Yaw stick controls rate of angular rotation above the horizontal plane.
- Manual control input is required (such as RC control, joystick).
  - Roll, Pitch: Assistance from autopilot to stabilize the attitude.
    Position of RC stick maps to the orientation of vehicle.
  - Throttle: Manual control via RC sticks. RC input is sent directly to control allocation.
  - Yaw: Assistance from autopilot to stabilize the attitude rate.
    Position of RC stick maps to the rate of rotation of vehicle in that orientation.

<a id="params"></a>

## Parameters

| Parameter                                                                                          | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| -------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="MPC_THR_HOVER"></a>[MPC_THR_HOVER](../advanced_config/parameter_reference.md#MPC_THR_HOVER) | Hover throttle that is output when the throttle stick is centered and `MPC_THR_CURVE` is set to default.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| <a id="MPC_THR_CURVE"></a>[MPC_THR_CURVE](../advanced_config/parameter_reference.md#MPC_THR_CURVE) | Defines the throttle scaling. By default this is set to **Rescale to hover thrust**, which means that when the throttle stick is centered the configured hover throttle is output (`MPC_THR_HOVER`) and the stick input is linearly rescaled below and above that (allowing for a smooth transition between Stabilized and Altitude/Position control). <br>On powerful vehicles the hover throttle might be very low (e.g. below 20%) so that rescaling distorts the throttle input - i.e. here 80% of the thrust would be controlled by just the top half of the stick input and 20% by the bottom. If needed `MPC_THR_CURVE` can be set to **No Rescale** so that there is no rescaling (stick input to throttle mapping is independent of `MPC_THR_HOVER`). |
