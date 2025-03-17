# Ackermann Rovers

<Badge type="tip" text="main (planned for: PX4 v1.16+)" /> <Badge type="warning" text="Experimental" />

An _Ackermann rover_ controls its direction by pointing the front wheels in the direction of travel — the [Ackermann steering geometry](https://en.wikipedia.org/wiki/Ackermann_steering_geometry) compensates for the fact that wheels on the inside and outside of the turn move at different rates.
This kind of steering is used on most commercial vehicles, including cars, trucks etc.

::: info
PX4 does not require that the vehicle uses the Ackermann geometry and will work with any front-steering rover.
:::

![Axial Trail Honcho](../../assets/airframes/rover/rover_ackermann/axial_trail_honcho.png)

See [Configuration/Tuning](../config_rover/ackermann.md) to set up your rover and [Drive Modes](../flight_modes_rover/ackermann.md) for the supported flight (aka drive) modes.