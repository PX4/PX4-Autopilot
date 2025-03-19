# Differential Rovers

<Badge type="tip" text="main (planned for: PX4 v1.16+)" /> <Badge type="warning" text="Experimental" />

A differential rover's motion is controlled using a differential drive mechanism, where the left and right wheel speeds are adjusted independently to achieve the desired forward speed and yaw rate.
Forward motion is achieved by driving both wheels at the same speed in the same direction.
Rotation is achieved by driving the wheels at different speeds in opposite directions, allowing the rover to turn on the spot.

![Aion R1](../../assets/airframes/rover/aion_r1/r1_rover_no_bg.png)

See [Configuration/Tuning](../config_rover/differential.md) to set up your rover and [Drive Modes](../flight_modes_rover/differential.md) for the supported flight (aka drive) modes.