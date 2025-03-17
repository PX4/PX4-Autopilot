# Autogyro Frames

<LinkedBadge type="warning" text="Experimental" url="../airframes/#experimental-vehicles"/>

:::warning
Support for autogyro frames is [experimental](../airframes/index.md#experimental-vehicles).
Maintainer volunteers, [contribution](../contribute/index.md) of new features, new frame configurations, or other improvements would all be very welcome!
:::

An [Autogyro](https://en.wikipedia.org/wiki/Autogyro) is a type of [rotary-wing](https://en.wikipedia.org/wiki/Rotorcraft).
Compared to other designs it possesses following advantages:

- Ability to take off and land using only a very short runway (compared to an fixed-wing vehicle).
- High resistance to weather conditions, especially gusts of wind.
- Possession of a non-powered rotor enabling it to operate in an autorotation mode (one of the helicopter’s emergency modes as well).
  Consequently, it does not need to actively change a flight mode in case of failure (it does not need a parachute or other actively working devices).
  Its flight is thus inherently stable at all times.
- Absence of a [downwash](https://en.wikipedia.org/wiki/Downwash) during take-off or landing creating an unwanted swirling of dust.
- [Low lift-to-drag ratio](https://en.wikipedia.org/wiki/Lift-to-drag_ratio) that can be adjusted by construction parameters.
  This ability can be useful because while an unmanned autogyro cannot fly very far in the case of failure (as is the case of a conventional airplane), the flight is still safe and aircraft does not fall (as is typical for multicopter or helicopter).

## Supported Frames

PX4 supports several autogyro airframes.
The set of supported configurations can be seen in [Airframes Reference > Autogyro](../airframes/airframe_reference.md#autogyro).

### DIY Frames

This section contains build logs/instructions for assembling and configuring a number of Autogyro frames.

- [ThunderFly Auto-G2 (Holybro pix32)](../frames_autogyro/thunderfly_auto_g2.md)

### Complete Frames with PX4 Preinstalled

This section lists vehicles that are sold fully assembled and ready to fly (RTF), with PX4 installed.

- [ThunderFly TF-G2](https://www.thunderfly.cz/tf-g2.html)
