# Вибір політного контролера

Політний контролер - це "мізки" безпілотного апарату.
PX4 can run on [many flight controller boards](../flight_controller/index.md).

Вам слід обирати плату, яка відповідає фізичним обмеженням вашого апарату, видам діяльності, які ви хочете виконувати та, звичайно, за вартістю.

<img src="../../assets/flight_controller/pixhawk6x/pixhawk6x_hero_upright.png" width="120px" title="Holybro Pixhawk6X"><img src="../../assets/flight_controller/cuav_pixhawk_v6x/pixhawk_v6x.jpg" width="200px" title="CUAV Pixhawk 6X" ><img src="../../assets/flight_controller/cube/orange/cube_orange_hero.jpg" width="250px" title="CubePilot Cube Orange" />

## Pixhawk Series

[Pixhawk Series](../flight_controller/pixhawk_series.md) open-hardware flight controllers run PX4 on NuttX OS.
With many form factors, there are versions targeted towards many use cases and market segments.

[Pixhawk Standard Autopilots](../flight_controller/autopilot_pixhawk_standard.md) are used as the PX4 reference platform.
They are supported and tested by the PX4 development team, and are highly recommended.

## Контролери від виробників

Other flight controllers are [manufacturer-supported](../flight_controller/autopilot_manufacturer_supported.md).
This includes FCs that are heavily based on the Pixhawk standard (but which are not fully compliant), and many others.

Note that manufacturer-supported controllers can be just as "good" (or better) than those which are Pixhawk-standard.

## Автопілоти для завдань з інтенсивними обчисленнями

Dedicated flight controllers like Pixhawk are not usually well-suited for general purpose computing or running computationally intensive tasks.
For more computing power, the most common approach is to run those applications on a separate onboard [Companion Computer](../companion_computer/index.md).
Some companion computers can also run PX4 on a separate DSP, as part of the same autopilot board.

Similarly, PX4 can also run natively Raspberry Pi (this approach is not generally considered as "robust" as having a separate companion or using a dedicated DSP):

- [Raspberry Pi 2/3 Navio2](../flight_controller/raspberry_pi_navio2.md)
- [Raspberry Pi 2/3/4 PilotPi Shield](../flight_controller/raspberry_pi_pilotpi.md)

## Комерційні БПЛА, які можуть працювати з PX4

PX4 is available on many popular commercial drone products, including some that ship with PX4 and others that can be updated with PX4 (allowing you to add mission planning and other PX4 Flight modes to your vehicle).

For more information see [Complete Vehicles](../complete_vehicles/index.md).

