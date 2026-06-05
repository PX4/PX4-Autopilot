# Симулятори, що підтримуються спільнотою

Цей розділ містить інформацію про симуляції, що _підтримуються спільнотою_.

:::warning
These simulators are not maintained, tested, or supported, by the core development team.
Вони можуть працювати або не працювати з поточними версіями PX4.

Дивіться [Встановлення інструментарію](../dev_setup/dev_env.md) для інформації про середовища та інструменти, що підтримуються основною командою розробників.
:::

Інструменти мають різний рівень підтримки своїми спільнотами (деякі добре підтримують, інші - ні).
Питання про ці інструменти повинні порушуватися на [форумах для обговорення](../contribute/support.md#forums-and-chat)

| Симулятор                                 | Опис                                                                                                                                                                                                                                                                                                                                                                                                           |
| ----------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [FlightGear](../sim_flightgear/README.md) | <p>Симулятор який надає фізично та візуально реалістичні симуляції. Зокрема він може моделювати багато погодних умов, включаючи грози, сніг, дощ та град, а також може симулювати температурні режими та різні типи атмосферних течій. [Симуляція кількох засобів](../sim_flightgear/multi_vehicle.md) також підтримується.</p> <p><strong>Рухомі засоби, що підтримуються:</strong> Літак, Автожир, Ровер</p> |
| [JMAVSim](../sim_jmavsim/index.md)        | <p>A simple multirotor/quad simulator. This was previously part of the PX4 development toolchain but was removed in favour of [Gazebo](../sim_gazebo_gz/index.md).</p> <p><strong>Supported Vehicles:</strong> Quad</p>                                                                                                                                                                                        |
| [JSBSim](../sim_jsbsim/README.md)         | <p>Симулятор, який надає моделі просунутої динаміки польоту. Він може використовуватися для моделювання реалістичної динаміки польоту, заснованої на даних з аеродинамічної труби.</p> <p><strong>Рухомі засоби, що підтримуються:</strong> Літак, Квадрокоптер, Гексакоптер</p>                                                                                                                               |
| [AirSim](../sim_airsim/README.md)         | <p>Міжплатформовий симулятор який надає фізично та візуально реалістичні симуляції. This simulator is resource intensive, and requires a significantly more powerful computer than the other simulators described here.</p><p><strong>Supported Vehicles:</strong> Iris (MultiRotor model and a configuration for PX4 QuadRotor in the X configuration).</p>                                                   |

:::tip
[Gazebo](../sim_gazebo_gz/index.md) and [SIH](../sim_sih/index.md) are the officially supported simulators. See the [Simulation](index.md) page for more information.
:::
