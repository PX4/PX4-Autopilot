# Конфігурація режиму польоту

This topic explains how to map [flight modes](../getting_started/px4_basic_concepts.md#flight-modes) and other functions to the switches on your radio control transmitter.

:::tip
In order to set up flight modes you must already have:

- [Configured your radio](../config/radio.md)
- [Setup your transmitter](#rc-transmitter-setup) to encode the physical positions of your mode switch(es) into a single channel.
  We provide examples for the popular _Taranis_ transmitter [below](#taranis-setup-3-way-switch-configuration-for-single-channel-mode) (check your documentation if you use a different transmitter).

:::

## Які режими польоту та перемикачі я повинен встановити?

Flight Modes provide different types of _autopilot-assisted flight_, and _fully autonomous flight_.
You can set any (or none) of the flight modes [available to your vehicle](../flight_modes/index.md#flight-modes).

Більшість користувачів повинні встановити наступні режими та функції, оскільки вони роблять літак більш легким та безпечним у польоті:

- **Position mode** — Easiest and safest mode for manual flight.
- **Return mode** — Return to launch position by safe path and land.
- (VTOL only) **VTOL Transition Switch** — Toggle between fixed-wing and multicopter flight configuration on VTOL vehicles.

Також звичайно відображати перемикачі на:

- **Mission mode** — This mode runs a pre-programmed mission sent by the ground control station.
- <a id="kill_switch"></a> [Kill Switch](../config/safety.md#kill-switch) - Immediately stops all motor outputs (the vehicle will crash, which may in some circumstances be more desirable than allowing it to continue flying).

## Вибір режиму польоту

PX4 дозволяє вам вказати канал "режиму" та вибрати до 6 режимів польоту, які будуть активовані на основі значення ШШІ каналу.
Ви також можете окремо вказати канали для відображення режиму аварійного вимкнення, режиму повернення до старту та режиму польоту за межами.

Для налаштування вибору режиму польоту з одним каналом:

1. Start _QGroundControl_ and connect the vehicle.

2. Увімкніть ваш передавач RC.

3. Select **"Q" icon > Vehicle Setup > Flight Modes** (sidebar) to open _Flight Modes Setup_.

  ![Flight modes single-channel](../../assets/qgc/setup/flight_modes/flight_modes_single_channel.jpg)

4. Specify _Flight Mode Settings_:
  - Select the **Mode channel** (above this shown as Channel 5, but this will depend on your transmitter configuration).
  - Перемістіть перемикач передавача (або перемикачі), які ви налаштували для вибору режиму, через доступні позиції.
    The mode slot matching your current switch position will be highlighted (above this is _Flight Mode 1_).
    ::: info
    While you can set flight modes in any of the 6 slots, only the channels that are mapped to switch positions will be highlighted/used.

:::
  - Виберіть режим польоту, який ви хочете активувати для кожного положення перемикача.

5. Specify _Switch Settings_:
  - Select the channels that you want to map to specific actions - e.g.: _Return_ mode, _Kill switch_, _offboard_ mode, etc. (if you have spare switches and channels on your transmitter).

6. Перевірте, що режими відображаються на правильні перемикачі передавача:
  - Check the _Channel Monitor_ to confirm that the expected channel is changed by each switch.
  - Select each mode switch on your transmitter in turn, and check that the desired flight mode is activated (the text turns yellow on _QGroundControl_ for the active mode).

Усі значення автоматично зберігаються після зміни.

## Налаштування радіопередавача

Цей розділ містить невелику кількість можливих налаштувань для таранісу.
QGroundControl _may_ have [setup information for other transmitters here](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/flight_modes.html#transmitter-setup).

<a id="taranis_setup"></a>

### Налаштування Taranis: Конфігурація перемикача 3-х позицій для одноканального режиму

Якщо вам потрібно підтримувати вибір лише між двома або трьома режимами, то ви можете відобразити режими на позиції одного 3-позиційного перемикача.
Нижче ми показуємо, як зіставити перемикач "SD" Taranis 3-way з каналом 5.

:::info
This example shows how to set up the popular _FrSky Taranis_ transmitter.
Налаштування передавача буде відрізнятися на інших передавачах.
:::

Open the Taranis UI **MIXER** page and scroll down to **CH5**, as shown below:

![Taranis - Map channel to switch](../../assets/qgc/setup/flight_modes/single_channel_mode_selection_1.png)

Press **ENT(ER)** to edit the **CH5** configuration then change the **Source** to be the _SD_ button.

![Taranis - Configure channel](../../assets/qgc/setup/flight_modes/single_channel_mode_selection_2.png)

Ось і все!
Channel 5 will now output 3 different PWM values for the three different **SD** switch positions.

The _QGroundControl_ configuration is then as described in the previous section.

### Налаштування Taranis: Конфігурація багато перемикачів для одноканального режиму

Більшість передавачів не мають перемикачів на 6 позицій, тому якщо вам потрібно мати можливість підтримувати більше режимів, ніж кількість доступних позицій перемикачів (до 6), то вам доведеться представляти їх за допомогою кількох перемикачів.
Зазвичай це робиться шляхом кодування позицій перемикача 2- та 3-позицій в один канал, так що кожна позиція перемикача призводить до різного значення ШІМ.

На FrSky Taranis цей процес включає в себе призначення "логічного перемикача" для кожної комбінації положень двох реальних перемикачів.
Кожний логічний перемикач потім призначається для різних значень ШІМ на тому ж каналі.

The video below shows how this is done with the _FrSky Taranis_ transmitter.

<!-- [youtube](https://youtu.be/scqO7vbH2jo) Video has gone private and is no longer available -->

<!-- @[youtube](https://youtu.be/BNzeVGD8IZI?t=427) - video showing how to set the QGC side - at about 7mins and 3 secs -->

<lite-youtube videoid="TFEjEQZqdVA" title="Taranis Mode Switches"/>

The _QGroundControl_ configuration is then [as described above](#flight-mode-selection).

## Подальша інформація

- [Flight Modes Overview](../flight_modes/index.md)
- [QGroundControl > Flight Modes](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/flight_modes.html#px4-pro-flight-mode-setup)
- [PX4 Setup Video - @6m53s](https://youtu.be/91VGmdSlbo4?t=6m53s) (Youtube)
- [Radio switch parameters](../advanced_config/parameter_reference.md#radio-switches) - Can be used to set mappings via parameters
