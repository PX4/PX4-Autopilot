# Режим висоти (мультикоптер)

<img src="../../assets/site/difficulty_easy.png" title="Easy to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />&nbsp;<img src="../../assets/site/altitude_icon.svg" title="Altitude required (e.g. Baro, Rangefinder)" width="30px" />

_Altitude mode_ is a _relatively_ easy-to-fly RC mode in which roll and pitch sticks control vehicle movement in the left-right and forward-back directions (relative to the "front" of the vehicle), yaw stick controls rate of rotation over the horizontal plane, and throttle controls speed of ascent-descent.

When the sticks are released/centered the vehicle will level and maintain the current _altitude_.
Якщо апарат рухається в горизонтальній площині, він буде продовжувати рух до тих пір, поки будь-який імпульс не буде погашений опором вітру.
Якщо дме вітер, літак буде дрейфувати у напрямку вітру.

:::tip
_Altitude mode_ is the safest non-GPS manual mode for new fliers. It is just like [Stabilized](../flight_modes_mc/manual_stabilized.md) mode but additionally locks the vehicle altitude when the sticks are released.
:::

The diagram below shows the mode behaviour visually (for a [mode 2 transmitter](../getting_started/rc_transmitter_receiver.md#transmitter_modes)).

![Altitude Control MC - Mode2 RC Controller](../../assets/flight_modes/altitude_mc.png)

## Технічний підсумок

RC/manual mode like [Stabilized mode](../flight_modes_mc/manual_stabilized.md) but with _altitude stabilization_ (centred sticks level vehicle and hold it to fixed altitude).
Горизонтальне положення транспортного засобу може змінюватися через вплив вітру (або наявного імпульсу).

- Центровані палиці (в межах дедбенду):
  - Рівень RPY прикріплюється до транспортного засобу.
  - Дросель (~50%) утримує поточну висоту стабільно проти вітру.
- Зовнішній центр:
  - Палиці кочення/крену керують кут нахилу у відповідних орієнтаціях, що призводить до відповідного руху ліворуч-праворуч та вперед-назад.
  - Ручка дроселя керує швидкістю вгору/вниз з попередньо визначеною максимальною швидкістю (та швидкістю руху в інших осях).
  - Палиця крену контролює швидкість кутової ротації вище горизонтальної площини.
- Зліт:
  - Після посадки транспортний засіб злетить, якщо важіль керування газом підніметься вище 62.5% від повного діапазону (від низу).
- Висота зазвичай вимірюється за допомогою барометра, який може стати неточним в екстремальних погодних умовах.
  Транспортні засоби, які включають датчик LIDAR/дальнісний датчик, зможуть керувати висотою з більшою надійністю та точністю.
- Потрібен ручний ввід управління (наприклад, за допомогою пульта дистанційного керування, джойстика).
  - Крен, Тангаж: Допомога від автопілота для стабілізації польоту.
    Положення палиці RC відображає орієнтацію транспортного засобу.
  - Газ: Допомога від автопілота для утримання позиції проти вітру.
  - Курс: Допомога від автопілота для стабілізації швидкості польоту.
    Положення палиці RC відображає швидкість обертання транспортного засобу в цій орієнтації.

## Параметри

Режим впливає на наступні параметри:

| Параметр                                                                                                                                                                                        | Опис                                                                                                                                                                                                                                                                                                                                                                                                               |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="MPC_Z_VEL_MAX_UP"></a>[MPC_Z_VEL_MAX_UP](../advanced_config/parameter_reference.md#MPC_Z_VEL_MAX_UP) | Максимальна швидкість вертикального підйому. За замовчуванням: 3 м/с.                                                                                                                                                                                                                                                                                              |
| <a id="MPC_Z_VEL_MAX_DN"></a>[MPC_Z_VEL_MAX_DN](../advanced_config/parameter_reference.md#MPC_Z_VEL_MAX_DN) | Максимальна швидкість вертикального спуску. За замовчуванням: 1 m/s.                                                                                                                                                                                                                                                                                               |
| <a id="RCX_DZ"></a>`RCX_DZ`                                                                                                                                                                     | RC dead zone for channel X. The value of X for throttle will depend on the value of [RC_MAP_THROTTLE](../advanced_config/parameter_reference.md#RC_MAP_THROTTLE). For example, if the throttle is channel 4 then [RC4_DZ](../advanced_config/parameter_reference.md#RC4_DZ) specifies the deadzone. |
| <a id="MPC_xxx"></a>`MPC_XXXX`                                                                                                                                                                  | Більшість параметрів MPC_xxx впливають на поведінку польоту в цьому режимі (принаймні до певної міри). For example, [MPC_THR_HOVER](../advanced_config/parameter_reference.md#MPC_THR_HOVER) defines the thrust at which a vehicle will hover.                                                   |
