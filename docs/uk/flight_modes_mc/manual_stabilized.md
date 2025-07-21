# Stabilized Mode (Multicopter)

<img src="../../assets/site/difficulty_medium.png" title="Medium difficulty to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />&nbsp;

The _Stabilized_ manual mode stabilizes and levels the multicopter when the RC control sticks are centred.
To move/fly the vehicle you move the sticks outside of the centre.

:::info
This mode is also enabled if you set the flight mode to _Manual_.
:::

When sticks are outside the centre, the roll and pitch sticks control the _angle_ of the vehicle (attitude) around the respective axes, the yaw stick controls the rate of rotation above the horizontal plane, and the throttle controls altitude/speed.

Як тільки ви відпустите ручки керування, вони повернуться до центральної мертвої зони.
Багатороторник вирівняється і зупиниться, як тільки палиці кочення та тангажу будуть в центрі.
The vehicle will then hover in place/maintain altitude - provided it is properly balanced, throttle is set appropriately (see [below](#params)), and no external forces are applied (e.g. wind).
Літальний апарат буде дрейфувати в напрямку будь-якого вітру, і вам доведеться керувати реостатом, щоб утримати висоту.

![MC Manual Flight](../../assets/flight_modes/stabilized_mc.png)

## Технічний опис

RC mode where centered sticks level vehicle.

:::info
[Altitude mode](../flight_modes_mc/altitude.md) additionally stabilizes the vehicle altitude when sticks are centred, and [Position mode](../flight_modes_mc/position.md) stabilizes both altitude and position over ground.
:::

Команди пілота передаються як команди кутів крену та тангажу, а рискання як команда швидкості.
Throttle is rescaled (see [below](#params)) and passed directly to control allocation.
Автопілот контролює положення, це означає що він регулює кути крену та тангажу до нуля коли органи керування пульту РК центровані всередині мертвої зони контролера (як наслідок вирівнюючи положення).
Автопілот не компенсує дрейф через вітер (або інші джерела).

- Центровані палиці (в межах дедбенду):
  - Рівень ковзання/крена прикріплюється до транспортного засобу.
- Зовнішній центр:
  - Палиці кочення/крену керують кутом нахилу у цих орієнтаціях, що призводить до відповідного руху ліворуч-праворуч та вперед-назад.
  - Ручка дроселя керує швидкістю вгору/вниз (та швидкістю руху в інших осях).
  - Палиця крену контролює швидкість кутової ротації вище горизонтальної площини.
- Потрібен ручний ввід управління (наприклад, за допомогою пульта дистанційного керування, джойстика).
  - Крен, Тангаж: Допомога від автопілота для стабілізації польоту.
    Положення палиці RC відображає орієнтацію транспортного засобу.
  - Дросель: Ручне керування за допомогою палиць RC. RC ввід передається напряму до розподілу керування.
  - Курс: Допомога від автопілота для стабілізації швидкості польоту.
    Положення палиці RC відображає швидкість обертання транспортного засобу в цій орієнтації.

<a id="params"></a>

## Параметри

| Параметр                                                                                                                                     | Опис                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| -------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="MPC_THR_HOVER"></a>[MPC_THR_HOVER](../advanced_config/parameter_reference.md#MPC_THR_HOVER) | Hover throttle that is output when the throttle stick is centered and `MPC_THR_CURVE` is set to default.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| <a id="MPC_THR_CURVE"></a>[MPC_THR_CURVE](../advanced_config/parameter_reference.md#MPC_THR_CURVE) | Визначає масштаб ручки газу. By default this is set to **Rescale to hover thrust**, which means that when the throttle stick is centered the configured hover throttle is output (`MPC_THR_HOVER`) and the stick input is linearly rescaled below and above that (allowing for a smooth transition between Stabilized and Altitude/Position control). <br>На потужних транспортних засобах керування порогу може бути дуже низьким (наприклад, нижче 20%), тому що масштабування спотворює вхід газу - тобто тут 80% тяги керується лише верхньою половиною входу палиці, а 20% - нижнім. If needed `MPC_THR_CURVE` can be set to **No Rescale** so that there is no rescaling (stick input to throttle mapping is independent of `MPC_THR_HOVER`). |
