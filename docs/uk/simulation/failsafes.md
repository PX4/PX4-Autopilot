# Симуляція запобігання відмовам

[Failsafes](../config/safety.md) define the safe limits/conditions under which you can safely use PX4, and the action that will be performed if a failsafe is triggered (for example, landing, holding position, or returning to a specified point).

У SITL деякі запобіжники відмов за замовчуванням вимкнені, щоб забезпечити простіше використання симуляції.
Ця тема пояснює, як ви можете перевірити критично важливу для безпеки поведінку в симуляції SITL перед тим, як спробувати її в реальному світі.

:::info
You can also test failsafes using [HITL simulation](../simulation/hitl.md).
HITL використовує нормальні параметри налаштувань вашого контролера польоту.
:::

## Втрата каналу зв'язку

The _Data Link Loss_ failsafe (unavailability of external data via MAVLink) is enabled by default.
Це робить симуляцію придатною до використання тільки з під'єднаним GCS, SDK або іншим додатком MAVLink.

Set the parameter [NAV_DLL_ACT](../advanced_config/parameter_reference.md#NAV_DLL_ACT) to the desired failsafe action to change the behavior.
For example, set to `0` to disable it.

:::info
All parameters in SITL including this one get reset when you do `make clean`.
:::

## Втрата каналу радіо керування

The _RC Link Loss_ failsafe (unavailability of data from a remote control) is enabled by default.
Це робить симуляцію придатною до використання тільки з активним з'єднанням MAVLink або дистанційного керування.

Set the parameter [NAV_RCL_ACT](../advanced_config/parameter_reference.md#NAV_RCL_ACT) to the desired failsafe action to change the behavior.
For example, set to `0` to disable it.

:::info
All parameters in SITL including this one get reset when you do `make clean`.
:::

## Низький заряд батареї

Батарею, що моделюється реалізовано таким чином щоб енергія ніколи не закінчувалась та за замовчуванням вона виснажується тільки до 50% її заряду, а отже і напруги, що доповідається.
Це дозволяє тестувати індикацію батареї в GCS Ui без спрацювання реакцій на низький заряд, що може перервати інші тести.

To change this minimal battery percentage value use the parameter [SIM_BAT_MIN_PCT](../advanced_config/parameter_reference.md#SIM_BAT_MIN_PCT).

To control how fast the battery depletes to the minimal value use the parameter [SIM_BAT_DRAIN](../advanced_config/parameter_reference.md#SIM_BAT_DRAIN).

:::tip
By changing [SIM_BAT_MIN_PCT](../advanced_config/parameter_reference.md#SIM_BAT_MIN_PCT) in flight, you can also test regaining capacity to simulate inaccurate battery state estimation or in-air charging technology.
:::

It is also possible to disable the simulated battery using [SIM_BAT_ENABLE](../advanced_config/parameter_reference.md#SIM_BAT_ENABLE) in order to, for example, provide an external battery simulation via MAVLink.

## Помилка датчику/системи

[Failure injection](../debug/failure_injection.md) can be used to simulate different types of failures in many sensors and systems.
Наприклад, це може бути використано для імітації відсутнього або переривчастого сигналу GPS, сигналу РК який перервався або застиг на певному значенні, збої в системі уникнення, і багато іншого.

Наприклад, для імітації відмови сигналу GPS:

1. Enable the parameter [SYS_FAILURE_EN](../advanced_config/parameter_reference.md#SYS_FAILURE_EN).
2. Enter the following commands on the SITL instance _pxh shell_:

   ```sh
   # Turn (all) GPS off
   failure gps off

   # Turn (all) GPS on
   failure gps ok
   ```

See [System Failure Injection](../debug/failure_injection.md) for a list of supported target sensors and failure modes.
