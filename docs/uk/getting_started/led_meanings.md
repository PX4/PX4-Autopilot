# Значення LED індикаторів (Pixhawk Series)

[Pixhawk-series flight controllers](../flight_controller/pixhawk_series.md) use LEDs to indicate the current status of the vehicle.

- The [UI LED](#ui_led) provides user-facing status information related to _readiness for flight_.
- The [Status LEDs](#status_led) provide status for the PX4IO and FMU SoC.
  Вони показують заряд, режим і стан завантажувача, а також помилки.

<a id="ui_led"></a>

## LED індикатори інтерфейсу

The RGB _UI LED_ indicates the current _readiness for flight_ status of the vehicle.
Зазвичай це дуже яскравий I2C, який може бути встановлений або не встановлений на платі політного контролера (наприклад. FMUv4 не має такого на борту, і зазвичай використовує світлодіод, встановлений на GPS).

На зображенні нижче показано взаємозв'язок між світлодіодом і станом апарату.

:::warning
It is possible to have a GPS lock (Green LED) and still not be able to arm the vehicle because PX4 has not yet [passed preflight checks](../flying/pre_flight_checks.md). **A valid global position estimate is required to takeoff!**
:::

:::tip
In the event of an error (blinking red), or if the vehicle can't achieve GPS lock (change from blue to green),   check for more detailed status information in _QGroundControl_ including calibration status, and errors messages reported by the [Preflight Checks (Internal)](../flying/pre_flight_checks.md).
Також перевірте, чи правильно підключений GPS-модуль, чи правильно Pixhawk зчитує ваш GPS, і чи правильно GPS передає дані про місцеперебування.
:::

![LED meanings](../../assets/flight_controller/pixhawk_led_meanings.gif)

- **[Solid Blue] Armed, No GPS Lock:** Indicates vehicle has been armed and has no position lock from a GPS unit.
  Коли апарат вмикається, PX4 розблоковує керування моторами, що дозволить вам керувати дроном.
  Як завжди, будьте обережні під час увімкнення, оскільки великі гвинти можуть бути небезпечними на високих обертах.
  У цьому режимі апарат не може виконувати керовані місії.

- **[Pulsing Blue] Disarmed, No GPS Lock:** Similar to above, but your vehicle is disarmed.
  Це означає, що ви не зможете управляти моторами, але всі інші підсистеми працюють.

- **[Solid Green] Armed, GPS Lock:** Indicates vehicle has been armed and has a valid position lock from a GPS unit.
  Коли апарат вмикається, PX4 розблоковує керування моторами, що дозволить вам керувати дроном.
  Як завжди, будьте обережні під час увімкнення, оскільки великі гвинти можуть бути небезпечними на високих обертах.
  У цьому режимі апарат може виконувати керовані місії.

- **[Pulsing Green] Disarmed, GPS Lock:** Similar to above, but your vehicle is disarmed.
  Це означає, що ви не зможете управляти моторами, але всі інші підсистеми, включаючи GPS-фіксацію положення, працюють.

- **[Solid Purple] Failsafe Mode:** This mode will activate whenever vehicle encounters an issue during flight,
  such as losing manual control, a critically low battery, or an internal error.
  Під час аварійного режиму апарат намагатиметься повернутися до місця зльоту або може просто знизитися там, де він зараз перебуває.

- **[Solid Amber] Low Battery Warning:** Indicates your vehicle's battery is running dangerously low.
  Після певного моменту пристрій перейде у failsafe режим. Однак цей режим повинен сигналізувати про те, що настав час завершити політ.

- **[Blinking Red] Error / Setup Required:** Indicates that your autopilot needs to be configured or calibrated before flying.
  Під'єднайте автопілот до наземної станції керування, щоб перевірити, в чому проблема.
  Якщо ви завершили процес налаштування, а автопілот все ще сигналізує червоним і блимає, можливо, виникла інша помилка.

<a id="status_led"></a>

## Індикатор стану

Three _Status LEDs_ provide status for the FMU SoC, and three more provide status for the PX4IO (if present).
Вони показують заряд, режим і стан завантажувача, а також помилки.

![Pixhawk 4](../../assets/flight_controller/pixhawk4/pixhawk4_status_leds.jpg)

Після ввімкнення живлення процесори FMU та PX4IO спочатку запускають завантажувач (BL), а потім додаток (APP).
У таблиці нижче показано, як завантажувач, а потім додаток використовують світлодіоди для індикації стану.

| Колір                 | Позначка                                           | Використання завантажувача                | Використання додатку |
| --------------------- | -------------------------------------------------- | ----------------------------------------- | -------------------- |
| Синій                 | ACT (активність)                | Мерехтить, коли завантажувач отримує дані | Індикація стану ARM  |
| Червоний/помаранчевий | B/E (в завантажувачі / помилка) | Мерехтить, коли в завантажувачі           | Індикація ERROR      |
| Зелений               | PWR (потужність)                | Не використовується в завантажувачі       | Індикація стану ARM  |

:::info
The LED labels shown above are commonly used, but might differ on some boards.
:::

Більш детальна інформація про те, як інтерпретувати світлодіодні індикатори, наведені нижче (де "х" означає "будь-який стан")

| Червоний/помаранчевий | Синій | Зелений | Значення                                                                                                                                                                             |
| --------------------- | ----- | ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 10 Гц                 | x     | x       | Overload CPU load > 80%, or RAM usage > 98%                                                                                                                                          |
| ВИМК                  | x     | x       | Overload CPU load <= 80%, or RAM usage <= 98%                                                                                      |
| NA                    | ВИМК  | 4 Гц    | actuator_armed->armed && failsafe                                                                                       |
| NA                    | УВІМК | 4 Гц    | actuator_armed->armed && !failsafe                                                                                      |
| NA                    | ВИМК  | 1 Гц    | !actuator_armed-> armed && actuator_armed->ready_to_arm  |
| NA                    | ВИМК  | 10 Гц   | !actuator_armed->armed  && !actuator_armed->ready_to_arm |
