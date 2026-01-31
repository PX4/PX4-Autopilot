# OneShot Сервоприводи та Регулятори Швидкості (Контролери Двигуна)

PX4 підтримує ESC OneShot 125 (тільки).
These are typically faster and more responsive than [PWM ESCs](../peripherals/pwm_escs_and_servo.md) but share the same wiring setup (all you need to do is set some different parameters)

:::info
[DShot](../peripherals/dshot.md) should always be used instead of OneShot where possible, as it is more responsive, more robust, does not required calibration, and may support telemetry.
Єдине обґрунтування для невикористання DShot - обмеження апаратного забезпечення (недостатньо доступних пінів DShot або використання ESC, який не підтримує DShot).
:::

## Загальний огляд

OneShot is essentially a version of [PWM](../peripherals/pwm_escs_and_servo.md) that can be, in theory, up to 8 times faster.

Як PWM, так і OneShot спілкуються за допомогою періодичного імпульсу, де ширина імпульсу вказує на бажаний рівень потужності.
Для ШІМ довжина імпульсу зазвичай коливається від 1000 мкс (нуль) до 2000 мкс (повна потужність), тоді як для OneShot 125 ширина імпульсів в 8 разів коротша, коливаючись від 125 мкс (нульова потужність) до 250 мкс (повна потужність).

Теоретична максимальна швидкість, з якою можуть бути відправлені імпульси, а отже, реакція, залежить від ширини найбільшого імпульсу.
Для PWM ця швидкість становить близько 500 Гц, тоді як для OneShot вона наближається до 4 кГц.
На практиці фактична максимальна частота для OneShot ESCs зазвичай становить від 1 кГц до 2 кГц, в залежності від використаного ESC.

## Установка

### Підключення

Wiring is exactly the same as for [PWM ESCs](../peripherals/pwm_escs_and_servo.md) (and dshot).

### Конфігурація PX4

To enable OneShot select the protocol for a group of outputs during [Actuator Configuration](../config/actuators.md).
Note that the output range values are set to values in the normal PWM range (nominally `1000` to `2000`).
Ці дані масштабуються внутрішньо для виведення відповідних ширин імпульсів для Oneshot.

Then perform [ESC Calibration](../advanced_config/esc_calibration.md).
