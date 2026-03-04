# VTOL з/без датчика швидкості повітря

<Badge type="warning" text="Experimental" />

:::warning
Support for VTOLs without an airspeed sensor is considered experimental and should only be attempted by experienced pilots.
Рекомендується використання датчика швидкості повітря.
:::

Fixed-wing vehicles use [airspeed sensors](../sensor/airspeed.md) to determine the speed at which the airplane is moving through the air.
Залежно від вітру це може відрізнятися від швидкості руху на землі.
У кожного літака є мінімальна швидкість повітря, нижче якої літак заходить в пікетаж.
В умовах сприятливої погоди та з налаштуваннями, значно вищими за мінімальну швидкість пікетажу, VTOL може працювати без використання датчика швидкості повітря.

Цей посібник описує параметри, необхідні для обходу датчика швидкості повітря для VTOL-літаків.

:::info
Most settings described here should also be applicable to fixed-wing vehicles that are not VTOL, but this is currently untested.
Поворот під час переходу та квадро-парашут - це специфічні для VTOL параметри.
:::

## Підготовка

Перш ніж спробувати вилучити датчик швидкості повітря, вам слід спочатку визначити безпечний рівень газу.
Також необхідно знати тривалість переднього переходу.
Для цього ви можете виконати референтний польот з датчиком швидкості повітря або пілотувати транспортний засіб вручну.
У обох випадках референтний польот слід виконати в умовах дуже слабкого вітру.

Польот повинен відбуватися зі швидкістю, достатньою для польоту в умовах сильного вітру, і повинен складатися з таких етапів:

- Успішний передній перехід
- Прямий та рівний польот
- Агресивний поворот
- Швидке підняття на вищу висоту

## Огляд журналу

After the reference flight download the log and use [FlightPlot](../log/flight_log_analysis.md#flightplot) (or another analysis tool) to examine the log.
Plot the altitude (`GPOS.Alt`), thrust (`ATC1.Thrust`), groundspeed (Expression: `sqrt(GPS.VelN\^2 + GPS.VelE\^2)`), pitch (`ATT.Pitch`) and roll (`AT.Roll`).

Перевірте рівень дросельної заслінки (тягу), коли транспортний засіб стоїть рівно (відсутній або невеликий нахил і крен), під час підйому (збільшення висоти) і коли автомобіль накрениться (більший крен).
Початковим значенням для використання в якості крейсерської швидкості має бути найвища тяга, застосована під час крену або підйому, тяга під час горизонтального польоту повинна вважатися мінімальним значенням, якщо ви вирішите надалі зменшити свою швидкість.

Також зверніть увагу на час, який знадобився для завершення переднього переходу.
Це буде використано для встановлення мінімального часу переходу.
З міркувань безпеки вам слід додати до цього часу +- 30%.

Нарешті, зверніть увагу на швидкість руху під час крейсерського польоту.
Це можна використовувати для налаштування дросельної заслінки після першого польоту без датчика швидкості.

## Встановлення параметрів

To bypass the airspeed preflight check you need to set [SYS_HAS_NUM_ASPD](../advanced_config/parameter_reference.md#SYS_HAS_NUM_ASPD) to 0.

To prevent an installed airspeed sensor being used for feedback control set [FW_USE_AIRSPD](../advanced_config/parameter_reference.md#FW_USE_AIRSPD) to `False`.
Це дозволить вам перевірити поведінку системи в умовах відсутності датчика швидкості повітря, зберігаючи при цьому фактичне значення швидкості повітря для перевірки безпеки відносно межі безпеки від відмивання та ін.

Set the trim throttle ([FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM)) to the percentage as determined from the log of the reference flight.
Note that QGC scales this from `1..100` and the thrust value from the log is scaled from `0..1`.
Таким чином, значення тяги 0.65 слід ввести як 65.
З міркувань безпеки рекомендується додати +- 10% газу до визначеного значення для тестування першого польоту.

Set the minimum front transition time ([VT_TRANS_MIN_TM](../advanced_config/parameter_reference.md#VT_TRANS_MIN_TM)) to the number of seconds determined from the reference flight and add +- 30% for safety.

### Додаткові рекомендовані параметри

Оскільки ризик звалювання є реальним, рекомендується встановлювати 'мінімальну висоту для фіксованого крила' (він же. 'quad-chute') threshold ([VT_FW_MIN_ALT](../advanced_config/parameter_reference.md#VT_FW_MIN_ALT)).

This will cause the VTOL to transition back to multicopter mode and initiate the [Return mode](../flight_modes_vtol/return.md) below a certain altitude.
Ви можете встановити значення 15 або 20 метрів, щоб дати мультикоптеру час відновитися після зупинки.

The position estimator tested for this mode is EKF2, which is enabled by default (for more information see [Switching State Estimators](../advanced/switching_state_estimators.md#how-to-enable-different-estimators) and [EKF2_EN ](../advanced_config/parameter_reference.md#EKF2_EN)).

## Перший польот без датчика швидкості повітря

The values apply to a position controlled flight (like [Hold mode](../flight_modes_fw/hold.md) or [Mission mode](../flight_modes_vtol/mission.md) or Mission mode).
Тому рекомендується налаштувати місію на безпечній висоті, приблизно на 10 метрів вище порогу квадратного парашуту.

Подібно до референтного польоту, цей польот слід виконувати в умовах дуже слабкого вітру.
Для першого польоту рекомендується:

- Залишайтеся на одній висоті
- Встановіть точки маршруту достатньо широко і так, щоб не потрібні були гострі повороти
- Зберіть місію достатньо малою, щоб вона залишалася на виду, якщо знадобиться ручне керування.
- Якщо швидкість повітря дуже велика, розгляньте можливість виконання ручного переходу назад, перемикаючи в режим висоти.

Якщо місія завершилася успішно, вам слід перевірити журнал наступного:

- Швидкість на землі повинна бути значно вище, ніж швидкість на землі під час референтного польоту.
- Висота не повинна бути значно нижчою, ніж під час референтного польоту.
- Кут крена не повинен постійно відрізнятися від референтного польоту.

Якщо всі ці умови виконані, ви можете почати поступово знижувати рівень газу круїзу до тих пір, поки швидкість на землі не відповідає тій, що була під час референтного польоту.

## Огляд параметрів

Відповідними параметрами є:

- [FW_USE_AIRSPD](../advanced_config/parameter_reference.md#FW_USE_AIRSPD)
- [SYS_HAS_NUM_ASPD](../advanced_config/parameter_reference.md#SYS_HAS_NUM_ASPD)
- [EKF2_EN](../advanced_config/parameter_reference.md#EKF2_EN) (1), [ATT_EN](../advanced_config/parameter_reference.md#ATT_EN) (0), [LPE_EN](../advanced_config/parameter_reference.md#LPE_EN) (0)
- [FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM): determined (e.g. 70%)
- [VT_TRANS_MIN_TM](../advanced_config/parameter_reference.md#VT_TRANS_MIN_TM): determined (e.g. 10 seconds)
- [VT_FW_MIN_ALT](../advanced_config/parameter_reference.md#VT_FW_MIN_ALT): 15
