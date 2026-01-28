# Налаштування оцінки батареї (Налаштування енергії)

Ця тема пояснює, як налаштувати параметри живлення так, щоб PX4 міг оцінити наявну ємність акумулятора.

:::info
These instructions require that the vehicle has a [Power Module (PM)](../power_module/index.md), or other hardware that can measure the battery voltage and (optionally) the current.
:::

:::tip
This tuning is not needed for [Smart/MAVLink Batteries](../smart_batteries/index.md): batteries that can supply a reliable indication of remaining charge.
:::

## Загальний огляд

Налаштування оцінки батареї використовує виміряну напругу та силу струму (якщо доступно) для оцінки залишкової ємності батареї.
Це важливо, оскільки це дозволяє PX4 діяти, коли транспортний засіб майже розряджений і збирається врізатися (а також запобігає пошкодженню акумулятора внаслідок глибокого розряду).

PX4 надає ряд (поступово більш ефективних) методів, які можна використовувати для оцінки місткості:

1. [Basic Battery Settings](#basic_settings) (default): raw measured voltage is compared to the range between "empty" and "full" voltages.
   Це призводить до грубих оцінок, оскільки виміряна напруга (і відповідна ємність) буде коливатися під навантаженням.
2. [Voltage-based Estimation with Load Compensation](#load_compensation): Counteracts the effects of loading on the capacity calculation.
3. [Voltage-based Estimation with Current Integration](#current_integration): Fuses the load-compensated voltage-based estimate for the available capacity with a current-based estimate of the charge that has been consumed.
   Це призводить до оцінки потужності, яка є порівнянною з потужністю розумної батареї.

Пізніші методи ґрунтуються на попередніх методах.
Підхід, який ви використовуєте, буде залежати від того, чи може блок живлення автомобіля вимірювати струм.

:::info
The instructions below refer to battery 1 calibration parameters: `BAT1_*`.
Other batteries use the `BATx_*` parameters, where `x` is the battery number.
All battery calibration parameters [are listed here](../advanced_config/parameter_reference.md#battery-calibration).
:::

:::tip
In addition to PX4 configuration discussed here, you should ensure that the ESC's low voltage cutoff is either disabled or set below the expected minimum voltage.
Це забезпечує, що поведінка аварійного відключення батареї керується PX4, і регулятори швидкості не відключатимуться, поки батарея все ще має заряд (згідно з налаштуванням "порожньої батареї", яке ви обрали).
:::

:::tip
[Battery Chemistry Overview](../power_systems/battery_chemistry.md) explains the difference between the main battery types, and how that impacts the battery settings.
:::

## Basic Battery Settings (default) {#basic_settings}

Основні налаштування батареї налаштовують PX4 на використання типового методу для оцінки ємності.
Цей метод порівнює виміряну сирову напругу акумулятора з діапазоном між напругами акумуляторів для "порожніх" та "повних" акумуляторів (масштабований за кількістю акумуляторів).

:::info
This approach results in relatively coarse estimations due to fluctuations in the estimated charge as the measured voltage changes under load.
:::

Для налаштування основних параметрів для акумулятора 1:

1. Start _QGroundControl_ and connect the vehicle.
2. Select **"Q" icon > Vehicle Setup > Power** (sidebar) to open _Power Setup_.

Вам пропонується базові налаштування, які характеризують акумулятор.
Розділи нижче пояснюють, які значення встановити для кожного поля.

![QGC Power Setup](../../assets/qgc/setup/power/qgc_setup_power_px4.png)

:::info
At time of writing _QGroundControl_ only allows you to set values for battery 1 in this view.
For vehicles with multiple batteries you'll need to directly [set the parameters](../advanced_config/parameters.md) for battery 2 (`BAT2_*`), as described in the following sections.
:::

### Кількість елементів (в серії)

Це встановлює кількість акумуляторів, які з'єднані послідовно в батареї.
Зазвичай це буде записано на батареї у вигляді числа, за яким слідує "S" (наприклад, "3S", "5S").

:::info
The voltage across a single galvanic battery cell is dependent on the [chemical properties of the battery type](../power_systems/battery_chemistry.md).
Lithium-Polymer (LiPo) batteries and Lithium-Ion batteries both have the same _nominal_ cell voltage of 3.7V.
In order to achieve higher voltages (which will more efficiently power a vehicle), multiple cells are connected in _series_.
Напруга батареї на зажимах потім є кратною напрузі акумуляторної клітини.
:::

Якщо кількість акумуляторних елементів не вказана, ви можете розрахувати її, поділивши напругу батареї на номінальну напругу для одного елемента.
Таблиця нижче показує відношення напруги до акумуляторів:

| Cells | LiPo (V) | LiIon (V) |
| ----- | --------------------------- | ---------------------------- |
| 1S    | 3.7         | 3.7          |
| 2S    | 7.4         | 7.4          |
| 3S    | 11.1        | 11.1         |
| 4S    | 14.8        | 14.8         |
| 5S    | 18.5        | 18.5         |
| 6S    | 22.2        | 22.2         |

:::info
This setting corresponds to [parameters](../advanced_config/parameters.md): [BAT1_N_CELLS](../advanced_config/parameter_reference.md#BAT1_N_CELLS) and [BAT2_N_CELLS](../advanced_config/parameter_reference.md#BAT2_N_CELLS).
:::

### Повний напруга (на одну клітину)

This sets the _nominal_ maximum voltage of each cell (the lowest voltage at which the cell will be considered "full").

Значення повинно бути трохи нижче номінальної максимальної напруги акумулятора, але не так низько, щоб оцінювана ємність все ще була 100% після кількох хвилин польоту.

Відповідні значення для використання:

- **LiPo:** 4.05V (default in _QGroundControl_)
- **LiIon:** 4.05V

:::info
The voltage of a full battery may drop a small amount over time after charging.
Встановлення значення трохи нижче максимального компенсує це зниження.
:::

:::info
This setting corresponds to [parameters](../advanced_config/parameters.md): [BAT1_V_CHARGED](../advanced_config/parameter_reference.md#BAT1_V_CHARGED) and [BAT2_V_CHARGED](../advanced_config/parameter_reference.md#BAT2_V_CHARGED).
:::

### Порожній напруга (на одну клітину)

Це встановлює номінальну мінімально безпечну напругу кожної клітини (використання напруги нижче цієї може пошкодити батарею).

:::info
There is no single value at which a battery is said to be empty.
Якщо ви виберете занадто низьке значення, акумулятор може бути пошкоджений через глибоке розряджання (і/або транспортний засіб може зазнати аварії).
Якщо ви виберете значення, яке є занадто високим, ви можете непотрібно обмежити свій політ.
:::

Правило великого пальця для мінімальних напруг на одну клітину:

| Рівень                                                                            | LiPo (V) | LiIon (V) |
| --------------------------------------------------------------------------------- | --------------------------- | ---------------------------- |
| Консервативний (напруга без навантаження)                      | 3.7         | 3                            |
| "Реальний" мінімум (напруга під навантаженням/під час польоту) | 3.5         | 2.7          |
| Пошкодження акумулятора (напруга під навантаженням)            | 3.0         | 2.5          |

:::tip
Below the conservative range, the sooner you recharge the battery the better - it will last longer and lose capacity slower.
:::

:::info
This setting corresponds to [parameter](../advanced_config/parameters.md): [BAT1_V_EMPTY](../advanced_config/parameter_reference.md#BAT1_V_EMPTY) and [BAT2_V_EMPTY](../advanced_config/parameter_reference.md#BAT2_V_EMPTY).
:::

### Роздільник напруги

If you have a vehicle that measures voltage through a power module and the ADC of the flight controller then you should calibrate the measurements once per power module.
To calibrate, the actual voltage from the battery is measured (using a multimeter) and compared to the value provided by the power module.
This is used to calculate a "voltage divider" value, which can subsequently be used to scale the power module measurement to the correct value.

The easiest way to perform this calibration is by using _QGroundControl_ and following the step-by-step guide on [Setup > Power Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/power.html) (QGroundControl User Guide).

:::info
This setting corresponds to parameters: [BAT1_V_DIV](../advanced_config/parameter_reference.md#BAT1_V_DIV) and [BAT2_V_DIV](../advanced_config/parameter_reference.md#BAT2_V_DIV).
:::

### Amps per volt {#current_divider}

:::tip
This calibration is not needed if your power module does not provide current measurements.
:::

Current measurements are used (by default) for [Load Compensation](#load_compensation) and [Current Integration](#current_integration) if provided by the power module.
The amps per volt divider must be calibrated to ensure an accurate current measurement.

The easiest way to calibrate the dividers is by using _QGroundControl_ and following the step-by-step guide on [Setup > Power Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/power.html) (QGroundControl User Guide).

:::info
This setting corresponds to parameter(s): [BAT1_A_PER_V](../advanced_config/parameter_reference.md#BAT1_A_PER_V) and [BAT2_A_PER_V](../advanced_config/parameter_reference.md#BAT2_A_PER_V).
:::

## Voltage-based Estimation with Load Compensation {#load_compensation}

When a current flows through a battery, the internal resistance causes a voltage drop, reducing the measured output voltage of the battery compared to its open-circuit (no-load) voltage.
When using the [basic configuration](#basic_settings), the measured output voltage is what is used to estimate the available capacity, which means that the battery level will appear to fluctuate when you fly up and down, or otherwise change the load on the battery.

_Load compensation_ uses a measured or estimated value for the internal resistance to correct for changes under load, resulting in far less variation in the estimated capacity when flying.
This is enabled by default when using a power module that provides current measurements.

To use the load compensation first set the [basic configuration](#basic_settings).
The _Empty Voltage_ ([BATn_V_EMPTY](../advanced_config/parameter_reference.md#BAT1_V_EMPTY), where `n` is the battery number) should be set higher (than without compensation) because the compensated voltage gets used for the estimation (typically set a bit below the expected rest cell voltage when empty after use).

You will then need to calibrate the [Amps per volt divider](#current_divider) in the basic settings screen (in order to get reliable current measurements).

PX4 uses current-based load compensation based on a _real-time estimate_ of the internal resistance of the battery by default (real time estimates are enabled if [BAT1_R_INTERNAL=-1](../advanced_config/parameter_reference.md#BAT1_R_INTERNAL)).
Using a real time estimate allows the compensation to adapt to changes in the internal resistance of the battery due to temperature changes during flight, as well as over time as the battery degrades.

The internal resistance can also be measured and [set manually](../advanced_config/parameters.md) in [BAT1_R_INTERNAL](../advanced_config/parameter_reference.md#BAT1_R_INTERNAL).
A positive value in this parameter will be used for the internal resistance instead of the estimated value (`0` disables load compensation altogether).

:::info
There are LiPo chargers that can measure the internal resistance of your battery.
A typical value for LiPo batteries is 5mΩ per cell but this can vary with discharge current rating, age and health of the cells.
:::

## Voltage-based Estimation Fused with Current Integration {#current_integration}

This method is the most accurate way to measure relative battery consumption.
Якщо все налаштовано правильно з здоровою та свіжою зарядженою батареєю при кожному запуску, тоді якість оцінки буде порівнянною з тим, що від розумної батареї (і теоретично дозволить точну оцінку залишкового часу польоту).

The method evaluates the remaining battery capacity by _fusing_ the voltage-based estimate for the available capacity with a current-based estimate of the charge that has been consumed.
Для цього потрібне обладнання, яке може точно вимірювати поточний стан.

Щоб увімкнути цю функцію:

1. First set up accurate voltage estimation using [load compensation](#load_compensation).

   :::tip
   Including calibrating the [Amps per volt divider](#current_divider) setting.

:::

2. Set the parameter [BAT1_CAPACITY](../advanced_config/parameter_reference.md#BAT1_CAPACITY) to around 90% of the advertised battery capacity (usually printed on the battery label).

   ::: info
   Do not set this value too high as this may result in a poor estimation or sudden drops in estimated capacity.

:::

---

**Additional information**

Оцінка заряду, який був використаний протягом часу, формується математичним інтегруванням виміряного струму (цей підхід забезпечує дуже точні оцінки споживання енергії).

При запуску системи PX4 спочатку використовує оцінку на основі напруги для визначення початкового заряду батареї. Далі цю оцінку поєднується зі значенням з поточної інтеграції, щоб надати поєднану кращу оцінку.
Відносна вартість, яку відводиться кожній оцінці в отриманому об'єднаному результаті, залежить від стану акумулятора.
Чим порожніша батарея, тим більше напругової оцінки зливається. Це запобігає глибокому розряду (наприклад, через те, що воно було налаштовано з неправильною ємністю або початкове значення було неправильним).

Якщо ви завжди починаєте з здоровою повною батареєю, цей підхід схожий на той, який використовується смарт-батареєю.

:::info
Current integration cannot be used on its own (without voltage-based estimation) because it has no way to determine the _initial_ capacity.
Оцінка напруги дозволяє оцінити початкову ємність і надає постійний зворотний зв'язок щодо можливих помилок (наприклад, якщо акумулятор несправний або якщо є неспівпадіння між ємністю, розрахованою за допомогою різних методів).
:::
