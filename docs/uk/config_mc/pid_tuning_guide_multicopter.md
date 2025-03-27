# Посібник з налаштування PID для багатокоптерів (Ручне/Продвинуте)

Ця тема надає докладну інформацію про регулятори PX4 та їх налаштування.

:::tip
[Autotune](../config/autotune_mc.md) is recommended for tuning the vehicles _around the hover thrust point_, as the approach described is intuitive, easy, and fast.
Цього достатньо для багатьох транспортних засобів.
:::

Використовуйте цю тему, коли налаштування навколо точки підняття на тягу не є достатнім (наприклад, на транспортних засобах, де є нелінійності та коливання при високій тяги).
It is also useful for a deeper understanding of how the basic tuning works, and to understand how to use the [airmode](#airmode-mixer-saturation) setting.

## Настройка крокування

:::info
For safety reasons, the default gains are set to low values.
Вам потрібно збільшити значення перед очікуванням гарної реакції управління.
:::

Ось деякі загальні рекомендації для настройки:

- Усі коефіцієнти крокування потрібно збільшувати дуже повільно, оскільки великі коефіцієнти можуть призвести до небезпечних коливань!
  Зазвичай збільшуйте коефіцієнти на 20-30% за кожною ітерацією, зменшуючи до 5-10% для кінцевої точної настройки.
- Перед зміною параметра приземліться.
  Повільно збільшуйте газ і перевіряйте наявність коливань.
- Tune the vehicle around the hovering thrust point, and use the [thrust curve parameter](#thrust-curve) to account for thrust non-linearities or high-thrust oscillations.
- Optionally enable the high-rate logging profile with the [SDLOG_PROFILE](../advanced_config/parameter_reference.md#SDLOG_PROFILE) parameter so you can use the log to evaluate the rate and attitude tracking performance (the option can be disabled afterwards).

:::warning
Always disable [MC_AIRMODE](../advanced_config/parameter_reference.md#MC_AIRMODE) when tuning a vehicle.
:::

### Контролер швидкості

The rate controller is the inner-most loop with three independent PID controllers to control the body rates (roll, pitch, yaw).

:::info
A well-tuned rate controller is very important as it affects _all_ flight modes.
A badly tuned rate controller will be visible in [Position mode](../flight_modes_mc/position.md), for example, as "twitches" or oscillations (the vehicle will not hold perfectly still in the air).
:::

#### Архітектура/Форма контролера швидкості

PX4 supports two (mathematically equivalent) forms of the PID rate controller in a single "mixed" implementation: [Parallel](#parallel-form) and [Standard](#standard-form).

Users can select the form that is used by setting the proportional gain for the other form to "1" (i.e. in the diagram below set **K** to 1 for the parallel form, or **P** to 1 for the standard form - this will replace either the K or P blocks with a line).

![PID_Mixed](../../assets/mc_pid_tuning/PID_algorithm_Mixed.png)

<!-- The drawing is on draw.io: https://drive.google.com/file/d/1hXnAJVRyqNAdcreqNa5W4PQFkYnzwgOO/view?usp=sharing -->

- _G(s)_ represents the angular rates dynamics of a vehicle
- _r_ is the rate setpoint
- _y_ is the body angular rate (measured by a gyro)
- _e_ is the error between the rate setpoint and the measured rate
- _u_ is the output of the PID controller

Ці дві форми описані нижче.

:::info
The derivative term (**D**) is on the feedback path in order to avoid an effect known as the [derivative kick](http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/).
:::

:::tip
Для додаткової інформації дивіться:

- [Not all PID controllers are the same](https://www.controleng.com/articles/not-all-pid-controllers-are-the-same/) (www.controleng.com)
- [PID controller > Standard versus parallel (ideal) PID form](https://en.wikipedia.org/wiki/PID_controller#Standard_versus_parallel_\(ideal\)_form) (Wikipedia)

:::

##### Паралельний порт

The _parallel form_ is the simplest form, and is (hence) commonly used in textbooks.
У цьому випадку вихід контролера просто є сумою пропорційних, інтегральних та диференційних дій.

![PID_Parallel](../../assets/mc_pid_tuning/PID_algorithm_Parallel.png)

##### Стандартна форма

Ця форма математично еквівалентна паралельній формі, але основною перевагою є те, що (навіть якщо це здається протирічливим) вона роз'єднує налаштування пропорційного коефіцієнта від інтегрального та диференціального коефіцієнтів.
Це означає, що нову платформу легко можна налаштувати, взявши вигоди від дрона зі схожим розміром/інерцією і просто налаштувати коефіцієнт K, щоб він летів належним чином.

![PID_Standard](../../assets/mc_pid_tuning/PID_algorithm_Standard.png)

#### Оцінити Налаштування PID

Пов'язані параметри для налаштування регуляторів швидкості PID є наступні:

- Roll rate control ([MC_ROLLRATE_P](../advanced_config/parameter_reference.md#MC_ROLLRATE_P), [MC_ROLLRATE_I](../advanced_config/parameter_reference.md#MC_ROLLRATE_I), [MC_ROLLRATE_D](../advanced_config/parameter_reference.md#MC_ROLLRATE_D), [MC_ROLLRATE_K](../advanced_config/parameter_reference.md#MC_ROLLRATE_K))
- Pitch rate control ([MC_PITCHRATE_P](../advanced_config/parameter_reference.md#MC_PITCHRATE_P), [MC_PITCHRATE_I](../advanced_config/parameter_reference.md#MC_PITCHRATE_I), [MC_PITCHRATE_D](../advanced_config/parameter_reference.md#MC_PITCHRATE_D), [MC_PITCHRATE_K](../advanced_config/parameter_reference.md#MC_PITCHRATE_K))
- Yaw rate control ([MC_YAWRATE_P](../advanced_config/parameter_reference.md#MC_YAWRATE_P), [MC_YAWRATE_I](../advanced_config/parameter_reference.md#MC_YAWRATE_I), [MC_YAWRATE_D](../advanced_config/parameter_reference.md#MC_YAWRATE_D), [MC_YAWRATE_K](../advanced_config/parameter_reference.md#MC_YAWRATE_K))

The rate controller can be tuned in [Acro mode](../flight_modes_mc/acro.md) or [Stabilized mode](../flight_modes_mc/manual_stabilized.md):

- _Acro mode_ is preferred because it allows for isolated rate control testing.
  However it is significantly harder to pilot.

  ::: warning
  If you choose this mode, you must [disable all stick expo and have reasonable maximum rates for all axes](../flight_modes_mc/acro.md#stick-input-mapping):

  - `MC_ACRO_EXPO` = 0, `MC_ACRO_EXPO_Y` = 0, `MC_ACRO_SUPEXPO` = 0,
    `MC_ACRO_SUPEXPOY` = 0
  - `MC_ACRO_P_MAX` = 200, `MC_ACRO_R_MAX` = 200
  - `MC_ACRO_Y_MAX` = 100

  For PX4 v1.15 and later the defaults are set for this purpose to a maximum rate of 100°/s linear mapping for all axes.

:::

- _Stabilized mode_ is simpler to fly, but it is also much more difficult to distinguish if attitude or rate controller causes a certain behavior.

Якщо транспортний засіб зовсім не літає:

- If there are strong oscillations when first trying to takeoff (to the point where it does not fly), decrease all **P** and **D** gains until it takes off.
- If the reaction to RC movement is minimal, increase the **P** gains.

The actual tuning is roughly the same in _Manual mode_ or _Acro mode_:
You iteratively tune the **P** and **D** gains for roll and pitch, and then the **I** gain.
Спочатку ви можете використовувати ті ж значення для крена та тангажу, і як тільки ви отримаєте хороші значення, ви можете їх налаштувати, розглядаючи відповідь на крен та тангаж окремо (якщо ваш транспортний засіб симетричний, це не потрібно).
For yaw it is very similar, except that **D** can be left at 0.

##### Коефіцієнт пропорційності (P/K)

The proportional gain is used to minimize the tracking error (below we use **P** to refer to both **P** or **K**).
Він відповідає за швидку відповідь і, отже, повинен бути встановлений як найвище можливо, але без введення коливань.

- If the **P** gain is too high: you will see high-frequency oscillations.
- If the **P** gain is too low:
  - транспортний засіб буде повільно реагувати на зміни введення.
  - In _Acro mode_ the vehicle will drift, and you will constantly need to correct to keep it level.

##### Похідне Надходження (D)

The **D** (derivative) gain is used for rate damping.
Це обов'язково, але повинно бути встановлене лише настільки високо, наскільки потрібно, щоб уникнути перестрілок.

- If the **D** gain is too high: the motors become twitchy (and maybe hot), because the **D** term amplifies noise.
- If the **D** gain is too low: you see overshoots after a step-input.

Типові значення:

- standard form (**P** = 1): between 0.01 (4" racer) and 0.04 (500 size), for any value of **K**
- parallel form (**K** = 1): between 0.0004 and 0.005, depending on the value of **P**

##### Інтегральний коефіцієнт (I)

The **I** (integral) gain keeps a memory of the error. The **I** term increases when the desired rate is not reached over some time.
It is important (especially when flying _Acro mode_), but it should not be set too high.

- Якщо значення I занадто високе: ви побачите повільні коливання.
- If the I gain is too low: this is best tested in _Acro mode_, by tilting the vehicle to one side about 45 degrees, and keeping it like that.
  Він повинен зберігати той самий кут.
  If it drifts back, increase the **I** gain.
  A low **I** gain is also visible in a log, when there is an offset between the desired and the actual rate over a longer time.

Типові значення:

- standard form (**P** = 1): between 0.5 (VTOL plane), 1 (500 size) and 8 (4" racer), for any value of **K**
- parallel form (**K** = 1): between 0.3 and 0.5 if **P** is around 0.15
  The pitch gain usually needs to be a bit higher than the roll gain.

#### Процедура тестування

To test the current gains, provide a fast **step-input** when hovering and observe how the vehicle reacts.
Він має негайно слідувати за командою і не коливатися, ані не перевищувати (відчувається "заблокованим").

Ви можете створити ступеневий вхід, наприклад, для крену, швидко натиснувши палицю крену в один бік, а потім швидко відпустити її (будьте обережні, що палиця також буде коливатися, якщо ви просто відпустите її, оскільки вона має пружину - добре налаштований транспортний засіб буде слідувати цим коливанням).

:::info
A well-tuned vehicle in _Acro mode_ will not tilt randomly towards one side, but keeps the attitude for tens of seconds even without any corrections.
:::

#### Журнали

Перегляд журналу допомагає оцінити продуктивність відстеження також.
Ось приклад хорошого відстеження кутової швидкості нахилу і кутової швидкості курсу:

![roll rate tracking](../../assets/mc_pid_tuning/roll_rate_tracking.png)
![yaw rate tracking](../../assets/mc_pid_tuning/yaw_rate_tracking.png)

І ось хороший приклад відстеження кутової швидкості нахилу з декількома обертами, які створюють екстремальний ступеневий вхід.
You can see that the vehicle overshoots only by a very small amount:
![roll rate tracking flips](../../assets/mc_pid_tuning/roll_rate_tracking_flip.png)

### Контролер нахилу

Це контролює орієнтацію та виводить бажані обороти тіла за допомогою наступних параметрів налаштування:

- Roll control ([MC_ROLL_P](../advanced_config/parameter_reference.md#MC_ROLL_P))
- Pitch control ([MC_PITCH_P](../advanced_config/parameter_reference.md#MC_PITCH_P))
- Yaw control ([MC_YAW_P](../advanced_config/parameter_reference.md#MC_YAW_P))

Контролер нахилу набагато простіше налаштовувати.
Фактично, більшість часу за замовчуванням не потрібно нічого змінювати.

To tune the attitude controller, fly in _Stabilized mode_ and increase the **P** gains gradually.
Якщо почнуть виникати коливання або перехід, коефіцієнти занадто великі.

Також можна налаштувати наступні параметри. Вони визначають максимальні швидкості обертання навколо всіх трьох осей:

- Maximum roll rate ([MC_ROLLRATE_MAX](../advanced_config/parameter_reference.md#MC_ROLLRATE_MAX))
- Maximum pitch rate ([MC_PITCHRATE_MAX](../advanced_config/parameter_reference.md#MC_PITCHRATE_MAX))
- Maximum yaw rate ([MC_YAWRATE_MAX](../advanced_config/parameter_reference.md#MC_YAWRATE_MAX))

### Крива тяги

Налаштування вище оптимізує продуктивність навколо ховер газу.
Але можливо ви побачите коливання при руханні в напрямку повного газу.

To counteract that, adjust the **thrust curve** with the [THR_MDL_FAC](../advanced_config/parameter_reference.md#THR_MDL_FAC) parameter.

:::info
The rate controller might need to be re-tuned if you change this parameter.
:::

The mapping from motor control signals (e.g. PWM) to expected thrust is linear by default — setting `THR_MDL_FAC` to 1 makes it quadratic.
Значення між ними використовують лінійну інтерполяцію між двома. Типові значення знаходяться між 0.3 і 0.5.

If you have a [thrust stand](https://www.tytorobotics.com/pages/series-1580-1585) <!-- RCbenchmark Series 1580/1585 Test Stand --> (or can otherwise _measure_ thrust and motor commands simultaneously), you can determine the relationship between the motor control signal and the motor's actual thrust, and fit a function to the data.
The motor command in PX4 called `actuator_output` can be PWM, Dshot, UAVCAN commands for the respective ESCs in use.
[This Notebook][THR_MDL_FAC_Calculation] shows one way for how the thrust model factor `THR_MDL_FAC` may be calculated from previously measured thrust and PWM data.
The curves shown in this plot are parametrized by both &alpha; and k, and also show thrust and PWM in real units (kgf and &mu;s).
In order to simplify the curve fit problem, you can normalize the data between 0 and 1 to find `k` without having to estimate &alpha; (&alpha; = 1, when the data is normalized).

![Thrust Curve Compensation](../../assets/mc_pid_tuning/thrust-curve-compensation.svg)] <!-- removed link to THR_MDL_FAC_Calculation as causes problems for link checker -->

:::info
The mapping between PWM and static thrust depends highly on the battery voltage.
:::

An alternative way of performing this experiment is to make a scatter plot of the normalized motor command and thrust values, and iteratively tune the thrust curve by experimenting with the `THR_MDL_FAC` parameter.
Приклад такої графіки показаний тут:

![Graph showing relative thrust and PWM scatter](../../assets/mc_pid_tuning/relative_thrust_and_pwm_scatter.svg)

Якщо дані про сигнал управління двигуном і тягу зібрані на протязі всього масштабного діапазону у експерименті, ви можете нормалізувати дані, використовуючи рівняння:

_normalized_value = ( raw_value - min (raw_value) ) / ( max ( raw_value ) - min ( raw_value ) )_

Після того, як у вас є діаграма розкиду нормалізованих значень, ви можете спробувати зробити так, щоб крива відповідала шляхом побудови рівняння

_rel_thrust = ( `THR_MDL_FAC` ) _ rel_signal^2 + ( 1 - `THR_MDL_FAC` ) \* rel_signal\*

на лінійному діапазоні нормалізованих значень команд двигуна від 0 до 1.
Note that this is the equation that is used in the firmware to map thrust and motor command, as shown in the [THR_MDL_FAC](../advanced_config/parameter_reference.md#THR_MDL_FAC) parameter reference.
Here, _rel_thrust_ is the normalized thrust value between 0 and 1, and _rel_signal_ is the normalized motor command signal value between 0 and 1.

In this example above, the curve seemed to fit best when `THR_MDL_FAC` was set to 0.7.

[THR_MDL_FAC_Calculation]: https://github.com/PX4/PX4-user_guide/blob/main/assets/config/mc/ThrustCurve.ipynb

Якщо у вас немає доступу до стенду тяги, ви також можете налаштувати коефіцієнт моделювання емпірично.
Почніть з 0,3 і збільшуйте його на 0,1 кожного разу.
Якщо він занадто великий, ви помітите початок коливань при нижніх значеннях обертів.
Якщо він занадто низький, ви помітите коливання при вищих значеннях обертів.

<a id="airmode"></a>

### Airmode & Mixer Saturation

Регулятор швидкості видає команди на обертовий момент для всіх трьох осей (кочан, тангаж і крен) і скалярне значення тяги, які потрібно перетворити в окремі команди тяги для кожного мотора.
Цей крок називається міксуванням.

Можливо, одна з команд мотора стане від'ємною, наприклад, при низькій тязі і великій команді на кочан (і подібно може бути вище 100%).
Це насичення міксера.
Фізично неможливо виконати ці команди для транспортного засобу (за винятком реверсивних моторів).
PX4 має два режими для вирішення цього:

- Або шляхом зменшення командованого моменту для кочану таким чином, щоб жодна з команд мотора не була нижче нуля (вимкнено режим "Повітряний режим").
  У крайньому випадку, коли командована тяга дорівнює нулю, це означає, що корекція атитюди вже неможлива, тому завжди потрібно мінімальне значення тяги для цього режиму.
- Або шляхом збільшення (підсилення) командованої тяги таким чином, щоб жодна з команд мотора не була від'ємною (ввімкнено режим "Повітряний режим").
  Це має велику перевагу, оскільки атитюди/швидкості можуть слідувати правильно навіть при низькій або нульовій потужності.
  Загалом це покращує польотні характеристики.

  Однак це збільшує загальну тягу, що може призвести до ситуацій, коли транспортний засіб продовжує підніматися, навіть якщо газ знижується до нуля.
  Для добре налаштованого, правильно функціонуючого транспортного засобу це не відбувається, але, наприклад, це може статися, коли транспортний засіб сильно коливається через занадто високі значення налаштування P.

Обидва режими показані нижче за допомогою двовимірної ілюстрації для двох моторів і команди на обертовий момент для кочану <span style="color:#9673A6">r</span>.
На лівому моторі <span style="color:#9673A6">r</span> додається до командованої тяги, тоді як на правому моторі він віднімається від неї.
Команди тяги мають <span style="color:#6A9153">зелений</span> колір.
З увімкненим режимом "Повітряний режим" командована тяга збільшується на <span style="color:#B85450">b</span>.
Коли він вимкнений, <span style="color:#9673A6">r</span> зменшується.

![Airmode](../../assets/mc_pid_tuning/MC_PID_tuning-Airmode.svg)

<!-- The drawing is on draw.io: https://drive.google.com/file/d/1N0qjbiJX6JuEk2I1-xFvigLEPKJRIjBP/view?usp=sharing
     On the first Tab
-->

Якщо міксування насичується до верхньої межі, командована тяга зменшується, щоб забезпечити, що жоден з моторів не буде командуватися з більшою тягою 100%.
Ця поведінка схожа на логіку режиму "Повітряний режим", і застосовується незалежно від того, чи ввімкнений режим "Повітряний режим", чи вимкнений.

Once your vehicle flies well you can enable Airmode via the [MC_AIRMODE](../advanced_config/parameter_reference.md#MC_AIRMODE) parameter.
