# Покращене налаштування TECS (Вага та висота)

This topic shows how you can compensate for changes to the [mass of the vehicle](#vehicle-weight-compensation) and the [air density](#air-density-compensation), along with information about the [algorithms](#weight-and-density-compensation-algorithms) that are used.

:::warning
Ця тема вимагає, щоб ви вже виконали [основне налаштування TECS](../config_fw/position_tuning_guide_fixedwing.md#tecs-tuning-altitude-and-airspeed).
:::

[Основне налаштування TECS](../config_fw/position_tuning_guide_fixedwing.md#tecs-tuning-altitude-and-airspeed) встановило ключові обмеження продуктивності транспортного засобу, які необхідні для належної роботи контролера висоти та швидкості.

Хоча ці обмеження вказані за допомогою постійних параметрів, насправді продуктивність транспортного засобу не є постійною і залежить від різних факторів.
If changes in vehicle mass and air density are not taken into account, altitude and airspeed tracking will likely deteriorate in the case where the configuration (air density and mass) deviate significantly from the configuration at which the vehicle was tuned.

## Компенсація ваги транспортного засобу

Set (both) the following parameters to scale the maximum climb rate, minimum sink rate, and adjust airspeed limits for the vehicle mass:

- [WEIGHT_BASE](../advanced_config/parameter_reference.md#WEIGHT_BASE) — the mass of the vehicle at which the [Basic TECS tuning](../config_fw/position_tuning_guide_fixedwing.md#tecs-tuning-altitude-and-airspeed) was performed.
- [WEIGHT_GROSS](../advanced_config/parameter_reference.md#WEIGHT_GROSS) — the actual mass of the vehicle at any given time, for example when using a larger battery, or with a payload that was not present during tuning.

You can determine the values by measuring the mass of the vehicle using a scale in the tuning configuration and when flying with a payload.

:::info
The `WEIGHT_*` parameters are masses in kilograms; the parameter names and the derived _weight ratio_ (the ratio of the current vehicle mass to `WEIGHT_BASE`) use "weight" following common aviation usage.
:::

Масштабування виконується, коли _обидва_ `WEIGHT_BASE` та `WEIGHT_GROSS` більше `0`, і не матиме жодного впливу, якщо значення однакові.
Дивіться розділ [алгоритми](#weight-and-density-compensation-algorithms) нижче для отримання додаткової інформації.

### Fuel Burn Compensation

On combustion-engine vehicles a significant proportion of the takeoff mass is fuel, so the vehicle mass decreases considerably over the course of a flight.
If a fuel tank sensor is present (publishing to the [FuelTankStatus](../msg_docs/FuelTankStatus.md) message, for example from a [DroneCAN](../dronecan/index.md) engine ECU), PX4 can continuously estimate the current vehicle mass from the amount of burned fuel.

To enable this, additionally set:

- [WEIGHT_FUEL](../advanced_config/parameter_reference.md#WEIGHT_FUEL) — the mass of a full fuel load (tank capacity multiplied by fuel density).

When `WEIGHT_FUEL` is greater than `0`, `WEIGHT_GROSS` must be set to the takeoff mass with a _full_ tank.
The current mass is then computed from the measured fraction of fuel remaining, so no parameter update is needed when taking off with a partially filled tank.
The measured fuel level is low-pass filtered to reject fuel slosh.

If no fuel data is available (no sensor, or the sensor fails before boot), any mass-based scaling makes the conservative assumption that the vehicle is at the full gross mass (including fuel).
If fuel data is lost mid-flight, the last known fuel state is kept.

Fuel-consumption compensation is only applied to the fuel tank with id `0` (the default id for single-tank systems).
Multi-tank vehicles should publish the aggregated total fuel state as tank `0`.
A warning is shown if fuel data is not received from id `0` but is received for other tank ids.

## Компенсація щільності повітря

### Вкажіть максимальну висоту обслуговування

In PX4 the service ceiling [FW_SERVICE_CEIL](../advanced_config/parameter_reference.md#FW_SERVICE_CEIL) specifies the altitude in standard atmospheric conditions at which the vehicle is still able to achieve a maximum climb rate of 0.5 m/s at maximum throttle and a vehicle mass equal to [WEIGHT_BASE](../advanced_config/parameter_reference.md#WEIGHT_BASE).
За замовчуванням цей параметр вимкнений, і компенсація не відбудеться.

Цей параметр потрібно визначити експериментально.
Завжди краще встановлювати консервативне значення (нижче значення), ніж оптимістичне значення.

### Застосувати корекцію щільності до мінімальної швидкості опускання

Мінімальна швидкість опускання встановлюється в [FW_T_SINK_MIN](../advanced_config/parameter_reference.md#FW_T_SINK_MIN).

Якщо налаштування [Основного налаштування TECS](../config_fw/position_tuning_guide_fixedwing.md#tecs-tuning-altitude-and-airspeed) не було виконано в стандартних умовах рівня моря, тоді параметр [FW_T_SINK_MIN](../advanced_config/parameter_reference.md#FW_T_SINK_MIN) повинен бути змінений шляхом множення на корекційний фактор $P$ (де $\rho$ - густина повітря під час налаштування):

$$P = \sqrt{\frac{\rho}{\rho_{\text{sealevel}}}}$$

Для отримання додаткової інформації див. [Ефект густини на мінімальну швидкість опускання](#effect-of-density-on-minimum-sink-rate).

### Застосувати корекцію щільності до обрізання ручки газу

Регулювання обтічної дроселі встановлюється за допомогою [FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM).

Якщо базове налаштування не було виконано в стандартних умовах рівня моря, тоді значення для [FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM) повинно бути змінено шляхом множення на корекційний фактор $P$:

$$P = \sqrt{\frac{\rho}{\rho_{\text{sealevel}}}}$$

Для отримання додаткової інформації див. [Ефект густини на обрізний регулятор](#effect-of-density-on-trim-throttle)

## Алгоритми насиченості ваги та щільності

У цьому розділі міститься інформація про операції масштабування, виконані PX4.
Це надається лише для цікавості, і може бути цікавим для розробників, які хочуть змінити код масштабування.

### Записка

У наступних розділах ми будемо використовувати позначення $\hat X$ для того, щоб вказати, що це значення є каліброваним значенням змінної $X$.
By calibrated we mean the value of that variable measured at sea level in standard atmospheric conditions, and when the vehicle mass was equal to [WEIGHT_BASE](../advanced_config/parameter_reference.md#WEIGHT_BASE).

Наприклад, by $\hat{\dot{h}}_{\text{max}}$ we specify the maximum climb rate the vehicle can achieve at [WEIGHT_BASE](../advanced_config/parameter_reference.md#WEIGHT_BASE) at sea level in standard atmospheric conditions.

### Effect of Mass on Maximum Climb Rate

Максимальна швидкість підйому ([FW_T_CLMB_MAX](../advanced_config/parameter_reference.md#FW_T_CLMB_MAX)) масштабується як функція відношення ваги.

З рівноважних рівнянь руху літака ми встановлюємо, що максимальна швидкість підйому може бути записана як:

$$\dot{h}_{\text{max}} = \frac{V \cdot (\text{Thrust} - \text{Drag})}{m \cdot g}$$

де `V` - це справжня швидкість повітря, а `m` - маса транспортного засобу.
З цього рівняння ми бачимо, що максимальні швидкості підйому масштабуються з масою транспортного засобу.

### Effect of Mass on Minimum Sink Rate

Мінімальна швидкість опускання ([FW_T_SINK_MIN](../advanced_config/parameter_reference.md#FW_T_SINK_MIN)) масштабується як функція відношення ваги

Мінімальна швидкість опускання може бути записана як:

$$\dot{h}_{\text{min}} = \sqrt{\frac{2mg}{\rho S}}\, f(C_L, C_D)$$

де $\rho$ - щільність повітря, S - площа опорної поверхні крила, а $f(C_L, C_D)$ - функція полюсів, підйому та опору.

З цього рівняння бачимо, що мінімальна швидкість опускання масштабується з квадратним коренем відношення ваги.

### Effect of Mass on Airspeed Limits

The minimum airspeed ([FW_AIRSPD_MIN](../advanced_config/parameter_reference.md#FW_AIRSPD_MIN)), the stall airspeed ([FW_AIRSPD_STALL](../advanced_config/parameter_reference.md#FW_AIRSPD_STALL)) and trim airspeed ([FW_AIRSPD_TRIM](../advanced_config/parameter_reference.md#FW_AIRSPD_TRIM)) are adjusted based on the weight ratio specified by [WEIGHT_BASE](../advanced_config/parameter_reference.md#WEIGHT_BASE) and [WEIGHT_GROSS](../advanced_config/parameter_reference.md#WEIGHT_GROSS).

У стані сталого польоту ми можемо вимагати, щоб підйом був рівним вазіллю транспортного засобу:

$$\text{Lift} = mg = \frac{1}{2} \rho V^2 S C_L$$

перегруповування цього рівняння для швидкості повітря дає:

$$V = \sqrt{\frac{2mg}{\rho S C_L}}$$

From this equation we see that if we assume a constant angle of attack (which we generally desire), the vehicle mass affects airspeed with a square root relation.
Отже, обмеження швидкості повітря, згадані вище, масштабуються за допомогою квадратного кореня відношення ваги.

### Effect of Bank Angle on Airspeed Limits

Flying a coordinated, level turn at bank angle $\phi$ increases the load factor by $\frac{1}{\cos{\phi}}$. This is similar to the added load factor due to increased mass (section above), and thus the stall and minimum airspeeds are increased by an additional factor of $\sqrt{\frac{1}{\cos{\phi}}}$.

The maximum airspeed ([FW_AIRSPD_MAX](../advanced_config/parameter_reference.md#FW_AIRSPD_MAX)) is _not_ compensated in this way, as it can represent structural limits of the airframe.

It can be that at maximum bank angle [FW_R_LIM](../advanced_config/parameter_reference.md#FW_R_LIM), the maximum airspeed is _lower_ than the minimum airspeed (compensated for weight ratio and bank angle). This means the allowed airspeed range is empty at that bank angle. If a system is configured like this, a warning on the ground station is shown.

### Вплив щільності на максимальну швидкість підйому

Максимальна швидкість підйому встановлюється за допомогою [FW_T_CLMB_MAX](../advanced_config/parameter_reference.md#FW_T_CLMB_MAX).

Як ми вже бачили раніше, максимальна швидкість підйому може бути сформульована як:

$$\dot{h}_{\text{max}} = \frac{V \cdot (\text{Thrust} - \text{Drag})}{m \cdot g}$$

Густина повітря впливає на швидкість повітря, тягу та опір, і моделювання цих ефектів не є прямолінійним.
Проте ми можемо посилатися на літературу та досвід, які вказують, що для літака з гвинтовим пропелером максимальна швидкість підйому зменшується приблизно лінійно з густиною повітря.
Таким чином, ми можемо написати максимальну швидкість підйому як:

$$\dot{h}_{\text{max}} = \hat{\dot{h}} \cdot \frac{\rho_{\text{sealevel}}}{\rho} K$$

where $\rho_{\text{sealevel}}$ is the air density at sea level in the standard atmosphere and K is a scaling factor which determines the slope of the function.
Замість спроби ідентифікувати ці константи, звичайною практикою в авіації є вказання висоти службового стелі, на якій транспортний засіб все ще може досягти мінімально вказаної швидкості підйому.

### Вплив щільності на мінімальну швидкість опускання

Мінімальна швидкість опускання встановлюється за допомогою [FW_T_SINK_MIN](../advanced_config/parameter_reference.md#FW_T_SINK_MIN).

У попередніх розділах ми бачили формулу для мінімальної швидкості опускання:

$$\dot{h}_{\text{min}} = \sqrt{\frac{2mg}{\rho S}}\, f(C_L, C_D)$$

Це показує, що мінімальна швидкість опускання масштабується з квадратним коренем відношення оберненої густини повітря.

### Вплив щільності на обертовий регулятор обрізання

TODO: Додати тут похідну.
