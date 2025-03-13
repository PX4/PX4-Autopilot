# Multicopter Racer Setup

This page describes how to setup and configure a multicopter racer for optimal performance (in particular for [Acro mode](../flight_modes_mc/acro.md)).

Пам'ятайте, що гонщики - це швидкі транспортні засоби, спеціально створені для того, щоб бути переважними!
Вам вже слід мати деякий досвід, або дозвольте комусь з досвідом допомогти вам.

:::tip
Many things described here can also be applied to improve the flight performance of other types of multicopters.
:::

:::info
A racer usually omits some sensors (e.g. GPS).
Як результат, менше надійних опцій доступно.
:::

## Опції збірки

Звичайно гонщик пропускає деякі датчики.

Мінімальна конфігурація полягає у використанні лише гіроскопа та акселерометра.

:::info
If the board has an internal magnetometer, it should not be used (small racers are particularly prone to strong electromagnetic interference).
:::

Гонщики зазвичай не мають GPS, оскільки воно додає деяку вагу та може пошкодитися під час аварій (GPS + зовнішній магнітометр повинен бути розміщений на стійці GPS подалі від великих струмів, щоб уникнути магнітних перешкод, що, на жаль, означає, що його легко розбити).

Проте є деякі переваги у додаванні GPS, особливо для початківців:

- Ви можете перейти в режим утримання позиції, і транспортний засіб просто залишиться на місці.
  Це зручно, якщо ви втратили орієнтацію або потребуєте перерви.
  Це також може бути використано для безпечної посадки.
- [Return mode](../flight_modes_mc/return.md) can be used, either on a switch or as RC loss/low battery failsafe.
- Ви матимете останню позицію, коли він вріжеся.
- Журнал містить трек польоту, що означає, що ви можете переглянути польот (у 3D).
  Це може допомогти вам покращити ваші акробатичні льотні навички.

:::info
During aggressive acrobatic maneuvers the GPS can lose its position fix for a short time.
If you switch into [position mode](../flight_modes_mc/position.md) during that time, [altitude mode](../flight_modes_mc/altitude.md) will be used instead until the position becomes valid again.
:::

## Налаштування програмного забезпечення

Наступні абзаци описують кілька важливих моментів під час будівництва транспортного засобу.
If you need complete build instructions, you can follow the [QAV-R 5" KISS ESC Racer](../frames_multicopter/qav_r_5_kiss_esc_racer.md) build log.

### Налаштування вібрації

Існують різні підходи до кріплення для зменшення вібрацій.
For example, the flight controller can be mounted with vibration dampening foam, or using [O-rings](../frames_multicopter/qav_r_5_kiss_esc_racer.md#mounting).

While there is no single best method, you will typically have fewer problems with vibrations if you use high-quality components (frame, motors, props) as for example used in the [QAV-R 5" KISS ESC Racer](../frames_multicopter/qav_r_5_kiss_esc_racer.md).

Make sure to use **balanced props**.

### Центр гравітації

Переконайтеся, що центр ваги розташований як можливо ближче до центру тяги.
Баланс ліворуч-праворуч зазвичай не є проблемою, але баланс вперед-назад може бути.
Ви можете перемістити батарею до тих пір, поки вона не буде вірною і позначити її на рамі, щоб завжди правильно її розміщувати.

:::info
The integral term can account for an imbalanced setup, and a custom mixer can do that even better.
Проте краще виправити будь-який дисбаланс як частину налаштування автомобіля.
:::

## Налаштування програмного забезпечення

Після того, як ви побудуєте гонщика, вам потрібно буде налаштувати програмне забезпечення.

Go through the [Basic Configuration Guide](../config/index.md).
In particular, set the [Airframe](../config/airframe.md) that most closely matches your frame (typically you will choose the [Generic 250 Racer](../airframes/airframe_reference.md#copter_quadrotor_x_generic_250_racer) airframe, which sets some racer-specific parameters by default).

Ці параметри є важливими:

- Enable One-Shot or DShot by selecting the protocol for a group of outputs during [Actuator Configuration](../config/actuators.md).
- Set the maximum roll-, pitch- and yaw rates for Stabilized mode as desired: [MC_ROLLRATE_MAX](../advanced_config/parameter_reference.md#MC_ROLLRATE_MAX), [MC_PITCHRATE_MAX](../advanced_config/parameter_reference.md#MC_PITCHRATE_MAX) and [MC_YAWRATE_MAX](../advanced_config/parameter_reference.md#MC_YAWRATE_MAX).
  The maximum tilt angle is configured with [MPC_MAN_TILT_MAX](../advanced_config/parameter_reference.md#MPC_MAN_TILT_MAX).
- The minimum thrust [MPC_MANTHR_MIN](../advanced_config/parameter_reference.md#MPC_MANTHR_MIN) should be set to 0.

### Оцінювач

Якщо ви використовуєте GPS, ви можете пропустити цей розділ і використовувати типовий оцінювач.
В іншому випадку вам слід перейти на оцінювач відносин Q, який працює без магнітометра або барометра.

To enable it set [ATT_EN = 1](../advanced_config/parameter_reference.md#ATT_EN), [EKF2_EN =0 ](../advanced_config/parameter_reference.md#EKF2_EN) and [LPE_EN = 0](../advanced_config/parameter_reference.md#LPE_EN) (for more information see [Switching State Estimators](../advanced/switching_state_estimators.md#how-to-enable-different-estimators)).

Потім змініть наступні параметри:

- Set [SYS_HAS_MAG](../advanced_config/parameter_reference.md#SYS_HAS_MAG) to `0` if the system does not have a magnetometer.
- Set [SYS_HAS_BARO](../advanced_config/parameter_reference.md#SYS_HAS_BARO) to `0` if the system does not have a barometer.
- Configure the Q estimator: set [ATT_ACC_COMP](../advanced_config/parameter_reference.md#ATT_ACC_COMP) to `0`, [ATT_W_ACC](../advanced_config/parameter_reference.md#ATT_W_ACC) to 0.4 and [ATT_W_GYRO_BIAS](../advanced_config/parameter_reference.md#ATT_W_GYRO_BIAS) to 0.
  Ви можете налаштувати це пізніше, якщо бажаєте.

### Безаварійність

Configure [RC loss and low battery failsafe](../config/safety.md).
If you do not use a GPS, set the failsafe to **Lockdown**, which turns off the motors.
Тест втрати дистанційного керування на лавці без пристроїв, приєднаних шляхом вимкнення пульта дистанційного керування, коли транспортний засіб увімкнено.

Make sure to assign a [kill switch](../config/safety.md#kill-switch) or an [arming switch](../config/safety.md#arm-disarm-switch).
Протестируйте його і навчіться його використовувати!

### Налаштування PID

:::info
Make sure to calibrate the ESCs before doing any tuning.
:::

На цьому етапі ви повинні бути готові до першого випробувального польоту.

Assuming the vehicle is able to fly using the default settings, we then do a first pass of [Basic MC PID tuning](../config_mc/pid_tuning_guide_multicopter_basic.md).
The vehicle needs to be **undertuned** (the **P** and **D** gains should be set too low), such that there are no oscillations from the controller that could be interpreted as noise (the default gains might be good enough).
This is important for the [filter tuning](#filter-tuning) (there will be a second PID tuning round later).

### Затримка керування

The _control latency_ is the delay from a physical disturbance of the vehicle until the motors react to the change.

:::tip
It is _crucial_ to reduce the control latency as much as possible!
A lower latency allows you to increase the rate **P** gains, which means better flight performance.
Навіть одна мілісекунда, додана до затримки, робить різницю.
:::

Це фактори, які впливають на затримку:

- М'яка конструкція або м'яка амортизація вібрацій збільшує затримку (вони діють як фільтр).
- [Low-pass filters](../config_mc/filter_tuning.md) in software and on the sensor chip trade off increased latency for improved noise filtering.
- Внутрішні складові програмного забезпечення PX4: сигнали датчиків потрібно зчитати у драйвері, а потім пройти через контролер до виходного драйвера.
- The IO chip (MAIN pins) adds about 5.4 ms latency compared to using the AUX pins (this does not apply to a _Pixracer_ or _Omnibus F4_, but does apply to a Pixhawk).
  Щоб уникнути затримки введення-виведення, підключіть мотори до додаткових контактів замість головних.
- PWM output signal: enable [Dshot](../peripherals/dshot.md) by preference to reduce latency (or One-Shot if DShot is not supported).
  The protocol is selected for a group of outputs during [Actuator Configuration](../config/actuators.md).

### Налаштування фільтра

Фільтри вирішують проблему затримки управління та фільтрації шуму, обидва з яких впливають на продуктивність.
For information see: [Filter/Control Latency Tuning](../config_mc/filter_tuning.md)

### Налаштування PID (другий раунд)

Зараз проведіть другий раунд налаштування PID, на цей раз якнайщільніше, а також налаштовуйте криву тяги.

:::tip
You can use the approach described in [Basic MC PID tuning](../config_mc/pid_tuning_guide_multicopter_basic.md) to tune the frame, but you will need to use the [Advanced Multicopter PID Tuning Guide (Advanced/Detailed)](../config_mc/pid_tuning_guide_multicopter.md#thrust-curve) to understand how to tune the thrust curve.

### Режим польоту

After you have verified that the vehicle flies well at low and high throttle, you can enable [airmode](../config_mc/pid_tuning_guide_multicopter.md#airmode) with the [MC_AIRMODE](../advanced_config/parameter_reference.md#MC_AIRMODE) parameter.
Ця функція гарантує, що автомобіль все ще контролюється і відслідковує швидкість з низькою частотою.
