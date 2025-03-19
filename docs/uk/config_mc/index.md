# Конфігурація мультикоптера

Конфігурація та калібрування багтороторного вертольота виконується за тими ж високорівневими кроками, що і інші рами: вибір прошивки, конфігурація рами, включаючи геометрію приводника/двигуна та відображення виводів, конфігурація та калібрування сенсорів, налаштування безпеки та інших функцій, а також налаштування.

This topic explains how to configure a multicopter using selected topics from [Standard Configuration](../config/index.md), [Advanced Configuration](../advanced_config/index.md), and [Flight Controller Peripherals](../peripherals/index.md), along with multicopter-specific tuning topics.

:::info
This topic is the recommended entry point when performing first-time configuration and calibration of a new multicopter frame.
:::

## Завантаження прошивки

The first step is to [load PX4 firmware](../config/firmware.md) onto your [flight controller](../flight_controller/index.md).
Це найлегше зробити за допомогою QGroundControl, який автоматично вибере відповідне прошивку для вашого конкретного обладнання контролера.
За замовчуванням QGC встановить останню стабільну версію PX4, але ви можете вибрати бета-версію або власні версії, якщо потрібно.

Відповідні теми:

- [Loading Firmware](../config/firmware.md)

## Вибір та налаштування каркасу

Цей розділ пояснює, як налаштувати тип транспортного засобу (багатокоптер), конкретну геометрію двигуна/керування польотом та виходи двигуна.

First [select a multicopter airframe](../config/airframe.md) (options are listed in [Airframe Reference > Copter](../airframes/airframe_reference.md#copter)).
Ви повинні вибрати рамку, яка відповідає марці та моделі вашого транспортного засобу, якщо така існує, а в іншому випадку виберіть тип рамки "Загальний", який найбільш точно відповідає вашій геометрії за кількістю двигунів та їх відносними положеннями.
For example, for a [Quadrotor X](../airframes/airframe_reference.md#quadrotor-x) frame you would look for the name of your frame in the list, and if it was not present select the [Generic Quadrotor X](../airframes/airframe_reference.md#copter_quadrotor_x_generic_quadcopter) frame.

:::info
Any selected multicopter frame can be modified in the next step (actuator configuration) to add/remove motors and otherwise change the geometry, and to specify what flight controller outputs are connected to particular motors and the output properties.
Вибір рамки, яка відповідає вашому транспортному засобу, зменшує роботу з налаштуванням, необхідну для виконання.

:::details
How does this work (details)
Selecting an airframe applies a [frame configuration file](../dev_airframes/adding_a_new_frame.md#adding-a-frame-configuration) that contains a predefined set of [parameters](../advanced_config/parameters.md), such as [CA_AIRFRAME=0](../advanced_config/parameter_reference.md#CA_AIRFRAME) for the vehicle type and [CA_ROTOR_COUNT](../advanced_config/parameter_reference.md#CA_ROTOR_COUNT) for the number of rotors.

Конфігурація рами може визначити все про транспортний засіб, від його геометрії та відображень виходу до налаштувань та калібрування значень.
Коли ви виводите новий транспортний засіб, рама зазвичай містить досить мінімальну конфігурацію:

- Кадри з назвою "Загальний" визначають тип транспортного засобу, кількість роторів та позиції роторів-заповнювачі.
 Після вибору конструкції фюзеляжу ви визначаєте фактичну геометрію, а потім налаштовуєте виходи.
- Кадри з назвою моделі/бренду визначать тип транспортного засобу, кількість роторів, фактичні позиції роторів та напрямки руху двигуна.
 Після вибору конструкції фюзеляжу вам зазвичай все ще потрібно налаштувати виводи.

:::

The next step is to define your vehicle [geometry](../config/actuators.md#motor-geometry-multicopter) (the number of motors and their relative positions) and [assign those motors](../config/actuators.md#actuator-outputs) to the physical outputs that they are wired to on your flight controller (both of these are covered in [Actuator Configuration and Testing](../config/actuators.md)).

If using PWM ESCs and OneShot ESCs (but not DShot and DroneCAN/Cyphal ESC) you should then perform [ESC Calibration](../advanced_config/esc_calibration.md) before proceeding to [Motor Configuration](../config/actuators.md#motor-configuration).
Це забезпечує, що всі ESC надають точно такий самий вихід для заданого входу (ідеально, ми спочатку калібруємо ESC, але ви не можете калібрувати свої ESC, поки ви не відобразите виходи).

The final step is [Motor Configuration](../config/actuators.md#motor-configuration):

- [Reverse any motors](../config/actuators.md#reversing-motors) that don't match the spin direction configured in the Geometry.
 Для DShot ESC ви можете це зробити через інтерфейс тестування приводу.
- PWM, OneShot та CAN ESC встановлюють ліміти введення мотора для режимів роззброєння, низької та високої швидкості (не потрібно для DShot ESC)

Відповідні теми:

- [Vehicle (Frame) Selection](../config/airframe.md) — Select vehicle type to match your frame.
- [Actuator Configuration and Testing](../config/actuators.md) — Vehicle geometry, output mapping, motor configuration, testing.
- [ESC Calibration](../advanced_config/esc_calibration.md) — Do between output mapping and motor configuration (topic above) for PWM and OneShot ESC.

## Налаштування та калібрування датчика

PX4 найчастіше покладається на магнітометр (компас) для отримання інформації про напрямок, барометр для висоти, гіроскоп для швидкостей тіла, акселерометр для ставлення та GPS/GNSS для глобального положення.
Польотні контролери Pixhawk (і багато інших) мають вбудований магнітометр, акселерометр, гіроскоп та барометр.
Вбудований компас зазвичай не є особливо надійним, і часто додається зовнішній компас (зазвичай поєднаний з приймачем ГНСС в одному пристрої).

We first need to set the [Sensor Orientation](../config/flight_controller_orientation.md), informing PX4 how the autopilot (and its inbuilt sensors) and external compasses are oriented relative to the vehicle.
Загалом ви будете орієнтуватися на передню частину транспортного засобу і не матимете встановлювати нічого.
Після того, як це зроблено, нам потрібно калібрувати компас(и), гіроскоп та акселерометр.

Основний датчик налаштування вкритий у цих темах:

- [Sensor Orientation](../config/flight_controller_orientation.md)
- [Компас](../config/compass.md)
- [Гіроскоп](../config/gyroscope.md)
- [Акселерометр](../config/accelerometer.md)

PX4 може використовувати інші периферійні пристрої, такі як датчики відстані, оптичні датчики руху, сигнали уникання трафіку, камери тощо:

- [Периферія контролера польоту](../peripherals/README.md) - налаштування конкретних датчиків, опціональних датчиків, приводів тощо.

:::info
Sensors that you don't need to calibrate/configure include:

- [Level Horizon](../config/level_horizon_calibration.md) calibration isn't usually needed if you have mounted the flight controller level.
- Sensors that are not present, or that are not used by PX4 multicopter for flight control, such as [Airspeed sensors](../config/airspeed.md).
- Датчики, які не потребують калібрування, включаючи: Барометри та GNSS.

:::

## Налаштування керування вручному режимі

Пілоти можуть керувати транспортним засобом вручну за допомогою системи радіоуправління (RC) або контролера джойстика/геймпада, підключеного через QGroundControl.

:::info
A manual control is essential in order to bring up a new vehicle safely!
:::

Радіоуправління:

- [Radio Controller (RC) Setup](../config/radio.md)
- [Flight Mode Configuration](../config/flight_mode.md)

Джойстик/Ґеймпад:

- [Joystick Setup](../config/joystick.md) (includes button/flight mode mapping)

## Конфігурація безпеки

PX4 може бути налаштований для автоматичної обробки умов, таких як низький заряд акумулятора, втрата радіо- або даних, польот занадто далеко від дому та інше:

- [Battery Estimation Tuning](../config/battery.md) — estimate remaining power (needed for low power failsafe).
- [Safety Configuration (Failsafes)](../config/safety.md)

## Вдосконалення

Налаштування - це останній крок, який виконується лише після завершення більшості інших налаштувань та конфігурації.

- Контролери швидкості та нахилу:

- [Autotune](../config/autotune_mc.md) — Automates tuning PX4 rate and attitude controllers (recommended).

 ::: info
 Automatic tuning works on frames that have reasonable authority and dynamics around all the body axes.
 Це було перевірено в основному на гоночних квадрокоптерах та X500, і очікується, що воно буде менш ефективним на трикоптерах з нахилом ротора.

 Manual tuning using these guides are only needed if there is a problem with autotune:

 - [MC PID Tuning (Manual/Basic)](../config_mc/pid_tuning_guide_multicopter_basic.md) — Manual tuning basic how to.
 - [MC PID Tuning Guide (Manual/Detailed)](../config_mc/pid_tuning_guide_multicopter.md) — Manual tuning with detailed explanation.


:::

- [MC Filter/Control Latency Tuning](../config_mc/filter_tuning.md) — Trade off control latency and noise filtering.

- [MC Setpoint Tuning (Trajectory Generator)](../config_mc/mc_trajectory_tuning.md)
 - [MC Jerk-limited Type Trajectory](../config_mc/mc_jerk_limited_type_trajectory.md)

- [Multicopter Racer Setup](../config_mc/racer_setup.md)

<!--
- Explain what you have to tune on PX4, what you can tune, and what each topic covers
- I expect we should start with an exhaustive list of the tuning you could want to do - such as position tuning, etc. Do we have one?
 -->

<!-- TBD this is just text for me to mine

AFAIK autotune was tested on various not so custom platforms e.g. X500, racer quad, Loong standard VTOL. I honestly used it only once on a tricopter and it worked for roll and pitch but the resulting yaw tuning was not stable. Since then it was improved but that's not merged yet :eyes: https://github.com/PX4/PX4-Autopilot/pull/21857
Autotune was never tested on a Helicopter.
can you in theory autotune frame with any number of motors?
In theory yes but it needs to be able to have reasonable authority around all axes so I'd expect autotune to not work well for a monocopter without swashplate and so on. Probably also the controllers wouldn't work out of the box. I saw issues before with designs that tilt the rotor e.g. tricopter, bicopter, ... again


will PX4 still understand how to autotune?
Autotune should work for any vehicle that has reasonable authority and dynamics around all the body axes. A tiltable motor e.g. tricopter has at the least dynamics which are less tested with autotune.
My assumption is that the mixing system can cope with whatever geometry you throw at it.
Yes but it must be physically feasible. E.g. if you make a quadrotor where all motors turn the same way it will "deal" with it but that cannot work without very specific controllers. Same for a monocopter or a tricopter without swiveling one motor.
-->

## Дивіться також

- [QGroundControl > Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/setup_view.html)
- [Периферія контролера польоту](../peripherals/README.md) - налаштування конкретних датчиків, опціональних датчиків, приводів тощо.
- [Advanced Configuration](../advanced_config/index.md) - Factory/OEM calibration, configuring advanced features, less-common configuration.
- Конфігурація/налаштування, що залежать від апарату:
 - **Multicopter Config/Tuning**
 - [Конфігурація/налаштування гелікоптера](../config_heli/index.md)
 - [Fixed Wing Config/Tuning](../config_fw/index.md)
 - [Конфігурація/налаштування VTOL](../config_vtol/index.md)
