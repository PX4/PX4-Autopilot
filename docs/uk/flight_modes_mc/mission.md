# Режим місії (Мультикоптер)

<img src="../../assets/site/position_fixed.svg" title="Global position fix required (e.g. GPS)" width="30px" />

_Режим місії_ змушує транспортний засіб виконувати передбачений автономний [план місії](../flying/missions.md) (план польоту), який був завантажений до керуючого пристрою польоту.
Зазвичай місія створюється та завантажується за допомогою програми для керування наземною станцією (GCS), такої як [QGroundControl](https://docs.qgroundcontrol.com/master/en/) (QGC).

::: info

- Цей режим потребує глобальної оцінки 3D-позиції (з GPS або виведеної з [локальної позиції](../ros/external_position_estimation.md#enabling-auto-modes-with-a-local-position)).
- Транспортний засіб повинен бути озброєний перед тим, як цей режим може бути активований.
- Цей режим є автоматичним - для керування автомобілем не потрібно втручання користувача.
- Перемикачі керування RC можуть використовуватися для зміни режимів польоту на будь-якому транспортному засобі.
- Рух палиць дистанційного керування буде [за замовчуванням](#COM_RC_OVERRIDE) змінювати транспортний засіб на [режим позиції](../flight_modes_mc/position.md), якщо не виникне критична аварія батареї.
  Це справжнє для багтороторів і ВПС у режимі КУ.

:::

## Опис

Місії зазвичай створюються в земній контрольній станції (наприклад, [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/plan_view.html)) та завантажуються перед запуском.
They may also be created by a MAVLink API such as [MAVSDK](../robotics/mavsdk.md), and/or uploaded in flight.

Індивідуальні [команди місії](#mission-commands) обробляються таким чином, який є відповідним для характеристик багтороторного польоту (наприклад, обертання виконується у вигляді _залишання на місці_).

:::info
Місії завантажуються на SD-карту, яку потрібно вставити **перед** запуском автопілота.
:::

На високому рівні всі типи транспортних засобів ведуть себе однаково, коли ввімкнено режим МІСІЯ:

1. Якщо місія не збережена, або якщо PX4 завершив виконання всіх команд місії, або якщо [місія не є можливою](#mission-feasibility-checks):

  - Якщо літає транспортний засіб, він буде утримувати.
  - Якщо посадять транспортний засіб, він буде "чекати".

2. Якщо місія збережена, а PX4 летить, вона виконає [місію / план польоту](../flying/missions.md) з поточного кроку.
  - Пункт `TAKEOFF` трактується як звичайна точка місії.

3. Якщо місія збережена і PX4 приземлився:
  - PX4 виконає [місію/план польоту](../flying/missions.md).
  - Якщо місія не має пункту `TAKEOFF`, то PX4 підніме транспортний засіб на мінімальну висоту перед виконанням решти польотного плану з поточного кроку.

4. Якщо жодне завдання не збережено, або якщо PX4 завершив виконання всіх команд місії:
  - Якщо літає транспортний засіб, він буде утримувати.
  - Якщо посадять транспортний засіб, він буде "чекати".

5. Ви можете вручну змінити поточну команду місії, вибравши її в _QGroundControl_.

  :::info
  Якщо у вас є команда _Перейти до елементу_ в місії, переміщення до іншого елементу **не** скине лічильник циклу.
  Однією з наслідків є те, що якщо ви зміните поточну команду місії на 1, це не призведе до "повного перезапуску" місії.

:::

6. Місія скине тільки тоді, коли транспортний засіб буде роззброєний або коли буде завантажена нова місія.

  :::tip
  Щоб автоматично роззброїти транспортний засіб після посадки, у _QGroundControl_ перейдіть до [Налаштування Транспортного Засобу > Безпека](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/safety.html), перейдіть до _Налаштувань Режиму Посадки_ та позначте прапорець _Роззброювати після_.
  Введіть час очікування після посадки перед відброюванням транспортного засобу.

:::

Місії можна призупинити, переключившись з режиму місії на будь-який інший режим (наприклад, [режим утримання](../flight_modes_mc/hold.md) або [режим позиціонування](../flight_modes_mc/position.md)), і продовжити, переключившись назад в режим місії.
Якщо транспортний засіб не захоплював зображення, коли він був призупинений, під час відновлення він рухатиметься зі своєї _поточної позиції_ до тієї ж точки шляху, до якої він спочатку рухався.
Якщо транспортний засіб захоплював зображення (має елементи спуску камери), він замість цього рухатиметься зі своєї поточної позиції до останньої точки шляху, якою він проїхав (перед зупинкою), а потім пройде свій шлях з тією самою швидкістю та з такою самою поведінкою спуску камери.
Це забезпечує, що планований шлях зафіксований під час місій з опитування/камери.
Місію можна завантажити, коли транспортний засіб зупинений, у такому випадку поточний активний елемент місії встановлюється на 1.

:::info
Коли місію призупинено під час спрацювання камери на транспортному засобі, PX4 встановлює поточний активний пункт місії на попередню точку маршруту, так що при відновленні місії транспортний засіб буде повторювати свій останній етап місії.
Крім того, PX4 зберігає останні застосовані пункти місії для налаштування швидкості та спуску камери (з вже покритого плану місії) та знову застосовує ці налаштування при відновленні місії.
:::

:::warning
Переконайтеся, що палиця регулювання газу не дорівнює нулю перед переключенням в будь-який режим RC (інакше транспортний засіб розбився).
Ми рекомендуємо вам вирівнювати ручки керування перед переходом до будь-якого іншого режиму.
:::

Для отримання додаткової інформації про планування місії, див:

- [Планування місій](../flying/missions.md)
- [План Перегляду](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/plan_view.html) (_Посібник користувача QGroundControl_)

## Перевірки можливостей місії

PX4 runs some basic sanity checks to determine if a mission is feasible when it is uploaded and before executing a mission.
If any of the checks fail, the user is notified and it is not possible to start the mission (the vehicle will switch to [Hold mode](../flight_modes_mc/hold.md) instead of Mission mode).

Підмножина найважливіших перевірок перерахована нижче:

- Будь-який елемент місії конфліктує з планом або безпечним геозахистом
- Відсутній пункт зльоту та/або посадки, коли вони налаштовані як вимога ([MIS_TKO_LAND_REQ](#MIS_TKO_LAND_REQ))

Additionally there is a check if the first waypoint is too far from the Home position ([MIS_DIST_1WP](#MIS_DIST_1WP)).
The user is notified should the check fail, but it has no effect on the validity of a mission plan, meaning that the mission can still be started even if the distance is too high.

## QGroundControl Підтримка

_QGroundControl_ надає додаткову підтримку обробки місій на рівні GCS (на додачу до того, що надає контролер польоту).

Для додаткової інформації дивіться:

- [Видалити місію після посадки транспортного засобу](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/releases/stable_v3.2_long.html#remove-mission-after-vehicle-lands)
- [Відновити місію після режиму Повернення](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/releases/stable_v3.2_long.html#resume-mission)

## Параметри місії

Поведінка місій залежить від ряду параметрів, більшість з яких задокументовані в [Довідник параметрів > Місія](../advanced_config/parameter_reference.md#mission).
Дуже маленька підмножина наведені нижче.

Загальні параметри:

| Параметр                                                                                                                                                                | Опис                                                                                                                                                                                                                                                                                                                                                      |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="NAV_RCL_ACT"></a>[NAV_RCL_ACT](../advanced_config/parameter_reference.md#NAV_RCL_ACT)                                  | Режим аварійного відновлення зв'язку RC (що робить транспортний засіб, якщо втрачає зв'язок RC) - наприклад, увійти в режим утримання, режим повернення, завершити тощо.                                                                                                                                               |
| <a id="COM_RC_OVERRIDE"></a>[COM_RC_OVERRIDE](../advanced_config/parameter_reference.md#COM_RC_OVERRIDE)                      | Контролює переміщення джойстика на мультикоптері (або конвертоплані у режимі MC) повертає керування пілоту в [Режим положення](../flight_modes_mc/position.md). Це можна окремо увімкнути для автоматичних режимів та для режиму поза бортом, і в автоматичних режимах воно включено за замовчуванням. |
| <a id="COM_RC_STICK_OV"></a>[COM_RC_STICK_OV](../advanced_config/parameter_reference.md#COM_RC_STICK_OV) | Кількість рухів стиків, яка викликає перехід у [режим Положення](../flight_modes_mc/position.md) (якщо [COM_RC_OVERRIDE](#COM_RC_OVERRIDE) увімкнено).                                                                                                                       |

Параметри, пов'язані з [перевірками можливостей місії](#mission-feasibility-checks):

| Параметр                                                                                                                                                                   | Опис                                                                                                                                                                            |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="MIS_DIST_1WP"></a>[MIS_DIST_1WP](../advanced_config/parameter_reference.md#MIS_DIST_1WP)                                  | There is a warning message if the distance of the first waypoint to Home is more than this value. Вимкнено, якщо значення дорівнює 0 або менше. |
| <a id="FW_LND_ANG"></a>[FW_LND_ANG](../advanced_config/parameter_reference.md#FW_LND_ANG)                                        | Максимальний кут нахилу підйому.                                                                                                                                |
| <a id="MIS_TKO_LAND_REQ"></a>[MIS_TKO_LAND_REQ](../advanced_config/parameter_reference.md#MIS_TKO_LAND_REQ) | Sets whether mission _requires_ takeoff and/or landing items. No requirement by default for multicopter.                                        |

## Mission Commands {#mission_commands}

PX4 "приймає" наступні команди місії MAVLink у режимі Місії (з деякими _попередженнями_, які наведені після списку).
Якщо не вказано інше, реалізація відповідає визначенню у специфікації MAVLink.

Предмети місії:

- [MAV_CMD_NAV_WAYPOINT](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT)
  - _Param3_ (проліт) ігнорується. Flythrough завжди ввімкнено, якщо _param 1_ (time_inside) > 0.
- [MAV_CMD_NAV_LOITER_UNLIM](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_UNLIM)
- [MAV_CMD_NAV_LOITER_TIME](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_TIME)
- [MAV_CMD_NAV_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND)
- [MAV_CMD_NAV_TAKEOFF](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF)
- [MAV_CMD_NAV_LOITER_TO_ALT](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_TO_ALT)
- [MAV_CMD_DO_JUMP](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_JUMP)
- [MAV_CMD_NAV_ROI](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_ROI)
- [MAV_CMD_DO_SET_ROI](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI)
- [MAV_CMD_DO_SET_ROI_LOCATION](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_LOCATION)
- [MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET)
- [MAV_CMD_DO_SET_ROI_NONE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_NONE)
- [MAV_CMD_DO_CHANGE_SPEED](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED)
- [MAV_CMD_DO_SET_HOME](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_HOME)
- [MAV_CMD_DO_SET_SERVO](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO)
- [MAV_CMD_DO_LAND_START](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_LAND_START)
- [MAV_CMD_DO_TRIGGER_CONTROL](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_TRIGGER_CONTROL)
- [MAV_CMD_DO_DIGICAM_CONTROL](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_DIGICAM_CONTROL)
- [MAV_CMD_DO_MOUNT_CONFIGURE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONFIGURE)
- [MAV_CMD_DO_MOUNT_CONTROL](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONTROL)
- [MAV_CMD_IMAGE_START_CAPTURE](https://mavlink.io/en/messages/common.html#MAV_CMD_IMAGE_START_CAPTURE)
- [MAV_CMD_IMAGE_STOP_CAPTURE](https://mavlink.io/en/messages/common.html#MAV_CMD_IMAGE_STOP_CAPTURE)
- [MAV_CMD_VIDEO_START_CAPTURE](https://mavlink.io/en/messages/common.html#MAV_CMD_VIDEO_START_CAPTURE)
- [MAV_CMD_VIDEO_STOP_CAPTURE](https://mavlink.io/en/messages/common.html#MAV_CMD_VIDEO_STOP_CAPTURE)
- [MAV_CMD_DO_SET_CAM_TRIGG_DIST](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_CAM_TRIGG_DIST)
- [MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL)
- [MAV_CMD_SET_CAMERA_MODE](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_MODE)
- [MAV_CMD_NAV_DELAY](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_DELAY)
- [MAV_CMD_NAV_RETURN_TO_LAUNCH](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_RETURN_TO_LAUNCH)
- [MAV_CMD_DO_CONTROL_VIDEO](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CONTROL_VIDEO)
- [MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)
- [MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE)
- [MAV_CMD_OBLIQUE_SURVEY](https://mavlink.io/en/messages/common.html#MAV_CMD_OBLIQUE_SURVEY)
- [MAV_CMD_SET_CAMERA_ZOOM](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_ZOOM)
- [MAV_CMD_SET_CAMERA_FOCUS](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_FOCUS)
- [MAV_CMD_NAV_VTOL_TAKEOFF](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_VTOL_TAKEOFF)
  - `MAV_CMD_NAV_VTOL_TAKEOFF.param2` (заголовок переходу) ігнорується.
    Замість цього напрямок до наступної маршрутної точки використовується для переходу. <!-- at LEAST until PX4 v1.13: https://github.com/PX4/PX4-Autopilot/issues/12660 -->

Визначення GeoFence

- [MAV_CMD_NAV_FENCE_RETURN_POINT](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_RETURN_POINT)
- [MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION)
- [MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION)
- [MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION)
- [MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION)

Точки збору

- [MAV_CMD_NAV_RALLY_POINT](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_RALLY_POINT)

:::info
Please add an issue report or PR if you find a missing/incorrect message.

- PX4 аналізує вищезазначені повідомлення, але на них не обов'язково _реагує_. Наприклад, деякі повідомлення є специфічними для типу транспортного засобу.
- PX4 не підтримує локальні координати для команд місій (наприклад, [MAV_FRAME_LOCAL_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_NED)).
- Не всі повідомлення/команди доступні через _QGroundControl_.
- Список може стати застарілим, оскільки додаються повідомлення.
  Ви можете перевірити поточний набір, оглянувши код.
  Підтримка - `MavlinkMissionManager::parse_mavlink_mission_item` у [/src/modules/mavlink/mavlink_mission.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_mission.cpp).

:::

## Mission Command Timeouts

Some mission commands/items can take time to complete, such as a gripper opening and closing, a winch extending or retracting, or a gimbal moving to point at a region of interest.

Where provided PX4 may use sensor feedback from the hardware to determine when the action has completed and then move to the next mission item.
If not provided, or if the feedback is lost, a mission command timeout can be used to ensure that these kinds of actions will progress to the next mission item rather than blocking progression.

The timeout is set using the [MIS_COMMAND_TOUT](../advanced_config/parameter_reference.md#MIS_COMMAND_TOUT) parameter.
This should be set to be a small amount greater than the time required for the longest long-running action in the mission to complete.

## Закруглені повороти: Траєкторія міжточкового маршруту

PX4 очікує пряму лінію від попередньої точки маршруту до поточної цілі (він не планує будь-якого іншого шляху між точками маршруту - якщо вам потрібен такий, ви можете симулювати це додаванням додаткових точок маршруту).

Технічні засоби керування MC будуть змінювати _швидкість_ при наближенні або виходженні з точки відповідно до налаштувань [обмеження ривків](../config_mc/mc_jerk_limited_type_trajectory.md#auto-mode).
Транспортний засіб буде слідувати плавною округлою кривою до наступної точки шляху (якщо визначено) визначеною радіусом прийняття ([NAV_ACC_RAD](../advanced_config/parameter_reference.md#NAV_ACC_RAD)).
Діаграма нижче показує види шляхів, які ви можете очікувати.

![acc-rad](../../assets/flying/acceptance_radius_mission.png)

Транспортні засоби переключаються на наступну точку шляху, як тільки вони потрапляють в радіус прийняття ([NAV_ACC_RAD](../advanced_config/parameter_reference.md#NAV_ACC_RAD)).

## Місія зліт

Заплануйте місію зльоту мультикоптера, додавши елемент місії `TAKEOFF` на карту (це відповідає [MAV_CMD_NAV_TAKEOFF](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF) команді MAVLink).

Під час виконання цієї місії транспортний засіб підніметься вертикально до мінімальної висоти взяття на озброєння, визначеної в параметрі [MIS_TAKEOFF_ALT](../advanced_config/parameter_reference.md#MIS_TAKEOFF_ALT), а потім рушить у напрямку 3D-позиції, визначеної у елементі місії.

Якщо місія без виконання завдань стартує, транспортний засіб підійде на мінімальну висоту взльоту, а потім перейде до першого елементу місії `Waypoint`.

Якщо транспортний засіб вже знаходиться в повітрі під час початку місії, місія зльоту розглядається як звичайний точка шляху.

## Дивіться також

- [Місії](../flying/missions.md)
  - [Місія доставки посилок](../flying/package_delivery_mission.md)
- [Режим Місії (FW)](../flight_modes_fw/mission.md)
