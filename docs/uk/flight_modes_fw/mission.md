# Режим місії (фіксоване крило)

<img src="../../assets/site/position_fixed.svg" title="Global position fix required (e.g. GPS)" width="30px" />

_Режим місії_ змушує транспортний засіб виконувати передбачений автономний [план місії](../flying/missions.md) (план польоту), який був завантажений до керуючого пристрою польоту.
Зазвичай місія створюється та завантажується за допомогою програми для керування наземною станцією (GCS), такої як [QGroundControl](https://docs.qgroundcontrol.com/master/en/) (QGC).

::: info

- Цей режим потребує глобальної оцінки 3D-позиції (з GPS або виведеної з [локальної позиції](../ros/external_position_estimation.md#enabling-auto-modes-with-a-local-position)).
- Транспортний засіб повинен бути озброєний перед тим, як цей режим може бути активований.
- Цей режим є автоматичним - для керування автомобілем не потрібно втручання користувача.
- Перемикачі керування RC можуть використовуватися для зміни режимів польоту на будь-якому транспортному засобі.

:::

## Опис

Місії зазвичай створюються в земній контрольній станції (наприклад, [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/plan_view.html)) та завантажуються перед запуском.
Вони також можуть бути створені за допомогою розробника API або завантажені під час польоту.

[Команди місії](#mission-commands) обробляються таким чином, який є відповідним для кожної характеристики польоту з фіксованим крилом (наприклад, обертання виконується у вигляді польоту по колу).

:::info
Місії завантажуються на SD-карту, яку потрібно вставити **перед** запуском автопілота.
:::

На високому рівні всі типи транспортних засобів ведуть себе однаково, коли ввімкнено режим МІСІЯ:

1. Якщо місія не збережена, або якщо PX4 завершив виконання всіх команд місії, або якщо [місія не є можливою](#mission-feasibility-checks):

  - Якщо літає транспортний засіб, він буде марнувати час.
  - Якщо посадять транспортний засіб, він буде "чекати".

2. Якщо місія збережена, а PX4 летить, вона виконає [місію / план польоту](../flying/missions.md) з поточного кроку.
  - Пункт відправлення буде розглядатися як звичайна точка місії.

3. Якщо місія збережена, а транспортний засіб приземлений, він злетить лише у випадку, якщо активна точка маршруту - це 'Зльот'.
  Якщо налаштовано для запуску з катапульта, транспортний засіб також повинен бути запущений (див. [Зліт/посадка FW у місії](#mission-takeoff)).

4. Якщо жодне завдання не збережено, або якщо PX4 завершив виконання всіх команд місії:
  - Якщо літає транспортний засіб, він буде марнувати час.
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

Місії можна призупинити, переключившись з режиму місії на будь-який інший режим (наприклад, [режим утримання](../flight_modes_fw/hold.md) або [режим позиціонування](../flight_modes_fw/position.md)), і продовжити, переключившись назад в режим місії.
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
- Визначено більше одного елемента місії початку посадки ([MAV_CMD_DO_LAND_START](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_LAND_START))
- Посадка на фіксованих крилах має неможливий кут нахилу схилу ([FW_LND_ANG](#FW_LND_ANG))
- Пункт початку посадки на землю (`MAV_CMD_DO_LAND_START`) з'являється в місії перед пунктом RTL ([MAV_CMD_NAV_RETURN_TO_LAUNCH](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_RETURN_TO_LAUNCH))
- Відсутній пункт зльоту та/або посадки, коли вони налаштовані як вимога ([MIS_TKO_LAND_REQ](#MIS_TKO_LAND_REQ))

Additionally there is a check if the first waypoint is too far from the Home position ([MIS_DIST_1WP](#MIS_DIST_1WP)).
The user is notified should the check fail, but it has no effect on the validity of a mission plan (the mission can still be started even if the distance is too great).

## QGroundControl Підтримка

_QGroundControl_ надає додаткову підтримку обробки місій на рівні GCS (на додачу до того, що надає контролер польоту).

Для додаткової інформації дивіться:

- [Видалити місію після посадки транспортного засобу](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/releases/stable_v3.2_long.html#remove-mission-after-vehicle-lands)
- [Відновити місію після режиму Повернення](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/releases/stable_v3.2_long.html#resume-mission)

## Параметри місії

Поведінка місій залежить від ряду параметрів, більшість з яких задокументовані в [Довідник параметрів > Місія](../advanced_config/parameter_reference.md#mission).
Дуже маленька підмножина наведені нижче.

Загальні параметри:

| Параметр                                                                                                                                     | Опис                                                                                                                                                                                                        |
| -------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="NAV_RCL_ACT"></a>[NAV_RCL_ACT](../advanced_config/parameter_reference.md#NAV_RCL_ACT)       | Режим аварійного відновлення зв'язку RC (що робить транспортний засіб, якщо втрачає зв'язок RC) - наприклад, увійти в режим утримання, режим повернення, завершити тощо. |
| <a id="NAV_LOITER_RAD"></a>[NAV_LOITER_RAD](../advanced_config/parameter_reference.md#NAV_RCL_ACT) | Фіксований радіус утримання крил.                                                                                                                                                           |

Параметри, пов'язані з [перевірками можливостей місії](#mission-feasibility-checks):

| Параметр                                                                                                                                                                   | Опис                                                                                                                                                                                                   |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="MIS_DIST_1WP"></a>[MIS_DIST_1WP](../advanced_config/parameter_reference.md#MIS_DIST_1WP)                                  | There is a warning message if the distance of the first waypoint to Home is more than this value. Вимкнено, якщо значення дорівнює 0 або менше.                        |
| <a id="FW_LND_ANG"></a>[FW_LND_ANG](../advanced_config/parameter_reference.md#FW_LND_ANG)                                        | Максимальний кут нахилу підйому.                                                                                                                                                       |
| <a id="MIS_TKO_LAND_REQ"></a>[MIS_TKO_LAND_REQ](../advanced_config/parameter_reference.md#MIS_TKO_LAND_REQ) | Sets whether mission _requires_ takeoff and/or landing items. FW та VTOL обидва мають його задано на 2 за замовчуванням, що означає, що місія повинна містити посадку. |

## Команди місій

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
- [MAV_CMD_DO_SET_CAMERA_ZOOM](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_CAMERA_ZOOM)
- [MAV_CMD_DO_SET_CAMERA_FOCUS](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_CAMERA_FOCUS)

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

Транспортний засіб буде слідувати плавною округлою кривою до наступної точки шляху (якщо визначено) визначеною радіусом прийняття ([NAV_ACC_RAD](../advanced_config/parameter_reference.md#NAV_ACC_RAD)).
Діаграма нижче показує види шляхів, які ви можете очікувати.

![acc-rad](../../assets/flying/acceptance_radius_mission.png)

Транспортні засоби переходять на наступну точку шляху, як тільки вони увійшли в радіус прийняття.
Це визначається "відстанню L1", яка обчислюється з двох параметрів: [NPFG_DAMPING](../advanced_config/parameter_reference.md#NPFG_DAMPING) та [NPFG_PERIOD](../advanced_config/parameter_reference.md#NPFG_PERIOD), та поточною швидкістю на землі.
За замовчуванням, це приблизно 70 метрів.

Рівняння:

$$L_{1_{distance}}=\frac{1}{\pi}L_{1_{damping}}L_{1_{period}}\left \| \vec{v}_{ {xy}_{ground} } \right \|$$

## Місія зліт

Початок польотів з місією зльоту (і посадка за допомогою місії посадки) є рекомендованим способом автономної роботи літака.

Обидва [зліт по злітній смузі](../flight_modes_fw/takeoff.md#runway-takeoff) та [ручний запуск злітний](../flight_modes_fw/takeoff.md#catapult-hand-launch) підтримуються — для інформації про конфігурацію див. [Режим зліт (FW)](../flight_modes_fw/takeoff.md).

Поведінка відбору визначена в елементі місій Takeoff, яка відповідає [MAV_CMD_NAV_TAKEOFF](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF).
Під час виконання місії транспортний засіб злетить до цієї точки маршруту та підніметься, поки не буде досягнута вказана висота.
Пункт місії потім приймається, і місія почне виконувати наступний пункт.

Конкретніше, пункт завдання зльоту визначає курс зльоту та висоту очищення.
Курс - це лінія між точкою старту транспортного засобу та горизонтальним положенням, визначеним у пункті місії, тоді як висота дозволу - це висота пункту місії.

Для зльоту зі злітної смуги, пункт місії `Takeoff` змусить транспортний засіб взберегтися, регулювати оберти моторів і злітати.
Під час ручного запуску транспортний засіб буде озброєний, але тільки збільшить швидкість, коли транспортний засіб буде кинутий (виявлено спусковий прискорювач).
У обох випадках транспортний засіб повинен бути розміщений (або запущений) у напрямку на візуальну точку вильоту при запуску місії.
Якщо можливо, завжди здійснюйте взліт транспортного засобу проти вітру.

:::info
A fixed-wing mission requires a `Takeoff` mission item to takeoff; if however the vehicle is already flying when the mission is started the takeoff item will be treated as a normal waypoint.
:::

Для отримання додаткової інформації про поведінку взльоту та конфігурацію див. [Режим взльоту (FW)](../flight_modes_fw/takeoff.md).

## Посадка місії

Посадка місії на літаку є рекомендованим способом автономної посадки літака.
Це можна запланувати в _QGroundControl_ за допомогою [шаблону посадки фіксованого крила](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/pattern_fixed_wing_landing.html).

Якщо це можливо, завжди плануйте приземлення так, щоб воно виконувало підхід проти вітру.

Наступні розділи описують послідовність посадки, відміну від посадки та коригування, питання безпеки та конфігурацію.

### Посадкова послідовність

Маршрут посадки складається з точки утримання ([MAV_CMD_NAV_LOITER_TO_ALT](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_TO_ALT)), за якою слідує точка посадки ([MAV_CMD_NAV_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND)).
Позиції двох точок визначають початкову та кінцеву точки підходу до посадки, а отже, кут звисання для підходу до посадки.

Ця схема призводить до наступної послідовності посадки:

1. **Літайте до місця посадки**: Літак летить на поточній висоті до точки обертання.
2. **Опускаючись на орбіту для наближення до висоти**: При досягненні радіусу утримання точки шляху, транспортний засіб виконує опускаючу орбіту до досягнення "висоти наближення" (висота точки утримання).
  Транспортний засіб продовжує обертатися на цій висоті до тих пір, поки він не матиме тангенціальну траєкторію до пункту наземної точки, після чого ініціюється посадковий захід.
3. **Приземлення**: Літак слідує нахилу під час приземлення до точки на землі до досягнення висоти флеру.
4. **Світло**: Транспортний засіб світиться, доки не сідає на землю.

![Fixed-wing landing](../../assets/flying/fixed-wing_landing.png)

### Прильот на посадку

Транспортний засіб відстежує кут посадки (зазвичай з меншою швидкістю, ніж під час круїзу) до досягнення висоти розгортання.

Зверніть увагу, що кут зниження обчислюється з тривимірних позицій точок обертання та посадки; якщо його кут перевищує параметр [FW_LND_ANG](#FW_LND_ANG), місія буде відхилена як неможлива для завантаження.

Параметри, які впливають на посадковий захід, перераховані нижче.

| Параметр                                                                                                                                      | Опис                                                                                                                                                                       |
| --------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="FW_LND_ANG"></a>[FW_LND_ANG](../advanced_config/parameter_reference.md#FW_LND_ANG)           | Максимальний досяжний кут нахилу під час посадки. Зверніть увагу, що менші кути все ще можуть бути вказані через пункт місії посадки.      |
| [FW_LND_EARLYCFG](../advanced_config/parameter_reference.md#FW_LND_EARLYCFG)                        | Необов'язково розгортати конфігурацію посадкового спуску під час посадкової орбіти (наприклад, закрилки, спойлери, швидкість посадки).  |
| [FW_LND_AIRSPD](../advanced_config/parameter_reference.md#FW_LND_AIRSPD)                            | Встановлена швидкість повітря під час посадки.                                                                                                             |
| [FW_FLAPS_LND_SCL](../advanced_config/parameter_reference.md#FW_FLAPS_LND_SCL) | Налаштування Flaps під час посадки.                                                                                                                        |
| [FW_LND_THRTC_SC](../advanced_config/parameter_reference.md#FW_LND_THRTC_SC)   | Фактор часової константи висоти для посадки (перевизначає типове [налаштування TECS](../config_fw/position_tuning_guide_fixedwing.md)). |

### Вогніння / Розгортання

Фларінг полягає в переключенні з відстеження висоти на маленький установлений показник швидкості опускання та обмеженнях на керований дросель, що призводить до підняття носа для сповільнення спуску та отримання більш м'якого приземлення.

Висота спалаху обчислюється під час останнього підходу через "час до впливу" ([FW_LND_FL_TIME](#FW_LND_FL_TIME)) та швидкість спуску на посадку.
Додатковий параметр безпеки [FW_LND_FLALT](#FW_LND_FLALT) встановлює мінімальну висоту, на якій транспортний засіб виконає флер (якщо висота на основі часу занадто низька для безпечного виконання флер-маневру).

Якщо посадка на живіт, транспортний засіб буде продовжувати перебувати в стані сповільнення до посадки, виявлення землі та подальше відключення. Для посадки на взлетно-посадочной полосе, [FW_LND_TD_TIME](#FW_LND_TD_TIME) дозволяє встановити час після початку спадання для опускання носа (наприклад, розглядайте шасі з триколісною підвіскою) на взлетно-посадочій смузі ([RWTO_PSP](#RWTO_PSP)) та уникнути стрибків. Цей час приблизно відповідає посадковому посту після спалаху, і його слід налаштувати для певної конструкції корпусу під час тільки після того, як спалах буде налаштований під час випробувань.

Параметри, які впливають на вогнення, перераховані нижче.

| Параметр                                                                                                                                                             | Опис                                                                                                                                                                                                                                                          |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="FW_LND_FL_TIME"></a>[FW_LND_FL_TIME](../advanced_config/parameter_reference.md#FW_LND_FL_TIME) | Час до удару (при поточній швидкості спуску), коли транспортний засіб повинен піднятися.                                                                                                                                   |
| <a id="FW_LND_FL_SINK"></a>[FW_LND_FL_SINK](../advanced_config/parameter_reference.md#FW_LND_FL_SINK) | Поверхотинна швидкість опускання літака буде слідувати під час розкриття.                                                                                                                                                                     |
| <a id="FW_LND_FLALT"></a>[FW_LND_FLALT](../advanced_config/parameter_reference.md#FW_LND_FLALT)                            | Мінімальна висота над землею, на якій повинен спрацювати шасі літака. Він використовується лише тоді, коли висота на основі часу занадто низька.                                                                              |
| <a id="FW_LND_FL_PMAX"></a>[FW_LND_FL_PMAX](../advanced_config/parameter_reference.md#FW_LND_FL_PMAX) | Максимальний допустимий кут нахилу під час зближення.                                                                                                                                                                                         |
| <a id="FW_LND_FL_PMIN"></a>[FW_LND_FL_PMIN](../advanced_config/parameter_reference.md#FW_LND_FL_PMIN) | Мінімально допустимий кут нахилення під час підйому (часто необхідний для уникнення командування від'ємними кутами нахилу для збільшення швидкості польоту, оскільки регулювання газа зменшується до нульового положення.) |
| <a id="FW_LND_TD_TIME"></a>[FW_LND_TD_TIME](../advanced_config/parameter_reference.md#FW_LND_TD_TIME) | Час після початку спалаху, коли транспортний засіб повинен опустити ніс.                                                                                                                                                                      |
| <a id="RWTO_PSP"></a>[RWTO_PSP](../advanced_config/parameter_reference.md#RWTO_PSP)                                                             | Налагодження польоту під час зльоту. Для шасі трициклів, зазвичай близько до нуля. Для літаків з хвостовим краденцем, позитивно.                                                                              |
| <a id="FW_THR_IDLE"></a>[FW_THR_IDLE](../advanced_config/parameter_reference.md#FW_THR_IDLE)                               | Встановлення планки холостого ходу. Транспортний засіб буде зберігати цей параметр протягом спалаху та розвороту.                                                                                                             |

### Відміна

#### Оператор відміни Abort

Приземлення може бути перервано оператором в будь-якій точці під час остаточного підходу з використанням команди [MAV_CMD_DO_GO_AROUND](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GO_AROUND).
Під час посадки на _QGroundControl_ висувається кнопка спливаючого вікна для активації цього.

Переривання посадки призводить до вибору курсу вище для формування орбіти над цільовою точкою на землі.
Максимальна висота поточної висоти літака та [MIS_LND_ABRT_ALT](#MIS_LND_ABRT_ALT) встановлюється як висота витягу абортного орбіту відносно (вище) пункту посадки.
Конфігурація посадки (наприклад, закрилки, спойлери, швидкість повітряного судна під час посадки) відключена під час спинення, і повітряне судно летить в умовах круїзу.

Команда відміни вимкнена під час спалаху для безпеки.
Оператори все ще можуть вручну припинити посадку, переключившись на будь-який ручний режим, такий як [Режим стабілізації](../flight_modes_fw/stabilized.md)), але варто зазначити, що це ризиковано!

#### Автоматичний вихід із системи

Автоматична логіка аварійного відміни додатково доступна для кількох умов, якщо налаштована.
Доступні автоматичні критерії перерви можуть бути увімкнені за допомогою параметра бітмаски [FW_LND_ABORT](#FW_LND_ABORT).
Один приклад автоматичних критеріїв аварійного відключення - це відсутність дійсного виміру діапазону від датчика відстані.

:::warning
Посадка без датчика відстані **строго** не рекомендується.
Вимкнення оцінки місцевості за допомогою [FW_LND_USETER](#FW_LND_USETER) та обрані біти [FW_LND_ABORT](#FW_LND_ABORT) призведе до видалення вимоги до датчика відстані за замовчуванням, але внаслідок цього спадає до висоти посадки GNSS для визначення висоти опускання, яка може бути кілька метрів занадто високо або занадто низько, що потенційно може призвести до пошкодження фюзеляжу.
:::

| Параметр                                                                                                                                                                   | Опис                                                                                                |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------- |
| <a id="MIS_LND_ABRT_ALT"></a>[MIS_LND_ABRT_ALT](../advanced_config/parameter_reference.md#MIS_LND_ABRT_ALT) | Мінімальна висота над точкою на землі, на яку може бути вказано відмову від орбіти. |
| <a id="FW_LND_ABORT"></a>[FW_LND_ABORT](../advanced_config/parameter_reference.md#FW_LND_ABORT)                                  | Визначає, які критерії автоматичної відмови увімкнені.                              |
| <a id="FW_LND_USETER"></a>[FW_LND_USETER](../advanced_config/parameter_reference.md#FW_LND_USETER)                               | Увімкнення використання датчика відстані під час фінального підходу.                |

### Торкання

У разі незначних розбіжностей ГНСС або карти, що викликають зміщення при підході, оператор може вносити невеликі ручні коригування до підходу до посадки та виїзду (за допомогою палиці руля), коли [FW_LND_NUDGE](../advanced_config/parameter_reference.md#FW_LND_NUDGE) увімкнено.
Варіанти включають виправлення кута підходу або повний шлях підходу.

У обох випадках транспортний засіб залишається в режимі повного автоматичного керування, відстежуючи зміщений вектор підходу.
[FW_LND_TD_OFF](../advanced_config/parameter_reference.md#FW_LND_TD_OFF) дозволяє визначити, на яку відстань вліво або вправо від пункту посадки може бути зміщена прогнозована точка дотику.
Введення палиці Yaw відповідає натисканню "швидкості".
Після відпускання палиці (нульова швидкість), шлях або кут підходу зупиниться відхилятися.

![Торкання при посадці на фіксованому крилі](../../assets/flying/fixed-wing_landing_nudge.png)

Підхідний шлях зафіксований, як тільки починається закручування.
Якщо виконується посадка на злітній смузі з керованим переднім колесом, команда керування починається безпосередньо на переднє колесо з початку флеру, під час виїзду, до відключення.
Зверніть увагу, що якщо контролер колеса увімкнено ([FW_W_EN](#FW_W_EN)), контролер активно намагатиметься керувати транспортним засобом до шляху підходу, тобто "боротьба" з введеннями оператора посадки.

:::info
Відштовхування (Nudging) не повинно використовуватися для доповнення поганого налаштування контролю позиції.
Якщо транспортний засіб постійно показує погану роботу слідування по визначеній траєкторії, будь ласка, зверніться до [керівництва з настройки керування фіксованим крилом](../flight_modes_fw/position.md) за інструкціями.
:::

| Параметр                                                                                                                                                          | Опис                                                                                         |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------- |
| <a id="FW_LND_NUDGE"></a>[FW_LND_NUDGE](../advanced_config/parameter_reference.md#FW_LND_NUDGE)                         | Увімкнути рух управляння для посадки літака з нерухомим крилом.              |
| <a id="FW_LND_TD_OFF"></a>[FW_LND_TD_OFF](../advanced_config/parameter_reference.md#FW_LND_TD_OFF) | Налаштувати допустиме бічне зміщення посадки від командованої точки посадки. |
| <a id="FW_W_EN"></a>[FW_W_EN](../advanced_config/parameter_reference.md#FW_W_EN)                                        | Увімкніть контролер керування передніми колесами.                            |

### Обмеження безпеки на низькій висоті

У режимі посадки використовується датчик відстані для визначення близькості до землі, а геометрія підфрейму використовується для розрахунку обмежень кочення для запобігання удару крилом.

![Посадка літака з фіксованим криломТоркання(../../assets/flying/wing_geometry.png)

| Параметр                                                                                                             | Опис                                                                                                    |
| -------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------- |
| [FW_WING_SPAN](../advanced_config/parameter_reference.md#FW_WING_SPAN)     | Розмах крила каркасу.                                                                   |
| [FW_WING_HEIGHT](../advanced_config/parameter_reference.md#FW_WING_HEIGHT) | Висота крила від нижньої частини шасі (або живота, якщо немає шасі). |

## Дивіться також

- [Місії](../flying/missions.md)
  - [Місія доставки посилок](../flying/package_delivery_mission.md)
- [Режим Місії (MC)](../flight_modes_mc/mission.md)
