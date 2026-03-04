# Місії

A mission is a predefined flight plan, which can be planned in QGroundControl and uploaded to the flight controller, and then executed autonomously in [Mission mode](../flight_modes_mc/mission.md).

Місії, як правило, включають елементи для контролю зльоту, польоту в послідовності маршрутних точок, знімання зображень та/або відео, розміщення вантажу та посадки.
QGroundControl дозволяє планувати місії, використовуючи повністю ручний підхід, або ви можете використовувати його більш розширені функції для планування обстежень місцевості, коридорів або оглядів структур.

Цей розділ надає огляд того, як планувати та виконувати місії.

## Планування місій

Планування місій вручну є простим процесом:

- Перейдіть до екрана місії
- Select the **Add Waypoint** ("plus") icon in the top left.
- Натисніть на карту, щоб додати точки маршруту.
- Use the waypoint list on the right to modify the waypoint parameters/type
  The altitude indicator on the bottom provides a sense of the relative altitude of each waypoint.
- Once finished, click on the **Upload** button (top right) to send the mission to the vehicle.

You can also use the _Pattern_ tool to automate creation of survey grids.

:::tip
For more information see the [QGroundControl User Guide](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/plan_view.html).
:::

![planning-mission](../../assets/flying/planning_mission.jpg)

### Перевірки можливостей місії

PX4 виконує деякі базові перевірки, щоб визначити, чи місія є виконуваною.
Наприклад, чи достатньо близько місія до апарату, чи буде місія конфліктувати з геозоною, або чи потрібен патерн посадки для місії, але він відсутній.

Перевірки виконуються під час завантаження місії та безпосередньо перед її запуском.
Якщо будь-яка з перевірок не пройде успішно, користувач отримує повідомлення, і почати місію неможливо.

For more detail on the checks and possible actions, see: [Mission Mode (FW) > Mission Feasibility Checks](../flight_modes_fw/mission.md#mission-feasibility-checks) and [Mission Mode (MC) > Mission Feasibility Checks](../flight_modes_mc/mission.md#mission-feasibility-checks).

### Налаштування кута повороту апарату

If set, a multi-rotor vehicle will yaw to face the **Heading** value specified in the target waypoint (corresponding to [MAV_CMD_NAV_WAYPOINT.param4](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT)).

If **Heading** has not been explicitly set for the target waypoint (`param4=NaN`) then the vehicle will yaw towards a location specified in the parameter [MPC_YAW_MODE](../advanced_config/parameter_reference.md#MPC_YAW_MODE).
За замовчуванням це наступна точка маршруту.

Типи апаратів, які не можуть самостійно контролювати поворот і напрямок руху, ігноруватимуть налаштування повороту (наприклад, апарати з фіксованим крилом).

### Налаштування радіусу прийняття/повороту

The _acceptance radius_ defines the circle around a waypoint within which a vehicle considers it has reached the waypoint, and will immediately switch to (and start turning towards) the next waypoint.

For a multi-rotor drones, the acceptance radius is tuned using the parameter [NAV_ACC_RAD](../advanced_config/parameter_reference.md#NAV_ACC_RAD).
За замовчуванням радіус є малим, щоб гарантувати, що мультикоптери пролітають над точками маршруту, але його можна збільшити, щоб створити більш плавний шлях, таким чином, щоб дрон починав поворот до досягнення точки маршруту.

Зображення нижче показує одну й ту саму місію, виконану з різними параметрами радіусу прийняття:

![acceptance radius comparison](../../assets/flying/acceptance_radius_comparison.jpg)

The speed in the turn is automatically computed based on the acceptance radius (= turning radius) and the maximum allowed acceleration and jerk (see [Jerk-limited Type Trajectory for Multicopters](../config_mc/mc_jerk_limited_type_trajectory.md#auto-mode)).

:::tip
For more information about the impact of the acceptance radius around the waypoint see: [Mission Mode > Inter-waypoint Trajectory](../flight_modes_fw/mission.md#rounded-turns-inter-waypoint-trajectory).
:::

### Місії з доставки посилок (вантажу)

PX4 підтримує доставку вантажу в місіях за допомогою захвату.

This kind of mission is planned in much the same as any other [waypoint mission](../flying/missions.md), with mission start, takeoff waypoint, various path waypoints, and possibly a return waypoint.
Єдина відмінність полягає в тому, що місія з доставки посилки повинна включати елементи місії, що вказують, як відбувається випуск посилки та механізм вивантаження.
For more information see: [Package Delivery Mission](../flying/package_delivery_mission.md).

## Виконання місій

Після того, як місію буде завантажено, перейдіть до екрана польоту.
Місія відображається таким чином, що дозволяє легко відстежувати прогрес (його не можна змінити на цьому екрані).

![flying-mission](../../assets/flying/flying_mission.jpg)
