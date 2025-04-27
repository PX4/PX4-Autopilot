# Камери MAVLink (Протокол камери v2)

Ця тема пояснює, як використовувати PX4 з MAVLink [камера](../camera/index.md), що реалізує [MAVLink Camera Protocol v2] (https://mavlink.io/en/services/camera.html) з PX4 та Наземною Станцією.

:::tip
Це рекомендований спосіб інтеграції камери з PX4!
:::

## Загальний огляд

[Протокол камери MAVLink v2](https://mavlink.io/en/services/camera.html) дозволяє запитувати, які функції підтримуються камерою, і надає команди для керування захопленням зображень та відео, передачі відео, встановлення зуму та фокусу, вибору між інфрачервоними та видимими потоками світла, встановлення місця, куди зберігаються зібрані дані, та інше.

Камера може реалізувати протокол на рівні системи, але більшість налаштувань камер MAVLink передбачають взаємодію PX4 з [менеджером камер](#camera-managers), що працює на супутниковому комп'ютері, який потім взаємодіє між MAVLink та вихідним протоколом камери.

Загалом можна сказати, що "інтеграція" PX4 з камерою полягає в повторному відтворенні команд камери, знайдених у місіях за допомогою протоколу команд.
В іншому випадку вона може діяти як міст, пересилаючи команди між земною станцією та камерою, якщо немає прямого каналу MAVLink.

:::info
PX4 не підтримує використання команд [MAVLink Протокол Камери v2](https://mavlink.io/en/services/camera.html) для управління камерами, які підключені до виходів контролера польоту.
Хоча це технічно можливо, це вимагатиме від PX4 реалізації інтерфейсу керування камерою.
:::

## Керування камерою

### Команди та повідомлення MAVLink

Камери виявлені за допомогою протоколу підключення MAVLink (https://mavlink.io/en/services/heartbeat.html), заснованого на їх [HEARTBEAT.type](https://mavlink.io/en/messages/common.html#HEARTBEAT), заснованими на MAV_TYPE_CAMERA (https://mavlink.io/en/messages/common.html#MAV_TYPE_CAMERA).

:::tip
Камери також повинні використовувати ID компонента в рекомендованому діапазоні, такому як [MAV_COMP_ID_CAMERA](https://mavlink.io/en/messages/common.html#MAV_COMP_ID_CAMERA), але, як правило, на це не можна покладатися для перевірки того, що компонент MAVLink є камерою.
:::

Якщо камера виявлена, можна подивитися її властивості та можливості за допомогою [MAV_CMD_REQUEST_MESSAGE](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE), щоб запросити повідомлення [CAMERA_INFORMATION](https://mavlink.io/en/messages/common.html#CAMERA_INFORMATION), а потім інспектувати поле `flags`, щоб визначити, які стандартні функції в [CAMERA_CAP_FLAGS](https://mavlink.io/en/messages/common.html#CAMERA_CAP_FLAGS) підтримуються.

На основі прапорців ви можете визначити, які інші команди та повідомлення підтримуються камерою.
Повний набір повідомлень, команд та констант [узагальнено тут](https://mavlink.io/en/services/camera.html#messagecommandenum-summary).

Додаткові параметри камери_можуть_бути викриті у [файлі-визначенні камери](https://mavlink.io/en/services/camera_def.html), який зв'язаний з `CAMERA_INFORMATION.cam_definition_uri`.
GCS або SDK можуть відкрити ці параметри через загальний інтерфейс користувача, не маючи розуміння будь-якого контексту.
Ці параметри не можуть бути встановлені безпосередньо в місіях і не мають конкретних команд-встановлювачів.

[Протокол камери MAVLink v2](https://mavlink.io/en/services/camera.html) описує всі взаємодії більш детально.

### Наземні станції & MAVLink SDKS

Наземні станції та MAVLink SDK виявляють камери та їх можливості, як описано в попередньому розділі.

Наземна станція може використовувати будь-яку функцію, яку надає камера.
PX4 не має жодної ролі в цій взаємодії, крім пересилання трафіку MAVLink між камерою та наземною станцією або SDK, якщо потрібно.

### Команди камери у місіях

PX4 дозволяє використовувати наступний піднабір команд [Camera Protocol v2](https://mavlink.io/en/services/camera.html) під час місій:

- [MAV_CMD_IMAGE_START_CAPTURE](https://mavlink.io/en/messages/common.html#MAV_CMD_IMAGE_START_CAPTURE)
- [MAV_CMD_IMAGE_STOP_CAPTURE](https://mavlink.io/en/messages/common.html#MMAV_CMD_IMAGE_STOP_CAPTURE)
- [MAV_CMD_VIDEO_START_CAPTURE](https://mavlink.io/en/messages/common.html#MAV_CMD_VIDEO_START_CAPTURE)
- [MAV_CMD_VIDEO_STOP_CAPTURE](https://mavlink.io/en/messages/common.html#MAV_CMD_VIDEO_STOP_CAPTURE)
- [MAV_CMD_SET_CAMERA_MODE](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_MODE)
- [MAV_CMD_SET_CAMERA_ZOOM](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_ZOOM)
- [MAV_CMD_SET_CAMERA_FOCUS](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_FOCUS)

PX4 відновлює команди камери, знайдені в місіях, як команди MAVLink.
Системний ідентифікатор випущених команд співпадає з ідентифікатором автопілота.
Ідентифікатор компонента команд може відрізнятися.
Перші чотири команди адресовані [MAV_COMP_ID_CAMERA (100)](https://mavlink.io/en/messages/common.html#MAV_COMP_ID_CAMERA) (якщо у камери є цей ідентифікатор компонента, вона виконає вказану команду).
Режим камери, зум та фокусування, команди надсилаються до компонента з ідентифікатором [MAV_COMP_ID_ALL](https://mavlink.io/en/messages/common.html#MAV_COMP_ID_ALL).

:::info
PX4 в даний час ігнорує ідентифікатор цілі камери `id` в [MAV_CMD_IMAGE_START_CAPTURE](https://mavlink.io/en/messages/common.html#MAV_CMD_IMAGE_START_CAPTURE) та інших повідомленнях камери.
Див. [PX4-Autopilot#23083](https://github.com/PX4/PX4-Autopilot/issues/23083).
:::

<!--
List of all supported commands in missions in:
format_mavlink_mission_item() => https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_mission.cpp#L1672-L1693

Mission items are executed when set active.
void Mission::setActiveMissionItems() => https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/mission.cpp#L187-L281
  At end the current non-waypoint command is "issued":
  note at end => issue_command(_mission_item);

Issuing command:
MissionBlock::issue_command(const mission_item_s &item) =>  https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/mission_block.cpp#L543-L562
  At end this publishes the current vehicle command
  _navigator->publish_vehicle_command(&vehicle_command);

Publishing command:
void Navigator::publish_vehicle_command(vehicle_command_s &vehicle_command)  => https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/navigator_main.cpp#L1395
  For camera commands set to vehicle_command.target_component = 100; // MAV_COMP_ID_CAMERA
  All others just get published as-is
-->

### Ручне керування

Стіки джойстика можуть бути налаштовані на спрацьовування захоплення зображення або перемикання захоплення відео.

PX4 видає команди [MAVLink Camera Protocol v2](https://mavlink.io/en/services/camera.html), такі як `MAV_CMD_IMAGE_START_CAPTURE`, коли натиснута відповідна кнопка джойстика.
Ця функція працює лише для цього типу камери та джойстика - підтримки для радіокерованних-контролерів немає.

## Конфігурація PX4

### Налаштування порту & перенаправлення MAVLink

Вам потрібно буде надати канал MAVLink для будь-яких підключених камер, щоб PX4 міг висилати будь-які команди камери, знайдені в місіях.
Якщо ваша мережа MAVLink така, що PX4 є "між" вашою камерою та наземною станцією, вам також потрібно буде пересилати комунікації, щоб вони могли спілкуватися.

Спочатку під'єднайте камеру до не використаного послідовного порту на контролері польоту, такого як `TELEM2` (ви також можете використовувати порт Ethernet, якщо він присутній як на контролері польоту, так і на камері).
Потім налаштуйте обраний порт як [MAVLink Peripheral](../peripherals/mavlink_peripherals.md).

Документ пояснює, як, але коротко:

1. Змініть невикористаний параметр `MAV_n_CONFIG`, такий як [MAV_2_CONFIG](../advanced_config/parameter_reference.md#MAV_2_CONFIG), щоб він був присвоєний порту, до якого підключена ваша камера/компаньйонський комп'ютер.
2. Встановіть відповідний [MAV_2_MODE](../advanced_config/parameter_reference.md#MAV_2_MODE) на `2` (На борту).
  Це забезпечує, що правильний набір повідомлень MAVLink випромінюється для супутнього комп'ютера (або камери).
3. Встановіть [MAV_2_FORWARD](../advanced_config/parameter_reference.md#MAV_2_FORWARD), щоб дозволити пересилання комунікацій з порту на інші порти, такі як той, що підключений до наземної станції.
4. Можливо, вам доведеться встановити деякі інші параметри, залежно від типу підключення та будь-яких конкретних вимог камери щодо очікуваної швидкості передачі даних і т. д.

### Ручне керування

Кнопки джойстика можуть бути налаштовані для захоплення зображень та перемикання запису відео включено/виключено.

- [Joystick](../config/joystick.md#enabling-px4-joystick-support) пояснює, як увімкнути джойстики на PX4.
- [QGroundControl > Налаштування джойстика](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/joystick.html) пояснює, як задати кнопки на функції стеку польоту

<!-- Cameras cannot be controlled from an RC controller as far as I can tell -->

## Менеджери камер

Якщо ви хочете використовувати камеру, яка не підтримує протокол камери MAVLink за замовчуванням, вам знадобиться менеджер камери MAVLink.
Менеджер камери працює на супутниковому комп'ютері та з'єднує інтерфейс протоколу камери MAVLink і нативний інтерфейс камери.

Існують "розширювані" менеджери камер, якими можна користуватися з багатьма різними камерами, менеджери камер, розроблені для роботи з певною камерою, а також ви можете написати свій власний (наприклад, використовуючи додатки сервера MAVSDK).

Загальні/розширені менеджери камер:

- [Менеджер камери MAVLink](https://github.com/mavlink/mavlink-camera-manager) - Розширюваний крос-платформенний сервер камери MAVLink, побудований на базі GStreamer та Rust-MAVLink.
- [Менеджер камери Dronecode](https://camera-manager.dronecode.org/en/) - Додає інтерфейс протоколу камери для камер, підключених до комп'ютера з Linux.

Специфічні менеджери камери:

- [Менеджер камери SIYI A8 mini](https://github.com/julianoes/siyi-a8-mini-camera-manager) - Менеджер камери на основі плагіна MAVSDK для [SIYI A8 mini] (включає навчальний посібник).

  ::: tip
  Це добрий приклад того, як MAVSDK може бути використаний для створення інтерфейсу протоколу камери MAVLink для певної камери.

:::

Під час використання менеджера камери ви підключаєте компаньйон-комп'ютер до контролера польоту (замість прямого підключення до камери), і вам знадобиться додаткове програмне забезпечення на комп'ютері для направлення трафіку MAVLink до менеджера камери на компаньйон-комп'ютері, таке як [mavlink-router](https://github.com/mavlink-router/mavlink-router).

Додаткову інформацію щодо камерного менеджера та налаштувань компаньйона можна знайти в:

- [Менеджер камери SIYI A8 mini](https://github.com/julianoes/siyi-a8-mini-camera-manager) - Навчальний посібник з інтеграції з [SIYI A8 mini](https://shop.siyi.biz/products/siyi-a8-mini), використовуючи менеджер камери на основі MAVSDK, який працює на комп'ютері-компаньйоні Raspberry Pi.
- [Використання комп'ютера-компаньйона з контролерами Pixhawk](../companion_computer/pixhawk_companion.md)
- [Компаньйони комп'ютери > Програмне забезпечення комп'ютера-компаньйона](../companion_computer/index.md#companion-computer-software): Зауважте [MAVLink-Router](https://github.com/mavlink-router/mavlink-router), який можна налаштувати для маршрутизації трафіку MAVLink між послідовним портом та IP-лінком (або іншим інтерфейсом керування камерою).
