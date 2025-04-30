# Архітектура/Інтеграція камери PX4

Ця тема надає короткий огляд того, як підтримується камера PX4 **реалізована**.

:::info
Дивіться [Камера](../camera/index.md) для отримання інформації щодо _використання_ камер.
:::

## Загальний огляд

PX4 інтегрується з трьома типами камер:

- [Камери MAVLink](../camera/mavlink_v2_camera.md), які підтримують [Протокол Камери v2](https://mavlink.io/en/services/camera.html) (**РЕКОМЕНДОВАНО**).
- [Прості камери MAVLink](../camera/mavlink_v1_camera.md), які підтримують старший [Протокол камери v1](https://mavlink.io/en/services/camera.html).
- [Камери, підключені до виходів контролера польоту](../camera/fc_connected_camera.md), які керуються з використанням [протоколу камери v1](https://mavlink.io/en/services/camera.html).

Всі ці камери повинні реагувати на команди MAVLink, отримані через MAVLink або знайдені в місіях (конкретний протокол залежить від камери).

Використана широка архітектура описана нижче.

## Камери MAVLink (Протокол камери v2)

PX4 не має конкретної обробки для [MAVLink камер](../camera/mavlink_v2_camera.md), які підтримують [Протокол камери v2](https://mavlink.io/en/services/camera.html), крім [переемітації елементів камери в місіях як команди](#camera-commands-in-missions)

Очікується, що земні станції будуть спілкуватися з цими камерами безпосередньо, щоб надсилати команди.
PX4 повинен бути налаштований для маршрутизації трафіку MAVLink між камерою та земними станціями за необхідності.

:::info
Модулі `camera_trigger`, `camera_capture` та `camera_feedback` не використовуються з цією камерою.
:::

## Камери, підключені за допомогою FC

Камери, підключені до виходів контролера польоту, потребують PX4 для активації виходів для запуску камери, і можуть потребувати PX4 для виявлення, коли [пін захоплення камери](../camera/fc_connected_camera.md#camera-capture-configuration) був спрацьований камерним гарячим черевиком (для покращення часу реєстрації захопленої камери).

Ця робота виконується трьома компонентами PX4: [`camera_trigger` driver](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/camera_trigger), [`camera_capture` driver](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/camera_capture), модуль `camera-feedback` (../modules/modules_system.md#camera-feedback).

`camera_trigger` підписується на тему [VehicleCommand](../msg_docs/VehicleCommand.md) та відстежує оновлення в [команди, які він підтримує](../camera/fc_connected_camera.md#mavlink-command-interface).
Ці оновлення відбуваються, коли отримано команду через MAVLink або коли [елемент камери досягнутий у місії](#camera-commands-in-missions).

Команди увімкнення та вимкнення спрацьовують, а також налаштовують спрацьовування за часовими та відстаневими інтервалами.
Водій відстежує ці інтервали, і коли це потрібно, спрацьовують виходи.
Водій публікує тему [CameraTrigger](../msg_docs/CameraTrigger.md) (з полем `feedback`, встановленим на `false`), що спричиняє відсилання повідомлення MAVLink [CAMERA_TRIGGER](https://mavlink.io/en/messages/common.html#CAMERA_TRIGGER).

Драйвер `camera_capture`, якщо він увімкнений, слідкує за підключенням камери та, спрацьовуючи, оприлюднює тему [CameraTrigger](../msg_docs/CameraTrigger.md) (з полем `feedback`, встановленим на `true`), що також спричиняє відсилання повідомлення MAVLink [CAMERA_TRIGGER](https://mavlink.io/en/messages/common.html#CAMERA_TRIGGER).

Модуль 'camera_feedback' відстежує оновлення теми [CameraTrigger](../msg_docs/CameraTrigger.md), та публікує тему [CameraCapture](../msg_docs/CameraCapture.md) для оновлення `CameraTrigger` від _або_ `camera_trigger`, або `camera_capture`.
Інформація, яка використовується, залежить від того, чи увімкнений пін захоплення камери, і значення поля `CameraTrigger.feedback`.
Ця тема `CameraCapture` реєструється і може бути використана для отримання часу знімку.

## Камери MAVLink (Протокол камери v1)

[MAVLink камери, які підтримують старіший Camera Protocol v1](../camera/mavlink_v1_camera.md) інтегруються так само, як і [Камери, що підключаються через FC](#fc-connected-cameras).

`camera_trigger` підписується на тему [VehicleCommand](../msg_docs/VehicleCommand.md) та відстежує оновлення в [команди, які він підтримує](../camera/fc_connected_camera.md#mavlink-command-interface).
Це відбувається, коли команда надходить через MAVLink або коли [елемент камери знаходиться в місіях](#camera-commands-in-missions).

Команди увімкнення та вимкнення спрацьовують, а також налаштовують спрацьовування за часовими та відстаневими інтервалами.
Водій відслідковує ці інтервали, але з "MAVLink backend" не потрібно фактично спрацьовувати будь-які виводи (оскільки команди переадресовуються на камеру).
Коли камера спрацьовує, водій публікує тему [CameraTrigger](../msg_docs/CameraTrigger.md) (з полем `feedback`, встановленим на `false`), що спричиняє відсилання повідомлення MAVLink [CAMERA_TRIGGER](https://mavlink.io/en/messages/common.html#CAMERA_TRIGGER).
Модуль `camera_feedback` повинен, а потім записати відповідну тему `CameraCapture`.

## Команди камери у місіях

PX4 повторно видає пункти камери, знайдені в місіях, як команди MAVLink для всіх підтримуваних [Протоколів камери v2] (https://mavlink.io/en/services/camera.html) та [Протоколів камери v1] (https://mavlink.io/en/services/camera.html).
Системний ідентифікатор випущених команд співпадає з ідентифікатором автопілота.
Ідентифікатор компоненту команд може відрізнятися, але зазвичай вони надсилаються на [MAV_COMP_ID_CAMERA (100)](https://mavlink.io/en/messages/common.html#MAV_COMP_ID_CAMERA) або [MAV_COMP_ID_ALL](https://mavlink.io/en/messages/common.html#MAV_COMP_ID_ALL) (дивіться документацію камери для того, який ID використовується у кожному випадку).

Команди випромінюються незалежно від того, чи є підключена камера будь-якого типу, за умови наявності каналу MAVLink для випромінювання.

:::info
Загалом PX4 повторно видає всі команди місій, які можуть бути використані зовнішніми компонентами MAVLink, такими як джимбалі.
Команди для точок маршруту та умовної поведінки не випускаються.
:::

Розділи нижче висвітлюють цікаві частини кодової бази

### Команди, підтримувані у місіях

Команди, підтримувані в місіях, включаючи команди камери, показані в цих методах:

- [`bool FeasibilityChecker::checkMissionItemValidity(mission_item_s &mission_item, const int current_index)`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/MissionFeasibility/FeasibilityChecker.cpp#L257-L306)
- [`format_mavlink_mission_item()`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_mission.cpp#L1672-L1693)

### Потік для повторної відправки команд камери, знайденої у місіях

- [`void Mission::setActiveMissionItems()`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/mission.cpp#L187-L281)
  - Предмети місії виконуються, коли вони активовані.
  - `issue_command(_mission_item)` викликається в кінці цього, щоб відправити поточну непунктову команду
    - [`MissionBlock::видача_команди(const mission_item_s &item)`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/mission_block.cpp#L543-L562)
      - Створює команду для місії транспортного засобу, а потім викликає `publish_vehicle_command` для публікації її (`_navigator->publish_vehicle_command(vehicle_command);`)
        - [`void Navigator::publish_vehicle_command(vehicle_command_s &vehicle_command)`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/navigator_main.cpp#L1395)
          - Для деяких команд камери це встановлює ідентифікатор компонента на ідентифікатор компонента камери (`vehicle_command.target_component = 100; // MAV_COMP_ID_CAMERA`)
          - Усі інші просто публікуються під стандартний компонент ID.
          - Тема UORB `VehicleCommand` публікується.

Код потоку MAVLink відстежує зміни у темі `VehicleCommand` та публікує їх через MAVLink.
Команда MAVLink надсилається незалежно від того, чи є камера камерою MAVLink, чи підключена до контролера польоту.

Драйвер `camera_trigger`, якщо включений, також контролює зміни для `VehicleCommand`.
Якщо він налаштований з внутрішнім інтерфейсом для камери, підключеної до вихідних сигналів контролера польоту, він відповідним чином активує ці вихідні сигнали.

## Логування

Теми `CameraCapture` реєструються при оновленні `CameraTrigger`.
Введений тема буде залежати від того, чи увімкнений пін захоплення камери.

Зверніть увагу, що події захоплення камери не реєструються, коли використовується [камери MAVLink, що підтримують протокол камери v2](../camera/mavlink_v2_camera.md), оскільки відповідні події спрацювання тригера не генеруються в межах PX4.

## Дивіться також

- Драйвер Тригера камери: [вихідний код](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/camera_trigger) <!-- no module doc -->
- Драйвер захоплення камери: [вихідний код](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/camera_capture) <!-- no module doc -->
