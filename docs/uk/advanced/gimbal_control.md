# Конфігурація Gimbal(стабілізатора)

Ця сторінка пояснює, як налаштувати та контролювати стабілізатор, що має приєднану камеру (або будь-який інший вантаж).

## Загальний огляд

PX4 містить універсальний драйвер керування кріпленням/гімбалем, який підтримує різні методи введення та виведення:

- Метод введення визначає протокол, який використовується для керування gimbal mount який керується PX4.
  Це може бути контролер RC, команда MAVLink, надіслана GCS, або обидва — автоматичне перемикання між ними.
- Метод виведення визначає, як PX4 взаємодіє з підключеним гімбалем.
  Рекомендований протокол — MAVLink v2, але ви також можете підключитися безпосередньо до виходного порту шим контролера польоту.

PX4 приймає вхідний сигнал і маршрутизує/перекладає його для відправлення на вивід.
Будь-який метод введення може бути обраний для керування будь-яким виводом.

Як вхід, так і вихід налаштовуються за допомогою параметрів.
Вхід встановлюється за допомогою параметра [MNT_MODE_IN](../advanced_config/parameter_reference.md#MNT_MODE_IN).
За замовчуванням це встановлено як `Вимкнено (-1)`, і драйвер не запускається.
Після вибору режиму введення перезавантажте транспортний засіб, щоб запустити драйвер кріплення.

Ви повинні встановити `MNT_MODE_IN` одним із наступних: `RC (1)`, `Протокол гімбала MAVLink v2 (4)` або `Авто (0)` (інші варіанти застарілі).
Якщо ви виберете `Авто (0)`, гімбал автоматично вибере вхід RC або MAVLink відповідно до останнього введення.
Зверніть увагу, що для автоматичного перемикання з MAVLink на RC потрібен великий рух ручкою!

Вихід налаштовується за допомогою параметра [MNT_MODE_OUT](../advanced_config/parameter_reference.md#MNT_MODE_OUT).
Усталеним налаштування виходу PXM є (`AUX (0)`).
Якщо ваш гімбал підтримує [Протокол гімбала MAVLink v2](https://mavlink.io/en/services/gimbal_v2.html), ви повинні замість цього вибрати `Протокол гімбала MAVLink v2 (2)`.

Повний список параметрів для налаштування драйвера кріплення можна знайти в [Довідці параметрів > Кріплення](../advanced_config/parameter_reference.md#mount).
Нижче наведено відповідні налаштування для декількох поширених конфігурацій гімбалів.

## MAVLink Gimbal (MNT_MODE_OUT=MAVLINK)

Кожен фізичний пристрій гімбала в системі повинен мати свій власний високорівневий менеджер гімбала, який може бути виявлений наземною станцією за допомогою протоколу MAVLink для гімбалів.
Наземна станція надсилає високорівневі команди [MAVLink Gimbal Manager](https://mavlink.io/en/services/gimbal_v2.html#gimbal-manager-messages) менеджеру гімбала, який вона хоче керувати, а менеджер, в свою чергу, надсилає відповідні команди нижчого рівня "пристрою гімбала", щоб керувати гімбалом.

PX4 може бути налаштований як менеджер гімбала для керування одним пристроєм гімбала (which can either be physically connected or be a MAVLink gimbal that implements the [gimbal device interface](https://mavlink.io/en/services/gimbal_v2.html#gimbal-device-messages)).

Щоб увімкнути гімбал по протоколу MAVLink, спочатку встановіть параметр [MNT_MODE_IN](../advanced_config/parameter_reference.md#MNT_MODE_IN) на `Протокол гімбала MAVLink v2` і [MNT_MODE_OUT](../advanced_config/parameter_reference.md#MNT_MODE_OUT) на `Протокол гімбала MAVLink v2`..

Гімбал можна підключити до _будь-якого вільного послідовного порту_, використовуючи інструкції у розділі [Послідовні пристрої MAVLink (GCS/OSD/Компаньйон)](../peripherals/mavlink_peripherals.md) (див. також [Конфігурація послідовного порту](../peripherals/serial_configuration.md#serial-port-configuration)).
Наприклад, якщо порт `TELEM2` на контролері польоту не використовується, ви можете підключити його до гімбала і встановити наступні параметри PX4:

- [MAV_1_CONFIG](../advanced_config/parameter_reference.md#MAV_1_CONFIG) на **TELEM2** (якщо `MAV_1_CONFIG` вже використовується для компаньйонного комп'ютера (скажімо), використовуйте `MAV_2_CONFIG`).
- [MAV_1_MODE](../advanced_config/parameter_reference.md#MAV_1_MODE) to **Gimbal**
- [MAV_1_FLOW_CTRL](../advanced_config/parameter_reference.md#MAV_1_FLOW_CTRL) to **Off (0)** (very few gimbals will have RST/CST wires connected).
- [MAV_1_FORWARD](../advanced_config/parameter_reference.md#MAV_1_FORWARD) to **Enabled** (Note strictly necessary as forwarding is enabled when `MAV_1_MODE` is set to Gimbal).
- [SER_TEL2_BAUD](../advanced_config/parameter_reference.md#SER_TEL2_BAUD) на рекомендовану виробником швидкість передачі даних.

### Підтримка декількох Gimbal

PX4 може автоматично створити менеджер гімбала для підключеного гімбала з PWM або першого пристрою гімбала MAVLink з тим самим ідентифікатором системи, який виявляється на будь-якому інтерфейсі.
Він не автоматично створює менеджер гімбала для будь-яких інших пристроїв гімбала MAVLink, які виявляються.

You can support additional MAVLink gimbals provided that they:

- Implement the gimbal _manager_ protocol.
- Становлять видимими для наземної станції та PX4 у мережі MAVLink.
  Це може вимагати налаштування пересилання трафіку між PX4, НЗП та гімбалем.
- Have a unique component id, and this component id must be in the range 7 - 255.

## Gimbal з FC PWM Output (MNT_MODE_OUT=AUX)

Gimbal також можна контролювати шляхом підключення до трьох портів польоту контролера польоту і налаштування режиму виводу в `MNT_MODE_OUT=AUX`.

Вихідні піни, які використовуються для керування гімбалем, встановлюються в [Конфігурація приводів > Виведення](../config/actuators.md#actuator-outputs), вибравши будь-які три невикористані виводи приводів та призначивши їм наступні функції виводу:

- `Gimbal Roll`: вихід керує поворотом gimbal.
- `Gimbal pitch`: вихід контролює крок підвісу.
- `Gimbal Yaw`: Output controls Gimbal yaw.

Наприклад, у вас можуть бути наступні налаштування для призначення кочення, тангажу та рида гімбала на виведення AUX1-3.

![Gimbal Actuator config](../../assets/config/actuators/qgc_actuators_gimbal.png)

PWM значення для використання при відблокованому, максимальному та мінімальному значеннях можна визначити так само, як і для інших сервоприводів, використовуючи [повзунки тесту приводу](../config/actuators.md#actuator-testing), щоб підтвердити, що кожний повзунок переміщує відповідну вісь, і змінюючи значення так, щоб гімбал знаходився у відповідному положенні при відблокованому стані, низькому і високому положенні повзунка.
Значення також можуть бути наведені у документації гімбала.

## Gimbal Control in Missions

[Gimbal Manager commands](https://mavlink.io/en/services/gimbal_v2.html#gimbal-manager-messages) may be used in missions if supported by the vehicle type.
For example [MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW) is supported in [multicopter mission mode](../flight_modes_mc/mission.md).

In theory you can address commands to a particular gimbal, specifying its component id using the "Gimbal device id" parameter.
However at time of writing (December 2024) this is [not supported](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/gimbal/input_mavlink.cpp#L889): all mission commands are sent to the (only) gimbal managed by the PX4 gimbal manager (if this is a MAVLink gimbal, it will be the gimbal with component id defined in the parameter [MNT_MAV_COMPID](../advanced_config/parameter_reference.md#MNT_MAV_COMPID), which is set by default to [MAV_COMP_ID_GIMBAL (154)](https://mavlink.io/en/messages/common.html#MAV_COMP_ID_GIMBAL)).

Gimbal movement is not immediate.
To ensure that the gimbal has time to move into position before the mission progresses to the next item (if gimbal feedback is not provided or lost), you should set [MIS_COMMAND_TOUT](../advanced_config/parameter_reference.md#MIS_COMMAND_TOUT) to be greater than the time taken for the gimbal to traverse its full range.
After this timeout the mission will proceed to the next item.

## Simulation / SITL

The following simulation environments come with a preconfigured simulated gimbal.
You can test the gimbal using the [QGroundControl UI](#qgc-testing) or by sending [driver commands](#driver-testing)

:::tip
If you only need to test the [gimbal driver](../modules/modules_driver.md#gimbal), then you can do this on any model or simulators.
Just make sure that the driver runs, using `gimbal start` in the MAVLink console, then configure the driver parameters.
:::

### Gazebo

To run the [Gazebo](../sim_gazebo_gz/index.md) simulation [Quadrotor(x500) with gimbal (Front-facing) in Gazebo](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-gimbal-front-facing), use:

```sh
make px4_sitl gz_x500_gimbal
```

![Quadrotor(x500) with gimbal (Front-facing) in Gazebo](../../assets/simulation/gazebo/vehicles/x500_gimbal.png)

### Gazebo Classic

To run the [Gazebo Classic](../sim_gazebo_classic/index.md) simulation [Typhoon H480 model](../sim_gazebo_classic/vehicles.md#typhoon-h480-hexrotor), use:

```sh
make px4_sitl gazebo-classic_typhoon_h480
```

![Typhoon H480 in Gazebo Classic](../../assets/simulation/gazebo_classic/vehicles/typhoon.jpg)

![Gazebo Gimbal Simulation](../../assets/simulation/gazebo_classic/gimbal-simulation.png)

## Тестування

### QGC Testing

The on-screen gimbal control can be used to move/test a connected MAVLink camera:

1. Start your preferred [simulator](#simulation-sitl) or connect to a real device.

2. Open QGroundControl and enable the on-screen camera control (Application settings).

  ![Quadrotor(x500) with gimbal (Front-facing) in Gazebo](../../assets/qgc/fly/gimbal_control_x500gz.png)

3. Make sure the vehicle is armed and flying, e.g. by entering with `commander takeoff`.

4. To change gimbal target position, click in the QGC GUI up, down, left, right, or use the buttons on the gimbal control (**Center**, **Tilt 90**, **Yaw lock**).

### Driver Testing

You can test a gimbal by sending `gimbal` driver commands in the [QGroundControl MAVLink Console](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_console.html):

1. Start your preferred [simulator](#simulation-sitl) or connect to a real device.
2. Open QGroundControl and connect to your vehicle.
3. Open the MAVLink Console using the menu: **Analyze > Mavlink Console**.
4. Make sure the vehicle is armed and flying, e.g. by entering the command: `commander takeoff`.

To check gimbal status enter:

```sh
gimbal status
```

To set the gimbal yaw to 30 degrees, use the command:

```sh
gimbal test yaw 30
```

More generally, you can set the angle or angular rate command using a command with this format:

```sh
gimbal test <axis> <value>
```

- `axis`:
  - `<roll|pitch|yaw>` for angles
  - `<rollrate|pitchrate|yawrate>` for angular rates
- `value`:
  - `<degrees>` for angles
  - `<degrees / second>` for angular rates

To set the MAVLink component that is in primary control of the gimbal:

```sh
gimbal primary-control <sys_id> <comp_id>
```

- `sys_id`: MAVLink system ID
- `comp_id`: MAVLink component ID

For other commands, see the [`gimbal`](../modules/modules_driver.md#gimbal) driver module document.

### MAVLink Testing

The gimbal can be tested by sending MAVLink gimbal manager commands using [MAVSDK](../robotics/mavsdk.md) or some other MAVLink library.

Зверніть увагу, що симульований гімбал стабілізується самостійно, тому якщо ви надсилаєте команди MAVLink, встановіть прапори `стабілізації` на значення `false`.
