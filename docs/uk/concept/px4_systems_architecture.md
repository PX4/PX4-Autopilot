# Архітектура системи PX4

Нижче наведено огляд апаратного та програмного забезпечення на PX4 для двох "типових" систем PX4; перша має лише польотний контролер, а друга має польотний контролер і супутній комп'ютер (також відомий як "комп'ютер політних завдань").

:::info
[Огляд архітектури PX4](../concept/architecture.md) надає інформацію про набір польотного програмного забезпечення та проміжного ПЗ.
Зовнішні API охоплено в розділах [ROS](../ros/README.md) та [MAVSDK](https://mavsdk.mavlink.io/main/en/).
:::

## Лише польотний контролер

На діаграмі нижче показано загальний огляд типової "простої" системи PX4 на основі польотного контролера.

![PX4 architecture - FC only system](../../assets/diagrams/px4_arch_fc.svg)

<!-- Source for drawing: https://docs.google.com/drawings/d/1_2n43WrbkWTs1kz0w0avVEeebJbfTj5SSqvCmvSOBdU/edit -->

Апаратне забезпечення складається з

- [Політного контролера](../flight_controller/README.md) (запускає набір польотного ПЗ PX4). Часто включає внутрішні ІВП, компас та барометр.
- [Електронного регулятора ходу двигунів](../peripherals/esc_motors.md) під'єднаного до [виводів ШІМ](../peripherals/pwm_escs_and_servo.md), [DroneCAN](../dronecan/escs.md) (DroneCAN дозволяє двонапрямну комунікацію, не в одному напрямку як показано) або іншої шини.
- Sensors ([GPS](../gps_compass/index.md), [compass](../gps_compass/index.md), distance sensors, barometers, optical flow, barometers, ADSB transponders, etc.) connected via I2C, SPI, CAN, UART etc.
- [Камера](../camera/index.md) або інше корисне навантаження. Камери можуть бути підключені до ШІМ виходів або за допомогою MAVLink.
- [Telemetry radios](../telemetry/index.md) for connecting to a ground station computer/software.
- [Система радіо керування](../getting_started/rc_transmitter_receiver.md) для ручного керування

Ліва частина діаграми показує набір програмного забезпечення, що по горизонталі (приблизно) вирівняно згідно з апаратними частинами діаграми.

- На комп'ютері наземної станції зазвичай працює [QGroundControl](../getting_started/px4_basic_concepts.md#qgc) (або інше програмне забезпечення наземної станції).
  It may also run robotics software like [MAVSDK](https://mavsdk.mavlink.io/) or [ROS](../ros/index.md).
- The PX4 flight stack running on the flight controller includes [drivers](../modules/modules_driver.md), [comms modules](../modules/modules_communication.md), [controllers](../modules/modules_controller.md), [estimators](../modules/modules_controller.md) and other [middleware and system modules](../modules/modules_main.md).

## Польотний контролер та супутній комп'ютер

На діаграмі показано систему PX4, яка включає як політний контролер, так і супутній комп'ютер (тут згадується як "комп'ютер політного завдання").

![PX4 architecture - FC + Companion Computer](../../assets/diagrams/px4_arch_fc_companion.svg)

<!-- source for drawing: https://docs.google.com/drawings/d/1zFtvA_B-BmfmxFmAd-XIvAZ-jRqOydj0aBtqSolBcqI/edit -->

Польотний контролер виконує звичайний набір ПЗ PX4, тоді як супутній комп'ютер забезпечує просунуті функції наприклад [уникнення об'єктів](../computer_vision/obstacle_avoidance.md) та [запобігання зіткненням](../computer_vision/collision_prevention.md).
Дві системи з'єднані за допомогою швидкого послідовного або IP-з'єднання і зазвичай взаємодіють за допомогою протоколу [MAVLink](https://mavlink.io/en/).
Зв'язок з наземними станціями та хмарою зазвичай направляється через супутній комп'ютер (наприклад, за допомогою [MAVLink Router](https://github.com/mavlink-router/mavlink-router) від Intel).

Системи PX4 зазвичай виконують ОС Linux на супутньому комп'ютері (тому що проєкт [PX4/PX4-Avoidance](https://github. com/PX4/PX4-Avoidance) поставляє засновані на ROS бібліотеки уникнення розроблені для Linux).
Linux є набагато кращою платформою для "загальної" розробки програмного забезпечення, ніж NuttX; у Linux багато розробників і вже написано багато корисного програмного забезпечення (наприклад для комп'ютерного бачення, зв'язку, інтеграції з хмарою, апаратні драйвери).
Супутні комп'ютери іноді працюють на Android з тієї ж причини.

:::info
Діаграма показує підключення до хмарних або наземних станцій через LTE, підхід, який був використаний в ряді систем заснованих на PX4.
PX4 не надає програмного забезпечення для LTE та/або хмарної інтеграції (це потребує додаткової розробки).
:::
