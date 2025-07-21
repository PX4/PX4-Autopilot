# Керівництво з перенесення контролера польоту

This topic is for developers who want to port PX4 to work with _new_ flight controller hardware.

## Архітектура системи PX4

PX4 consists of two main layers: The [board support and middleware layer](../middleware/index.md) on top of the host OS (NuttX, Linux or any other POSIX platform like Mac OS), and the applications (Flight Stack in [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules)\). Please reference the [PX4 Architectural Overview](../concept/architecture.md) for more information.

Цей посібник спрямований лише на операційну систему хоста та проміжне програмне забезпечення, оскільки програми/стек польоту будуть працювати на будь-якій цільовій платі.

## Налаштування режиму польоту файлової структури

Board startup and configuration files are located under [/boards](https://github.com/PX4/PX4-Autopilot/tree/main/boards/) in each board's vendor-specific directory (i.e. **boards/_VENDOR_/_MODEL_/**).

Наприклад, для FMUv5:

- (All) Board-specific files: [/boards/px4/fmu-v5](https://github.com/PX4/PX4-Autopilot/tree/main/boards/px4/fmu-v5).<!-- NEED px4_version -->
- Build configuration: [/boards/px4/fmu-v5/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/default.px4board).<!-- NEED px4_version -->
- Board-specific initialisation file: [/boards/px4/fmu-v5/init/rc.board_defaults](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/init/rc.board_defaults) <!-- NEED px4_version -->
  - A board-specific initialisation file is automatically included in startup scripts if found under the boards directory at **init/rc.board**.
  - Файл використовується для запуску сенсорів (та інших речей), які існують лише на конкретній платі.
    Це також може бути використано для встановлення параметрів за замовчуванням дошки, відображень UART та будь-яких інших виняткових випадків.
  - Для FMUv5 ви можете побачити, як запускаються всі датчики Pixhawk 4, а також встановлюється більший LOGGER_BUF.

## Конфігурація операційної системи хоста

Цей розділ описує призначення та місцезнаходження файлів конфігурації, необхідних для кожної підтримуваної операційної системи хоста, щоб перенести їх на нове апаратне засіб керування польотом.

### NuttX

See [NuttX Board Porting Guide](porting_guide_nuttx.md).

### Linux

Плати Linux не включають ОС та конфігурацію ядра.
Ці дані вже надаються зображенням Linux, доступним для плати (яке повинно підтримувати інерційні сенсори з коробки).

- [boards/px4/raspberrypi/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/raspberrypi/default.px4board) - RPi cross-compilation. <!-- NEED px4_version -->

## Компоненти та конфігурація проміжного програмного забезпечення

Цей розділ описує різноманітні компоненти проміжного програмного забезпечення та необхідні оновлення файлів конфігурації для перенесення їх на нове апаратне засіб керування польотом.

### QuRT / Шестикутник

- The start script is located in [posix-configs/](https://github.com/PX4/PX4-Autopilot/tree/main/posix-configs). <!-- NEED px4_version -->
- Конфігурація ОС є частиною стандартного образу Linux (TODO: Вказати місце розташування ОБРАЗУ LINUX та інструкції щодо прошивки).
- The PX4 middleware configuration is located in [src/boards](https://github.com/PX4/PX4-Autopilot/tree/main/boards). <!-- NEED px4_version --> TODO: ADD BUS CONFIG

## Рекомендації з підключення RC UART

Зазвичай рекомендується підключати RC через окремі піни RX та TX до мікроконтролера.
Якщо, проте, RX та TX з'єднані разом, UART повинен бути переведений в режим одножильного кабелю, щоб уникнути будь-яких конфліктів.
Це робиться за допомогою конфігураційної дошки та файлів маніфесту.
One example is [px4fmu-v5](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/src/manifest.c). <!-- NEED px4_version -->

## Офіційно Підтримуване Обладнання

The PX4 project supports and maintains the [FMU standard reference hardware](../hardware/reference_design.md) and any boards that are compatible with the standard.
This includes the [Pixhawk-series](../flight_controller/pixhawk_series.md) (see the user guide for a [full list of officially supported hardware](../flight_controller/index.md)).

Кожна офіційно підтримувана дошка користується наступними перевагами:

- Порт PX4 доступний у сховищі PX4
- Automatic firmware builds that are accessible from _QGroundControl_
- Сумісність з рештою екосистеми
- Автоматизовані перевірки через CI - безпека залишається найважливішою для цієї спільноти
- [Flight testing](../test_and_ci/test_flights.md)

We encourage board manufacturers to aim for full compatibility with the [FMU spec](https://pixhawk.org/).
З повною сумісністю ви користуєтеся постійним розвитком PX4 щодня, але не маєте жодних витрат на обслуговування, які виникають у зв'язку з відхиленнями від специфікації.

:::tip
Manufacturers should carefully consider the cost of maintenance before deviating from the specification (the cost to the manufacturer is proportional to the level of divergence).
:::

We welcome any individual or company to submit their port for inclusion in our supported hardware, provided they are willing to follow our [Code of Conduct](https://github.com/PX4/PX4-Autopilot/blob/main/CODE_OF_CONDUCT.md) and work with the Dev Team to provide a safe and fulfilling PX4 experience to their customers.

Також важливо зауважити, що команда розробників PX4 має відповідальність випускати безпечне програмне забезпечення, тому ми вимагаємо, щоб будь-який виробник плати зобов'язався витрачати всі необхідні ресурси для підтримки їхнього порту в актуальному стані та працездатному.

Якщо ви хочете, щоб ваша дошка була офіційно підтримана в PX4:

- Ваше обладнання повинно бути доступним на ринку (тобто його можна придбати будь-якому розробнику без обмежень).
- Hardware must be made available to the PX4 Dev Team so that they can validate the port (contact [lorenz@px4.io](mailto:lorenz@px4.io) for guidance on where to ship hardware for testing).
- The board must pass full [test suite](../test_and_ci/index.md) and [flight testing](../test_and_ci/test_flights.md).

**The PX4 project reserves the right to refuse acceptance of new ports (or remove current ports) for failure to meet the requirements set by the project.**

You can reach out to the core developer team and community on the [official support channels](../contribute/support.md).

## Пов'язана інформація

- [Device Drivers](../middleware/drivers.md) - How to support new peripheral hardware (device drivers)
- [Building the Code](../dev_setup/building_px4.md) - How to build source and upload firmware
- Підтримувані автопілоти включають:
  - [Autopilot Hardware](../flight_controller/index.md)
  - [Supported boards list](https://github.com/PX4/PX4-Autopilot/#supported-hardware) (Github) - Boards for which PX4-Autopilot has specific code
- [Supported Peripherals](../peripherals/index.md)
